/*
 * Rackmount Replacement — Teensy 4.1
 * -----------------------------------
 * Measures two ~150–300 Hz sinusoidal inputs (L, R) and regenerates a set of
 * analog (PWM + RC lowpass) and TTL outputs on the enclosure's BNC jacks.
 *
 * Signal path (hardware):
 *   detector -> INA828 (x20) -> panel ladder attenuator (x0.2..x1.0)
 *            -> OPA2134 (x100) -> JUMPER (or external BNC, +/-1 V)
 *            -> summing level shift (slope 1.0, centred on VDD_SCF/2)
 *            -> MAX7424 SCF (5-pole Butterworth, fc = fCLK/100)
 *            -> 1k + 6.8nF -> Teensy ADC
 *
 * INPUTS:
 *   A0 (pin 14)  L, centred on VDD_SCF/2, ~2 Vpp full scale
 *   A1 (pin 15)  R, same
 *   A2 (pin 16)  Front-panel LPF cutoff pot (rheostat in a divider to 3.3 V)
 *   Pin 10       Front-panel L/R selector switch (to GND, internal pullup)
 *                OPEN (HIGH) = L selected, CLOSED (LOW) = R selected
 *
 * ANALOG OUTPUTS (PWM @ ~146.5 kHz, 10-bit):
 *   Pin 2   L peak-peak amplitude
 *   Pin 3   R peak-peak amplitude
 *   Pin 4   (L + R)
 *   Pin 5   (L - R), offset so zero difference = mid-scale
 *   Pin 6   Fundamental frequency of selected channel
 *
 * DIGITAL (TTL) OUTPUTS:
 *   Pin 7   SYNC   — one pulse per cycle of the selected channel
 *   Pin 8   L FLIP — one pulse per L cycle, timed to the waveform minimum
 *   Pin 9   R FLIP — one pulse per R cycle, timed to the waveform minimum
 *
 * CLOCK OUTPUT:
 *   Pin 33  SCF clock, 50% duty, 100 kHz–1.1 MHz -> CLK of BOTH MAX7424s
 *
 *   !! PIN 36 MUST BE ON A DIFFERENT FlexPWM MODULE THAN PINS 2–6 !!
 *   analogWriteFrequency() acts on the whole timer module, so if the clock
 *   pin shares a module with the analog outputs, turning the cutoff knob will
 *   drag their 146.5 kHz carrier with it. Symptom: output ripple changes when
 *   you touch the LPF knob.
 */

#include <Arduino.h>

// ------------------------- Pin assignments -------------------------
constexpr uint8_t PIN_ADC_L    = A1;   // pin 15
constexpr uint8_t PIN_ADC_R    = A0;   // pin 14
constexpr uint8_t PIN_LPF_POT  = A2;   // pin 16
constexpr uint8_t PIN_SWITCH   = 10;

constexpr uint8_t PIN_OUT_AMP_L = 2;
constexpr uint8_t PIN_OUT_AMP_R = 3;
constexpr uint8_t PIN_OUT_SUM   = 4;
constexpr uint8_t PIN_OUT_DIFF  = 5;
constexpr uint8_t PIN_OUT_FREQ  = 6;

constexpr uint8_t PIN_SYNC    = 7;
constexpr uint8_t PIN_FLIP_L  = 8;
constexpr uint8_t PIN_FLIP_R  = 9;

constexpr uint8_t PIN_SCF_CLK = 36;

// ------------------------- Tuning constants ------------------------
constexpr float    FS_HZ        = 50000.0f;          // ADC sample rate/channel
constexpr uint32_t TS_US        = (uint32_t)(1e6f / FS_HZ);

constexpr uint8_t  ADC_BITS     = 12;
constexpr float    VREF         = 3.3f;

constexpr uint8_t  PWM_BITS     = 10;
constexpr float    PWM_FREQ_HZ  = 146484.375f;       // 150 MHz / 1024

// Hardware gain of the level-shift stage: ADC volts per volt at the jumper.
constexpr float    FRONTEND_GAIN = 1.0f;

// Comparator hysteresis in ADC counts (~64 mV).
constexpr int32_t  HYST_COUNTS  = 80;

constexpr uint32_t PERIOD_MIN_US = 2500;             // 400 Hz
constexpr uint32_t PERIOD_MAX_US = 10000;            // 100 Hz
constexpr int32_t  MIN_PP_COUNTS = 200;              // ~160 mVpp
constexpr uint32_t PULSE_US      = 100;

// ---------------- Switched-capacitor filter control ----------------
// MAX7424: 5th-order Butterworth, fCLK = 100 x fC.
constexpr float SCF_CLK_RATIO = 100.0f;
constexpr float FC_MIN_HZ     = 1000.0f;             // knob fully CCW
constexpr float FC_MAX_HZ     = 11000.0f;            // knob fully CW

// Pot calibration — RUN THE SERIAL MONITOR, turn the knob to each stop, and
// paste the raw ADC values you see here. Anything is fine; the mapping below
// normalises it, so the pot's own value and taper don't matter.
constexpr int32_t POT_ADC_MIN = 88;
constexpr int32_t POT_ADC_MAX = 2020;

// Only re-tune the clock when the knob has actually moved this far (fraction
// of full travel). Stops ADC noise from continuously dithering the cutoff.
constexpr float POT_DEADBAND  = 0.004f;
constexpr float POT_SMOOTHING = 0.08f;               // EMA coefficient

// -------- Output scaling (EDIT THESE to match downstream gear) -----
constexpr float AMP_GAIN    = 1.5f;       // Vout = AMP_GAIN * Vpp_in
constexpr float SUM_GAIN    = 0.75f;
constexpr float DIFF_GAIN   = 0.75f;
constexpr float DIFF_OFFSET = VREF / 2;

constexpr float FREQ_MIN_HZ = 100.0f;     // maps to 0 V
constexpr float FREQ_MAX_HZ = 400.0f;     // maps to VREF

// ------------------------- Per-channel state -----------------------
struct Channel {
  bool     compHigh   = false;
  int32_t  midpoint   = 2048;
  int32_t  curMin     =  1 << 14;
  int32_t  curMax     = -1;
  uint32_t lastRiseUs = 0;

  volatile int32_t  ppCounts   = 0;
  volatile uint32_t periodUs   = 0;
  volatile bool     freshCycle = false;

  volatile uint32_t flipDueUs  = 0;
  volatile bool     flipArmed  = false;
  volatile uint32_t flipEndUs  = 0;
  volatile bool     flipActive = false;
};

Channel chL, chR;

volatile uint32_t syncEndUs  = 0;
volatile bool     syncActive = false;
volatile bool     selectR    = false;
volatile uint16_t potRaw     = 0;

float scfCutoffHz = FC_MAX_HZ;    // last applied cutoff, for reporting

IntervalTimer sampleTimer;

// ------------------------- Helpers ---------------------------------
static inline uint16_t voltsToPwm(float v) {
  if (v < 0) v = 0;
  if (v > VREF) v = VREF;
  return (uint16_t)(v / VREF * ((1 << PWM_BITS) - 1) + 0.5f);
}

static inline float countsToVolts(int32_t c) {
  return (float)c * VREF / ((1 << ADC_BITS) - 1);
}

static void updateAmplitudeOutputs() {
  float lpp = countsToVolts(chL.ppCounts) / FRONTEND_GAIN;
  float rpp = countsToVolts(chR.ppCounts) / FRONTEND_GAIN;

  analogWrite(PIN_OUT_AMP_L, voltsToPwm(AMP_GAIN * lpp));
  analogWrite(PIN_OUT_AMP_R, voltsToPwm(AMP_GAIN * rpp));
  analogWrite(PIN_OUT_SUM,   voltsToPwm(SUM_GAIN * (lpp + rpp)));
  analogWrite(PIN_OUT_DIFF,  voltsToPwm(DIFF_OFFSET + DIFF_GAIN * (lpp - rpp)));
}

static void updateFrequencyOutput(uint32_t periodUs) {
  if (periodUs == 0) return;
  float f = 1e6f / (float)periodUs;
  float v = (f - FREQ_MIN_HZ) / (FREQ_MAX_HZ - FREQ_MIN_HZ) * VREF;
  analogWrite(PIN_OUT_FREQ, voltsToPwm(v));
}

// Map knob fraction (0..1) to a cutoff and retune the SCF clock.
// The mapping is exponential so each degree of rotation is a constant
// *percentage* change in cutoff — the knob feels the same at both ends
// regardless of whether the pot itself is linear or log taper.
static void applyCutoff(float x) {
  if (x < 0) x = 0;
  if (x > 1) x = 1;

  float fc   = FC_MIN_HZ * powf(FC_MAX_HZ / FC_MIN_HZ, x);
  float fclk = fc * SCF_CLK_RATIO;

  analogWriteFrequency(PIN_SCF_CLK, fclk);
  analogWrite(PIN_SCF_CLK, (1 << PWM_BITS) / 2);   // 50% duty
  scfCutoffHz = fc;
}

// ------------------------- Sampling ISR ----------------------------
static void processSample(Channel &ch, int32_t raw, uint32_t now,
                          bool isSelected, uint8_t flipPin) {
  if (raw < ch.curMin) ch.curMin = raw;
  if (raw > ch.curMax) ch.curMax = raw;

  bool rose = false;
  if (!ch.compHigh && raw > ch.midpoint + HYST_COUNTS) {
    ch.compHigh = true;
    rose = true;
  } else if (ch.compHigh && raw < ch.midpoint - HYST_COUNTS) {
    ch.compHigh = false;
  }

  if (rose) {
    uint32_t period = now - ch.lastRiseUs;
    ch.lastRiseUs = now;

    int32_t pp = ch.curMax - ch.curMin;
    bool confident = (period >= PERIOD_MIN_US && period <= PERIOD_MAX_US &&
                      pp >= MIN_PP_COUNTS);

    if (confident) {
      ch.ppCounts   = pp;
      ch.periodUs   = period;
      ch.midpoint   = (ch.curMax + ch.curMin) / 2;
      ch.freshCycle = true;

      if (isSelected) {
        digitalWriteFast(PIN_SYNC, HIGH);
        syncEndUs  = now + PULSE_US;
        syncActive = true;
      }

      ch.flipDueUs = now + (period * 3) / 4;
      ch.flipArmed = true;
    }

    ch.curMin =  1 << 14;
    ch.curMax = -1;
  }

  if (ch.flipArmed && (int32_t)(now - ch.flipDueUs) >= 0) {
    ch.flipArmed = false;
    digitalWriteFast(flipPin, HIGH);
    ch.flipEndUs  = now + PULSE_US;
    ch.flipActive = true;
  }
  if (ch.flipActive && (int32_t)(now - ch.flipEndUs) >= 0) {
    ch.flipActive = false;
    digitalWriteFast(flipPin, LOW);
  }
}

void sampleISR() {
  uint32_t now = micros();
  int32_t rawL = analogRead(PIN_ADC_L);
  int32_t rawR = analogRead(PIN_ADC_R);

  bool selR = selectR;
  processSample(chL, rawL, now, !selR, PIN_FLIP_L);
  processSample(chR, rawR, now,  selR, PIN_FLIP_R);

  if (syncActive && (int32_t)(now - syncEndUs) >= 0) {
    syncActive = false;
    digitalWriteFast(PIN_SYNC, LOW);
  }

  // The pot MUST be sampled here, not in loop(). Teensyduino's analogRead is
  // not reentrant — calling it from loop() while this ISR is also using the
  // ADC corrupts conversions. Decimate hard: once every 2500 ISRs is 20 Hz,
  // far faster than anyone turns a knob, and costs ~2 us every 50 ms.
  static uint32_t potDecim = 0;
  if (++potDecim >= 2500) {
    potDecim = 0;
    potRaw = analogRead(PIN_LPF_POT);
  }
}

// ------------------------- Setup / loop ----------------------------
void setup() {
  analogReadResolution(ADC_BITS);
  analogReadAveraging(1);

  analogWriteResolution(PWM_BITS);
  const uint8_t pwmPins[] = {PIN_OUT_AMP_L, PIN_OUT_AMP_R, PIN_OUT_SUM,
                             PIN_OUT_DIFF, PIN_OUT_FREQ};
  for (uint8_t p : pwmPins) {
    pinMode(p, OUTPUT);
    analogWriteFrequency(p, PWM_FREQ_HZ);
    analogWrite(p, 0);
  }

  // Bring the SCF clock up before anything else needs the filters running.
  pinMode(PIN_SCF_CLK, OUTPUT);
  applyCutoff(1.0f);                 // default to widest cutoff

  pinMode(PIN_SYNC,   OUTPUT);  digitalWriteFast(PIN_SYNC,   LOW);
  pinMode(PIN_FLIP_L, OUTPUT);  digitalWriteFast(PIN_FLIP_L, LOW);
  pinMode(PIN_FLIP_R, OUTPUT);  digitalWriteFast(PIN_FLIP_R, LOW);

  pinMode(PIN_SWITCH, INPUT_PULLUP);
  selectR = (digitalRead(PIN_SWITCH) == LOW);

  Serial.begin(115200);

  sampleTimer.begin(sampleISR, TS_US);
  sampleTimer.priority(64);
}

void loop() {
  // ---- Debounce the front-panel L/R switch (~10 ms) ----
  static uint32_t lastSwChangeMs = 0;
  static bool rawPrev = false;
  bool rawNow = (digitalRead(PIN_SWITCH) == LOW);
  if (rawNow != rawPrev) {
    rawPrev = rawNow;
    lastSwChangeMs = millis();
  } else if (rawNow != selectR && millis() - lastSwChangeMs > 10) {
    selectR = rawNow;
  }

  // ---- Track the LPF cutoff knob ----
  static float potSmoothed = -1.0f;
  static float potApplied  = -1.0f;
  {
    float x = (float)((int32_t)potRaw - POT_ADC_MIN) /
              (float)(POT_ADC_MAX - POT_ADC_MIN);
    if (x < 0) x = 0;
    if (x > 1) x = 1;

    if (potSmoothed < 0) potSmoothed = x;             // first pass
    potSmoothed += POT_SMOOTHING * (x - potSmoothed);

    if (potApplied < 0 || fabsf(potSmoothed - potApplied) > POT_DEADBAND) {
      potApplied = potSmoothed;
      applyCutoff(potApplied);
    }
  }

  // ---- Refresh analog outputs when either channel completes a cycle ----
  if (chL.freshCycle || chR.freshCycle) {
    noInterrupts();
    chL.freshCycle = false;
    chR.freshCycle = false;
    uint32_t perL = chL.periodUs;
    uint32_t perR = chR.periodUs;
    interrupts();

    updateAmplitudeOutputs();
    updateFrequencyOutput(selectR ? perR : perL);
  }

  // ---- 2 Hz status printout over USB serial ----
  static uint32_t lastPrintMs = 0;
  if (millis() - lastPrintMs > 500) {
    lastPrintMs = millis();
    Serial.printf("L: %.3f Vpp @ %.1f Hz | R: %.3f Vpp @ %.1f Hz | "
                  "sel=%c | pot=%4u  fc=%.0f Hz (fclk=%.0f Hz)\n",
                  countsToVolts(chL.ppCounts) / FRONTEND_GAIN,
                  chL.periodUs ? 1e6f / chL.periodUs : 0.0f,
                  countsToVolts(chR.ppCounts) / FRONTEND_GAIN,
                  chR.periodUs ? 1e6f / chR.periodUs : 0.0f,
                  selectR ? 'R' : 'L',
                  potRaw, scfCutoffHz, scfCutoffHz * SCF_CLK_RATIO);
  }
}
