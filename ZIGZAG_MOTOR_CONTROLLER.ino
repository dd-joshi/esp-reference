/*
===============================================================================
            ZIGZAG MOTOR CONTROLLER — ONE-VARIABLE S-CURVE
                   Armed-Sensor Model + Cosine Ramp
===============================================================================

WHAT'S NEW IN THIS VERSION
-------------------------------------------------------------------------------
Tuning is now just ONE knob:

      #define RAMP_TIME_MS  500   ← total accel (or decel) time in milliseconds

No more juggling RAMP_PERCENT, SCURVE_MIN_FACTOR, shape factors, etc.
Whatever speed you type, the motor takes RAMP_TIME_MS to get there from
rest, following a smooth cosine S-curve. Decel uses the same time.

WHY IT NO LONGER VIBRATES AT START / STOP
-------------------------------------------------------------------------------
The earlier S-curve lingered near 0 Hz at the endpoints — and stepper
motors physically can't run smoothly below roughly 50–80 Hz. Pulses get
so far apart that the rotor chatters between detent positions. You hear
it as a buzz / hum / shudder at the very beginning of motion and the
very end of a stop.

The fix: skip the resonance zone entirely.

  • On START :  speed jumps from 0 → MIN_HZ instantly,
                then S-curve ramps MIN_HZ → target.
  • On STOP  :  S-curve ramps current → MIN_HZ,
                then snaps to 0.

MIN_HZ is internal (defined below). 80 Hz is a safe default for most
NEMA17 + TB6600 setups. Increase it if your motor still buzzes; decrease
it for finer low-speed control.

===============================================================================
WIRING
-------------------------------------------------------------------------------

  ESP32 GPIO14  ───►  TB6600 PUL+
  ESP32 GPIO27  ───►  TB6600 DIR+

  ESP32 GND     ───►  TB6600 PUL-
  ESP32 GND     ───►  TB6600 DIR-      ← make this connection SOLID.
                                          A loose DIR- = phantom direction
                                          flips that look like a software bug.

  Hall LEFT  OUT  ───►  ESP32 GPIO32
  Hall RIGHT OUT  ───►  ESP32 GPIO22
  Hall VCC        ───►  3.3V
  Hall GND        ───►  GND

  TB6600 ENA:  leave disconnected (driver enabled by default)

CRITICAL: ESP32 GND and TB6600 GND must share a common ground.

===============================================================================
SERIAL MONITOR
-------------------------------------------------------------------------------

  Baud:  115200

  Type a speed in Hz and press ENTER:

      100     gentle
      500     medium
      2000    fast
      4500    very fast

  Type 0 to smoothly stop.

===============================================================================
THE COSINE S-CURVE — HOW IT WORKS
-------------------------------------------------------------------------------

Each ramp interpolates from a START speed to an END speed over RAMP_TIME_MS:

    p     = elapsed / RAMP_TIME_MS                   (progress, 0.0 → 1.0)
    s     = 0.5 × (1 − cos(π × p))                   (cosine shape, 0 → 1)
    speed = startSpeed + (endSpeed − startSpeed) × s

Why cosine and not the old 4·p·(1−p)?

  • Cosine has ZERO slope at both ends — perfectly smooth handoff at
    start and stop. (4·p·(1−p) has a non-zero slope at the endpoints.)
  • Maximum acceleration sits in the middle of the ramp — easy on the
    mechanics, easy on the ear.
  • Time-based parameterization means we don't have to know or compute
    target-relative shape factors. One math expression covers every case:
    starting, stopping, mid-flight speed changes.

Profile (illustrative):

   speed
   target ┤            _____
          │          ╱
          │         ╱             ← max accel here (mid-ramp)
          │       ╱
   MIN_HZ ┤  ────╱                 ← jump-start out of resonance
        0 ┤ /
          └─────────────────────► time

===============================================================================
*/


// ============================================================================
// PIN DEFINITIONS
// ============================================================================

#define PIN_STEP    14    // STEP pulse pin — one pulse = one microstep
#define PIN_DIR     27    // DIRECTION pin — HIGH=CW, LOW=CCW

#define HALL_LEFT   32    // Left  end sensor (active LOW)
#define HALL_RIGHT  22    // Right end sensor (active LOW)


// ============================================================================
// THE ONE TUNING KNOB
// ============================================================================
//
#define ACCEL_HZ_PER_SEC   2000     // acceleration rate (steps/sec²)
                                    //   2000  → very gentle
                                    //   8000  → balanced default
                                    //   20000 → aggressive


// ============================================================================
// INTERNAL CONSTANTS (rarely need changing)
// ============================================================================

// STEP pulse width in microseconds.
// TB6600 needs at least ~2 us; 5–15 us is safe.
#define PULSE_US     10

// Settle delay after toggling DIR pin (ms).
// Lets TB6600 register the new direction before the next pulse.
#define DIR_SETTLE   15

// Minimum stepper frequency in Hz — below this, the motor enters its
// resonance/cogging zone and buzzes instead of stepping cleanly.
// We "jump" past this zone on start, and snap to 0 below it on stop.
//   60–80   →  good for most NEMA17 setups
//   100+    →  if your motor still buzzes at startup
//   30–50   →  if you need finer low-speed control and the motor is happy
#define MIN_HZ       80

// How often the ramp engine recomputes currentSpeed (ms).
// Smaller = smoother (but more CPU); larger = coarser.
#define RAMP_UPDATE_MS  5


// ============================================================================
// SENSOR ARMING — picks which end-stop is "live" right now
// ============================================================================
//
// Only the armed sensor can trip the motor. The other is ignored entirely.
//
// Moving CW  (→ RIGHT)  →  arm RIGHT sensor
// Moving CCW (→ LEFT )  →  arm LEFT  sensor
//
// After a reversal we swap the armed sensor. The just-tripped sensor is
// disarmed, so the carriage can sit on its magnet without re-triggering.
//
enum ArmedSensor { ARM_LEFT, ARM_RIGHT };


// ============================================================================
// STATE
// ============================================================================

// User-commanded target speed (Hz). Updated by Serial input.
float  targetSpeedUser = 0;

// Live speed used to time STEP pulses (Hz).
float  currentSpeed    = 0;

// Microseconds between STEP pulses. Derived from currentSpeed each ramp tick.
float  stepIntervalUs  = 999999;

// Direction state.
//   true  = CW  (DIR HIGH)
//   false = CCW (DIR LOW)
bool   direction       = true;

// True while we're ramping DOWN toward a direction flip.
// handleRamp() sets it false once the flip is complete.
bool   reversing       = false;

// Which end sensor is currently watching for an end-of-travel hit.
ArmedSensor armed      = ARM_RIGHT;     // start CW → expect RIGHT trip first

// Cosine ramp bookkeeping — set by beginRamp(), consumed by handleRamp().
float         rampFromHz     = 0;
float         rampToHz       = 0;
unsigned long rampStartTime  = 0;
unsigned long rampDurationMs = 0;     // derived from speed delta + accel
bool          ramping        = false;

// Step pulse timing.
unsigned long lastStepTime = 0;

// Ramp update timing.
unsigned long lastRampTime = 0;

// Serial input buffer.
String inputText = "";


// ============================================================================
// SETUP
// ============================================================================

void setup()
{
    Serial.begin(115200);

    // Motor pins
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR,  OUTPUT);
    digitalWrite(PIN_STEP, LOW);
    digitalWrite(PIN_DIR,  HIGH);            // boot up in CW

    // Sensors — idle HIGH via internal pull-up, LOW when magnet present
    pinMode(HALL_LEFT,  INPUT_PULLUP);
    pinMode(HALL_RIGHT, INPUT_PULLUP);

    Serial.println();
    Serial.println("===========================================");
    Serial.println(" Zigzag Controller — One-Variable S-Curve");
    Serial.print  (" Ramp time: ");
    Serial.print  (" Accel    : ");
    Serial.print  (ACCEL_HZ_PER_SEC);
    Serial.println(" Hz/s");
    Serial.println(MIN_HZ);
    Serial.println(" Enter speed in Hz   (0 = stop)");
    Serial.println("===========================================");
    Serial.println();
}


// ============================================================================
// MAIN LOOP
// ============================================================================
//
// Four cooperative non-blocking tasks. No delay() in the hot path.
//
void loop()
{
    readSerial();      // accept new speed commands
    handleRamp();      // advance currentSpeed along the cosine curve
    stepPulse();       // emit STEP pulses at the right rate
    checkSensors();    // watch the armed end-stop
}


// ============================================================================
// START A NEW RAMP toward a target Hz
// ============================================================================
//
// Called whenever we want the motor's speed to change:
//
//   • user typed a new speed         →  beginRamp(targetSpeedUser)
//   • user typed 0 (stop)            →  beginRamp(0)
//   • sensor tripped (start reversal)→  beginRamp(0)        with reversing=true
//   • reversal completed             →  beginRamp(targetSpeedUser)
//
// Handles three special situations:
//   1) Cold start from 0 → jump-start past resonance (0 → MIN_HZ instantly)
//   2) Target below MIN_HZ but not zero → clamp up to MIN_HZ
//   3) From == To → nothing to do
//
void beginRamp(float toHz)
{
    // Clamp non-zero targets above resonance floor
    if (toHz > 0 && toHz < MIN_HZ) toHz = MIN_HZ;

    // Jump-start out of the resonance zone if we're stationary
    if (currentSpeed < 1 && toHz > 0)
    {
        currentSpeed = MIN_HZ;
        rampFromHz   = MIN_HZ;
    }
    else
    {
        rampFromHz = currentSpeed;
    }

    rampToHz       = toHz;
    rampStartTime  = millis();

    // time = Δspeed / acceleration
    float delta    = fabsf(rampToHz - rampFromHz);
    rampDurationMs = (unsigned long)(delta * 1000.0f / ACCEL_HZ_PER_SEC);
    if (rampDurationMs < 1) rampDurationMs = 1;

    ramping = (delta > 0.5f);
}


// ============================================================================
// READ SERIAL INPUT
// ============================================================================
//
// Build a line one character at a time. ENTER commits it.
// Negative values are treated as 0 (stop).
//
void readSerial()
{
    while (Serial.available())
    {
        char c = Serial.read();

        if (c == '\n' || c == '\r')
        {
            inputText.trim();

            if (inputText.length() > 0)
            {
                float v = inputText.toFloat();
                targetSpeedUser = (v < 0) ? 0 : v;

                // If we're NOT in the middle of a reversal, ramp to the
                // new target right away. If we ARE reversing, the motor
                // is busy stopping; the new target will be picked up
                // automatically when the reversal completes.
                if (!reversing)
                {
                    beginRamp(targetSpeedUser);
                }

                Serial.print("Target: ");
                Serial.print (targetSpeedUser);
                Serial.println(" Hz");
            }

            inputText = "";
        }
        else
        {
            inputText += c;
        }
    }
}


// ============================================================================
// RAMP ENGINE — cosine S-curve interpolation
// ============================================================================
//
// Each tick:
//   • Compute progress p ∈ [0, 1] across RAMP_TIME_MS
//   • currentSpeed = from + (to − from) × 0.5 × (1 − cos(π × p))
//   • When p reaches 1, ramp is finished
//   • If decelerating and we slip below MIN_HZ on the way to 0, snap to 0
//     (avoids the buzz zone)
//   • If we just reached 0 while reversing → flip direction + swap armed
//     sensor + kick off the climb-back-up ramp
//
void handleRamp()
{
    if (millis() - lastRampTime < RAMP_UPDATE_MS) return;
    lastRampTime = millis();

    if (ramping)
    {
        unsigned long elapsed = millis() - rampStartTime;

       if (elapsed >= rampDurationMs)
        {
            currentSpeed = rampToHz;
            ramping = false;
        }
        else
        {
            float p     = (float)elapsed / (float)rampDurationMs;
            float shape = 0.5f * (1.0f - cosf(PI * p));
            float v     = rampFromHz + (rampToHz - rampFromHz) * shape;

            // Decelerating to 0 and we've dropped into the buzz zone?
            // Snap straight to 0.
            if (rampToHz <= 0 && v < MIN_HZ)
            {
                currentSpeed = 0;
                ramping = false;
            }
            else
            {
                currentSpeed = v;
            }
        }
    }

    // Reversal completion — speed hit 0 while reversing flag is up
    if (reversing && currentSpeed <= 0 && !ramping)
    {
        direction = !direction;
        digitalWrite(PIN_DIR, direction ? HIGH : LOW);
        delay(DIR_SETTLE);

        armed     = direction ? ARM_RIGHT : ARM_LEFT;
        reversing = false;

        Serial.println(direction ? "-> CW" : "<- CCW");

        // Ramp back up to the user's target speed
        if (targetSpeedUser > 0) beginRamp(targetSpeedUser);
    }

    // Refresh STEP pulse interval from currentSpeed
    stepIntervalUs = (currentSpeed < 1) ? 999999 : (1000000.0f / currentSpeed);
}


// ============================================================================
// STEP PULSE GENERATOR (non-blocking)
// ============================================================================
//
//   1000 Hz  →  1000 us between pulses
//   2000 Hz  →   500 us between pulses
//   5000 Hz  →   200 us between pulses
//
void stepPulse()
{
    if (currentSpeed <= 0) return;

    unsigned long now = micros();
    if (now - lastStepTime >= stepIntervalUs)
    {
        lastStepTime = now;
        digitalWrite(PIN_STEP, HIGH);
        delayMicroseconds(PULSE_US);
        digitalWrite(PIN_STEP, LOW);
    }
}


// ============================================================================
// SENSOR CHECK — armed-sensor model
// ============================================================================
//
//   • Only the armed sensor is checked.
//   • When it trips, we begin a ramp-to-zero (decel) and flag reversing.
//   • handleRamp() finishes the reversal (flip DIR, swap armed sensor,
//     ramp back up to user target).
//   • While reversing == true, this function is muted so the just-tripped
//     magnet can't double-trigger us.
//
void checkSensors()
{
    if (reversing) return;
    if (currentSpeed <= 0) return;       // nothing to trip on if we're stopped

    if (armed == ARM_LEFT && digitalRead(HALL_LEFT) == LOW)
    {
        reversing = true;
        beginRamp(0);
        Serial.println("LEFT trip");
    }
    else if (armed == ARM_RIGHT && digitalRead(HALL_RIGHT) == LOW)
    {
        reversing = true;
        beginRamp(0);
        Serial.println("RIGHT trip");
    }
}
