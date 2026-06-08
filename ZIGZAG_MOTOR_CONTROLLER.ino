/*
===============================================================================
            ZIGZAG MOTOR CONTROLLER — FINAL VERSION
                    Armed-Sensor Model
                    Acceleration-Based Cosine S-Curve
                    Proportional Reversal with Resonance Floor
===============================================================================

WHAT THIS CODE DOES
-------------------------------------------------------------------------------
Drives a stepper motor back-and-forth between two end sensors.

  • ESP32
  • TB6600 stepper driver
  • NEMA stepper motor
  • Two Hall-effect end sensors (LEFT and RIGHT)

Motion cycle:

      [LEFT sensor]  <---- carriage ---->  [RIGHT sensor]

  → carriage cruises toward an end at user speed
  → sensor at that end trips
  → motor smoothly DECELERATES to a "reversal speed" (NOT zero)
  → DIR pin flips WHILE STILL PULSING
  → motor smoothly ACCELERATES back up to cruise speed
  → carriage moves to the other end
  → repeat forever

User types a speed in Hz on Serial Monitor.
Type 0 to smoothly stop.


===============================================================================
THREE KEY DESIGN DECISIONS
-------------------------------------------------------------------------------

1) ARMED-SENSOR MODEL
   ---------------------------------------------------------------------------
   At any moment, exactly ONE sensor is "armed" — the one at the FAR end.

       Moving CW  (→ RIGHT)  →  RIGHT sensor armed
       Moving CCW (→ LEFT )  →  LEFT  sensor armed

   The instant a sensor trips, we flag a reversal AND swap the armed sensor.
   The just-tripped sensor is now disarmed — the carriage can sit on its
   magnet without re-triggering. No debouncing, no spike rejection,
   no timed lockouts. Physics does the work.


2) ACCELERATION-BASED COSINE S-CURVE
   ---------------------------------------------------------------------------
   Tuning knob: ACCEL_HZ_PER_SEC (units: Hz per second = steps/s²)

   Ramp duration is DERIVED from the speed delta:

       duration = Δspeed / acceleration

   This means acceleration FORCE on the mechanics is constant regardless
   of target speed — the right thing for belts, leadscrews, bearings.

       500 Hz target  →  500 / 8000  = 62 ms ramp
       4000 Hz target →  4000 / 8000 = 500 ms ramp

   Within each ramp, a cosine curve shapes the speed-vs-time profile:

       p     = elapsed / duration               (0.0 → 1.0)
       shape = 0.5 × (1 − cos(π × p))           (0 → 1, zero slope at ends)
       speed = from + (to − from) × shape

   Cosine has ZERO slope at both endpoints — perfectly smooth handoff
   in and out of every ramp (no jerk = no missed steps).


3) PROPORTIONAL REVERSAL WITH RESONANCE FLOOR
   ---------------------------------------------------------------------------
   The S-curve's job at reversal is NOT to stop the motor.
   It's to CUSHION the direction change so the motor doesn't jerk.

   Stopping at 0 Hz drags the motor through its resonance band
   (~0–300 Hz on NEMA17) where it vibrates and slips. So we don't go there.

   Instead, reversal speed scales with cruise speed:

       reversalHz = cruise × (100 − REVERSAL_PERCENT) / 100

       cruise 2000 →  20% bleed  →  flip DIR at 1600 Hz
       cruise 1000 →  20% bleed  →  flip DIR at  800 Hz

   But at very low cruise speeds, 20% bleed would put us inside the
   resonance band. So we clamp:

       if (reversalHz < REVERSAL_FLOOR_HZ) reversalHz = REVERSAL_FLOOR_HZ

   The motor flips DIR while still pulsing above resonance, then ramps
   back up to cruise. No vibration, no slip, no audible jerk.


===============================================================================
WIRING
-------------------------------------------------------------------------------

  ESP32 GPIO14  ───►  TB6600 PUL+
  ESP32 GPIO27  ───►  TB6600 DIR+

  ESP32 GND     ───►  TB6600 PUL-
  ESP32 GND     ───►  TB6600 DIR-      ← make this connection SOLID.
                                          A loose DIR- causes phantom
                                          direction flips that look like
                                          software bugs (lessons learned).

  Hall LEFT  OUT  ───►  ESP32 GPIO32
  Hall RIGHT OUT  ───►  ESP32 GPIO22
  Hall VCC        ───►  3.3V
  Hall GND        ───►  GND

  TB6600 ENA:  leave disconnected (driver enabled by default)

CRITICAL: ESP32 GND and TB6600 GND must share a common ground.


===============================================================================
SERIAL MONITOR
-------------------------------------------------------------------------------

  Baud: 115200

  Type a speed in Hz and press ENTER:

      400     gentle
      1000    medium
      2500    fast
      4500    very fast

  Type 0 to smoothly stop.

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
// TUNING KNOBS
// ============================================================================

// ---- ACCELERATION ----------------------------------------------------------
// Constant acceleration rate in Hz per second (i.e. steps/s²).
// Defines how aggressively the motor changes speed.
//
//   2000   →  very gentle      (heavy carriages, fragile mechanics)
//   8000   →  balanced default
//   20000  →  aggressive       (light loads, snappy response)
//
#define ACCEL_HZ_PER_SEC   4000

// ---- REVERSAL DEPTH --------------------------------------------------------
// How much of cruise speed to bleed off before flipping DIR.
//
//   10  →  small cushion, fast reversal but harder on motor
//   20  →  balanced default
//   40  →  large cushion, very smooth but slow reversal
//
#define REVERSAL_PERCENT   20

// ---- RESONANCE FLOOR -------------------------------------------------------
// Minimum speed at which we'll ever flip DIR or start/stop the motor.
// Stays above the motor's resonance/cogging band.
//
//   300   →  good for most NEMA17 (8-step microstepping)
//   500   →  safer if the motor still vibrates at startup
//   700+  →  for stubborn high-inertia setups
//
#define REVERSAL_FLOOR_HZ  600


// ============================================================================
// INTERNAL CONSTANTS (rarely need changing)
// ============================================================================

// STEP pulse width in microseconds. TB6600 needs ~2 us minimum; 5–15 is safe.
#define PULSE_US        10

// Settle delay after toggling DIR pin (microseconds).
// Lets the TB6600 register the new direction before the next STEP.
// Short because we flip DIR while spinning — long delay() would stall pulses.
#define DIR_SETTLE_US   500

// How often the ramp engine recomputes currentSpeed (ms).
#define RAMP_UPDATE_MS  5


// ============================================================================
// SENSOR ARMING
// ============================================================================
//
// Identifies which end-stop is currently "live". The other is ignored.
//
enum ArmedSensor { ARM_LEFT, ARM_RIGHT };


// ============================================================================
// STATE
// ============================================================================

// User-commanded target cruise speed (Hz). Updated by Serial input.
float  targetSpeedUser = 0;

// Live speed used to time STEP pulses (Hz).
float  currentSpeed    = 0;

// Microseconds between STEP pulses. Derived from currentSpeed each ramp tick.
float  stepIntervalUs  = 999999;

// Direction state.
//   true  = CW  (DIR HIGH)
//   false = CCW (DIR LOW)
bool   direction       = true;

// True while ramping DOWN toward a sensor-triggered direction reversal.
// Cleared once the reversal completes.
bool   reversing       = false;

// Which end sensor is currently watching for an end-of-travel hit.
ArmedSensor armed      = ARM_RIGHT;     // boot in CW → expect RIGHT trip first

// Cosine ramp bookkeeping — set by beginRamp(), consumed by handleRamp().
float         rampFromHz     = 0;       // speed at start of current ramp
float         rampToHz       = 0;       // target speed of current ramp
unsigned long rampStartTime  = 0;       // millis() when this ramp began
unsigned long rampDurationMs = 0;       // computed from speed delta / accel
bool          ramping        = false;   // true while a ramp is in progress

// STEP pulse timing.
unsigned long lastStepTime = 0;

// Ramp engine update timing.
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
    digitalWrite(PIN_DIR,  HIGH);          // boot in CW

    // Sensors — idle HIGH via internal pull-up, LOW when magnet detected
    pinMode(HALL_LEFT,  INPUT_PULLUP);
    pinMode(HALL_RIGHT, INPUT_PULLUP);

    Serial.println();
    Serial.println("===========================================");
    Serial.println(" Zigzag Controller — Final");
    Serial.print  (" Accel        : ");
    Serial.print  (ACCEL_HZ_PER_SEC);
    Serial.println(" Hz/s");
    Serial.print  (" Reversal     : ");
    Serial.print  (REVERSAL_PERCENT);
    Serial.println("% bleed");
    Serial.print  (" Floor        : ");
    Serial.print  (REVERSAL_FLOOR_HZ);
    Serial.println(" Hz");
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
// COMPUTE REVERSAL SPEED FROM CRUISE SPEED
// ============================================================================
//
// Implements the proportional-with-floor rule:
//
//   reversal = max(cruise × (100 − REVERSAL_PERCENT)% , REVERSAL_FLOOR_HZ)
//
// Examples (REVERSAL_PERCENT=20, FLOOR=400):
//
//   cruise 3000  →  2400 Hz  (proportional)
//   cruise 1000  →   800 Hz  (proportional)
//   cruise  500  →   400 Hz  (clamped to floor)
//   cruise  300  →   300 Hz  (already below floor — no bleed at all)
//
float computeReversalHz(float cruise)
{
    float rev = cruise * (100.0f - REVERSAL_PERCENT) / 100.0f;

    if (rev < REVERSAL_FLOOR_HZ) rev = REVERSAL_FLOOR_HZ;
    if (rev > cruise)            rev = cruise;     // edge case: cruise ≤ floor

    return rev;
}


// ============================================================================
// START A NEW RAMP toward a target Hz
// ============================================================================
//
// Called whenever we want the motor's speed to change:
//
//   • user typed a new speed         → beginRamp(targetSpeedUser)
//   • user typed 0 (stop)            → beginRamp(0)
//   • sensor tripped                 → beginRamp(reversalHz)  with reversing=true
//   • reversal completed             → beginRamp(targetSpeedUser)
//
// Cold-start handling:
//   If we're stationary (currentSpeed≈0) and asked to go to a non-zero
//   target, we instantly jump to REVERSAL_FLOOR_HZ first. This skips the
//   resonance band the same way a reversal does — the motor never tries
//   to run below the floor (except during a final stop, handled below).
//
void beginRamp(float toHz)
{
    // Clamp non-zero targets to the resonance floor
    if (toHz > 0 && toHz < REVERSAL_FLOOR_HZ) toHz = REVERSAL_FLOOR_HZ;

    // Jump-start out of the resonance zone if currently stationary
    if (currentSpeed < 1 && toHz > 0)
    {
        currentSpeed = REVERSAL_FLOOR_HZ;
        rampFromHz   = REVERSAL_FLOOR_HZ;
    }
    else
    {
        rampFromHz = currentSpeed;
    }

    rampToHz      = toHz;
    rampStartTime = millis();

    // duration = |Δspeed| / acceleration
    float delta    = fabsf(rampToHz - rampFromHz);
    rampDurationMs = (unsigned long)(delta * 1000.0f / ACCEL_HZ_PER_SEC);
    if (rampDurationMs < 1) rampDurationMs = 1;

    ramping = (delta > 0.5f);
}


// ============================================================================
// SERIAL INPUT
// ============================================================================
//
// Build a line one character at a time. ENTER commits it.
// Negative values → 0 (stop).
//
// If a reversal is in progress, the new target is stored but not acted on
// immediately — the reversal will pick it up automatically when complete.
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
// Each tick (every RAMP_UPDATE_MS ms):
//   • If a ramp is active, advance currentSpeed along the cosine curve
//   • Snap to 0 when decelerating to a stop and we cross the floor
//   • When a reversal-decel ramp finishes, flip DIR and start the accel ramp
//   • Refresh stepIntervalUs from the new currentSpeed
//
void handleRamp()
{
    if (millis() - lastRampTime < RAMP_UPDATE_MS) return;
    lastRampTime = millis();

    // ---- ACTIVE RAMP ------------------------------------------------------
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

            // User-initiated stop: snap to 0 once we drop below the floor.
            // Avoids dragging the motor through the resonance band at stop.
            if (rampToHz <= 0 && v < REVERSAL_FLOOR_HZ)
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

    // ---- REVERSAL HANDOFF -------------------------------------------------
    // Decel ramp finished — currentSpeed is now at the reversal speed,
    // safely above resonance. Flip DIR while still pulsing, swap the
    // armed sensor, then kick off the accel ramp back up to cruise.
    if (reversing && !ramping)
    {
        direction = !direction;
        digitalWrite(PIN_DIR, direction ? HIGH : LOW);
        delayMicroseconds(DIR_SETTLE_US);   // brief settle; pulses keep flowing

        armed     = direction ? ARM_RIGHT : ARM_LEFT;
        reversing = false;

        Serial.println(direction ? "-> CW" : "<- CCW");

        if (targetSpeedUser > 0)
        {
            beginRamp(targetSpeedUser);
        }
    }

    // ---- STEP INTERVAL ----------------------------------------------------
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
//   • On trip, begin a decel ramp toward the proportional reversal speed
//     (not zero) and raise the reversing flag.
//   • handleRamp() finishes the reversal (flip DIR, swap armed sensor,
//     ramp back up).
//   • While reversing == true, this function is muted — the just-tripped
//     magnet can't double-trigger us.
//
void checkSensors()
{
    if (reversing) return;
    if (currentSpeed <= 0) return;       // nothing to trip on if stopped

    if (armed == ARM_LEFT && digitalRead(HALL_LEFT) == LOW)
    {
        reversing = true;
        beginRamp(computeReversalHz(currentSpeed));
        Serial.print("LEFT trip → reversal at ");
        Serial.print (rampToHz);
        Serial.println(" Hz");
    }
    else if (armed == ARM_RIGHT && digitalRead(HALL_RIGHT) == LOW)
    {
        reversing = true;
        beginRamp(computeReversalHz(currentSpeed));
        Serial.print("RIGHT trip → reversal at ");
        Serial.print (rampToHz);
        Serial.println(" Hz");
    }
}
