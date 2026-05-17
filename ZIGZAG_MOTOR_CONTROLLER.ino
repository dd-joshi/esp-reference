/*
===============================================================================
                ZIGZAG MOTOR CONTROLLER — PROPORTIONAL RAMP
===============================================================================

AUTHOR PURPOSE
-------------------------------------------------------------------------------
This program controls a stepper motor using:

  • ESP32
  • TB6600 stepper driver
  • NEMA stepper motor
  • Two Hall sensors for end detection

The motor continuously moves LEFT and RIGHT.

When the carriage reaches an end sensor:
  → motor smoothly decelerates
  → motor fully stops
  → direction reverses
  → motor smoothly accelerates again

This creates smooth zigzag motion suitable for:
  • lead screw systems
  • cutting machines
  • slider mechanisms
  • automation demos
  • educational motion-control projects

===============================================================================
WHY PROPORTIONAL RAMPING?
-------------------------------------------------------------------------------

Instead of fixed acceleration values, this code uses:

    rampStep = motorSpeed × percentage

Meaning:

  Low speed  → small acceleration steps
  High speed → larger acceleration steps

Result:
  Motion "feels" similarly smooth at different speeds.

Example:
-------------------------------------------------------------------------------

If target speed = 800 Hz
and RAMP_PERCENT = 5%

Then:

    rampStep = 800 × 5 / 100
             = 40 Hz per ramp update

If target speed = 4000 Hz:

    rampStep = 4000 × 5 / 100
             = 200 Hz per update

So high-speed motion ramps faster automatically.

===============================================================================
WIRING
===============================================================================

ESP32 GPIO14  ---> TB6600 PUL+
ESP32 GPIO27  ---> TB6600 DIR+

ESP32 GND     ---> TB6600 PUL-
ESP32 GND     ---> TB6600 DIR-

Hall LEFT OUT  ---> GPIO32
Hall RIGHT OUT ---> GPIO22

Hall sensor VCC ---> 3.3V
Hall sensor GND ---> GND

TB6600 ENA:
  Leave disconnected

IMPORTANT:
  ESP32 GND and TB6600 GND MUST be connected.

===============================================================================
SERIAL MONITOR
===============================================================================

Baud Rate = 115200

Type a speed value in Hz:

  50      slow
  200     medium
  1000    fast
  4500    very fast

Type:
  0

to smoothly stop the motor.

===============================================================================
*/


// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// STEP pulse pin
// Every pulse = one microstep/step movement
#define PIN_STEP    14

// DIRECTION pin
// HIGH = RIGHT
// LOW  = LEFT
#define PIN_DIR     27

// Hall sensor pins
#define HALL_LEFT   32
#define HALL_RIGHT  22


// ============================================================================
// MOTOR CONFIGURATION
// ============================================================================

// STEP pulse width in microseconds.
//
// TB6600 requires pulse width long enough
// to detect reliably.
//
// Typical safe values:
//   5–15 us
//
#define PULSE_US    10


// ============================================================================
// RAMP CONFIGURATION
// ============================================================================

// Percentage of target speed added/removed
// during each ramp update.
//
// Higher value:
//   faster acceleration
//   harder direction changes
//
// Lower value:
//   smoother motion
//   slower acceleration
//
#define RAMP_PERCENT  5


// Time between ramp updates.
//
// Smaller value:
//   more responsive acceleration
//
// Larger value:
//   gentler acceleration
//
#define RAMP_MS       5


// Small delay after changing direction.
//
// Gives TB6600 time to register
// new DIR state before next pulse.
//
#define DIR_SETTLE    5


// ============================================================================
// MOTOR STATES
// ============================================================================

// RUNNING
//   Motor running at target speed
//
// DECELERATING
//   Motor slowing down
//
// ACCELERATING
//   Motor speeding up
//
enum MotorState
{
    RUNNING,
    DECELERATING,
    ACCELERATING
};


// Current motor state
MotorState motorState = RUNNING;


// ============================================================================
// VARIABLES
// ============================================================================

// Target speed entered by user
float motorSpeed = 0;


// Actual live speed during ramping
float currentSpeed = 0;


// How much speed changes each ramp tick
float rampStep = 0;


// Time between step pulses in microseconds
//
// Formula:
//
//   interval = 1,000,000 / speed
//
float stepIntervalUs = 999999;


// Direction flag
//
// true  = RIGHT
// false = LEFT
//
bool direction = true;


// Timestamp of last step pulse
unsigned long lastStepTime = 0;


// Timestamp of last ramp update
unsigned long lastRampTime = 0;


// Serial text buffer
String inputText = "";


// ============================================================================
// SETUP
// ============================================================================

void setup()
{
    // Start serial communication
    Serial.begin(115200);


    // Configure motor pins
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR,  OUTPUT);

    digitalWrite(PIN_STEP, LOW);


    // Configure hall sensor pins
    //
    // INPUT_PULLUP means:
    //   normal state = HIGH
    //   triggered     = LOW
    //
    pinMode(HALL_LEFT,  INPUT_PULLUP);
    pinMode(HALL_RIGHT, INPUT_PULLUP);


    // Start direction = RIGHT
    //
    // Motor will NOT move until user enters speed.
    //
    digitalWrite(PIN_DIR, HIGH);


    Serial.println();
    Serial.println("========================================");
    Serial.println(" Zigzag Controller Ready");
    Serial.println(" Enter speed in Hz");
    Serial.println(" Type 0 to stop");
    Serial.println("========================================");
    Serial.println();
}


// ============================================================================
// MAIN LOOP
// ============================================================================

void loop()
{
    // Read new serial commands
    readSerialInput();

    // Handle acceleration/deceleration
    handleRamp();

    // Generate motor pulses
    moveMotor();

    // Check end sensors
    checkHallSensors();
}


// ============================================================================
// CALCULATE RAMP STEP
// ============================================================================
//
// Called whenever target speed changes.
//
// Example:
//
// motorSpeed = 2000
// RAMP_PERCENT = 5
//
// rampStep = 2000 × 5 / 100
//          = 100 Hz per update
//
void updateRampStep()
{
    rampStep = motorSpeed * RAMP_PERCENT / 100.0;

    // Prevent extremely tiny ramp values
    if (rampStep < 1)
    {
        rampStep = 1;
    }
}


// ============================================================================
// READ SERIAL INPUT
// ============================================================================
//
// Reads serial characters one-by-one.
//
// User types:
//
//   2000 + ENTER
//
// Buffer becomes:
//   "2000"
//
// Then converted into float.
//
void readSerialInput()
{
    while (Serial.available())
    {
        char c = Serial.read();

        // ENTER pressed
        if (c == '\n' || c == '\r')
        {
            inputText.trim();

            if (inputText.length() > 0)
            {
                float value = inputText.toFloat();


                // STOP COMMAND
                if (value <= 0)
                {
                    motorSpeed = 0;

                    // Smooth stop
                    motorState = DECELERATING;

                    Serial.println("STOPPING");
                }


                // NEW SPEED COMMAND
                else
                {
                    motorSpeed = value;

                    // Recalculate ramp step
                    updateRampStep();

                    Serial.print("Speed: ");
                    Serial.print(motorSpeed);
                    Serial.println(" Hz");


                    // Decide whether to
                    // accelerate or decelerate
                    if (currentSpeed < motorSpeed)
                    {
                        motorState = ACCELERATING;
                    }
                    else if (currentSpeed > motorSpeed)
                    {
                        motorState = DECELERATING;
                    }
                }

                inputText = "";
            }
        }

        // Collect typed characters
        else
        {
            inputText += c;
        }
    }
}


// ============================================================================
// HANDLE ACCELERATION / DECELERATION
// ============================================================================
//
// Runs every RAMP_MS milliseconds.
//
// Adjusts currentSpeed gradually.
//
void handleRamp()
{
    // Wait until next ramp tick
    if (millis() - lastRampTime < RAMP_MS)
    {
        return;
    }

    lastRampTime = millis();


    // ========================================================================
    // ACCELERATING
    // ========================================================================

    if (motorState == ACCELERATING)
    {
        currentSpeed += rampStep;

        // Reached target?
        if (currentSpeed >= motorSpeed)
        {
            currentSpeed = motorSpeed;
            motorState = RUNNING;
        }

        updateStepInterval();
    }


    // ========================================================================
    // DECELERATING
    // ========================================================================

    else if (motorState == DECELERATING)
    {
        currentSpeed -= rampStep;

        // Fully stopped?
        if (currentSpeed <= 0)
        {
            currentSpeed = 0;

            updateStepInterval();


            // User requested STOP
            if (motorSpeed <= 0)
            {
                motorState = RUNNING;

                Serial.println("STOPPED");

                return;
            }


            // Otherwise:
            // Hall sensor triggered reversal
            reverseMotor();

            motorState = ACCELERATING;
        }
        else
        {
            updateStepInterval();
        }
    }
}


// ============================================================================
// CONVERT SPEED → STEP TIMING
// ============================================================================
//
// Example:
//
// 1000 Hz
//
// means:
//
// 1000 steps per second
//
// therefore:
//
// 1,000,000 / 1000
// = 1000 us between steps
//
void updateStepInterval()
{
    // Prevent divide-by-zero
    if (currentSpeed < 1)
    {
        stepIntervalUs = 999999;
        return;
    }

    stepIntervalUs = 1000000.0 / currentSpeed;
}


// ============================================================================
// GENERATE STEP PULSES
// ============================================================================
//
// Uses micros() timing.
//
// NON-BLOCKING:
//   no delay() used for step timing
//
// This allows:
//   smooth multitasking
//   sensor checking
//   ramp handling
//
void moveMotor()
{
    // stopped?
    if (currentSpeed <= 0)
    {
        return;
    }

    unsigned long now = micros();

    // Time for next pulse?
    if (now - lastStepTime >= stepIntervalUs)
    {
        lastStepTime = now;


        // STEP HIGH
        digitalWrite(PIN_STEP, HIGH);

        delayMicroseconds(PULSE_US);

        // STEP LOW
        digitalWrite(PIN_STEP, LOW);
    }
}


// ============================================================================
// CHECK HALL SENSORS
// ============================================================================
//
// Only checks sensor in movement direction.
//
// RIGHT movement:
//   only RIGHT sensor matters
//
// LEFT movement:
//   only LEFT sensor matters
//
void checkHallSensors()
{
    // Ignore sensors during ramping
    if (motorState != RUNNING)
    {
        return;
    }


    // Moving RIGHT
    if (direction == true)
    {
        if (digitalRead(HALL_RIGHT) == LOW)
        {
            Serial.println("Right end detected");

            motorState = DECELERATING;
        }
    }


    // Moving LEFT
    else
    {
        if (digitalRead(HALL_LEFT) == LOW)
        {
            Serial.println("Left end detected");

            motorState = DECELERATING;
        }
    }
}


// ============================================================================
// REVERSE DIRECTION
// ============================================================================
//
// Called ONLY after motor fully stops.
//
// Prevents harsh instant reversal.
//
void reverseMotor()
{
    // Flip direction
    direction = !direction;

    digitalWrite(PIN_DIR,
                 direction ? HIGH : LOW);


    // Allow driver to settle
    delay(DIR_SETTLE);


    Serial.println(direction ? "-> RIGHT"
                             : "<- LEFT");
}
