#include <AccelStepper.h>

// Define the stepper motor connections
const int DIR_PIN = 2;  // Direction
const int STEP_PIN = 9; // Step (Timer1 OC1A — rig wired here so FastAccelStepper-based sketches work too)

// Create an instance of the AccelStepper class
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN, 0, 0, false);

const int MICROSTEPS = 32;  // 32 = DRV8825 M0=M1=M2=HIGH, TMC2209 MS1=H/MS2=L
                            // (different levels per driver); 16 = TMC2209 MS1=MS2=HIGH
const long STEPS_PER_REVOLUTION = 200 * MICROSTEPS;

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);  // LED on during setup

    Serial.begin(115200); // Start the serial communication

    // AccelStepper on Arduino Nano is limited to ~4000 steps/sec with run()
    // At 6400 steps/rev (32x microstepping), 2000 steps/sec = 18.75 RPM
    stepper.setMaxSpeed(2000);
    stepper.setAcceleration(4000);  // Reaches max speed in 0.5 sec

    // Set the enable pin for the stepper motor driver and
    // invert it because we are using a DRV8825 board with an
    // active-low enable signal (LOW = enabled, HIGH = disabled)
    stepper.setEnablePin(5);
    stepper.setPinsInverted(false, false, true);

    // Set the initial position
    stepper.setCurrentPosition(0);

    // Enable the motor outputs
    stepper.enableOutputs();

    digitalWrite(LED_BUILTIN, LOW);  // LED off when ready

    Serial.println("Motor Test - Moving 1/4 revolution back and forth");
}

void loop()
{
    // Move the motor to +1/4 revolution (absolute position 400)
    Serial.println("Moving to +1/4 revolution");
    digitalWrite(LED_BUILTIN, HIGH);
    stepper.runToNewPosition(STEPS_PER_REVOLUTION / 4);
    digitalWrite(LED_BUILTIN, LOW);

    // Wait for 1 second
    delay(1000);

    // Move the motor to -1/4 revolution (absolute position -400)
    // This is a 1/2 revolution movement from the current position
    Serial.println("Moving to -1/4 revolution");
    digitalWrite(LED_BUILTIN, HIGH);
    stepper.runToNewPosition(-STEPS_PER_REVOLUTION / 4);
    digitalWrite(LED_BUILTIN, LOW);

    // Wait for 1 second
    delay(1000);
}
