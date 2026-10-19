#include <AS5600.h>
#include <Wire.h>

// =============================================================================
// CONFIGURATION (matched to RLControl.ino)
// =============================================================================
const long SERIAL_BAUD_RATE = 500000;  // High speed serial
const long I2C_CLOCK_HZ = 400000;      // Fast mode I2C (vs 100kHz default)

// Motor pins (to disable motor and prevent heating)
const int DIR_PIN = 2;
const int STEP_PIN = 9;     // Timer1 OC1A (rig wired here so FastAccelStepper-based sketches work too)
const int ENABLE_PIN = 5;

// =============================================================================
// STATE
// =============================================================================
AS5600 as5600;

// Plotting variables
int counter_plot = 0;
const int FREQUENCY_PLOT = 20;

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);  // LED on during setup

    // Disable motor driver to prevent heating
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);  // HIGH = disabled (active-low enable)
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);

    Serial.begin(SERIAL_BAUD_RATE);
    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);
    as5600.begin();

    while (!as5600.detectMagnet())
    {
        Serial.println("[AS5600] Waiting for magnet...");
        delay(1000);
    }

    // Print the current magnitude of the magnet
    Serial.print("[AS5600] Current magnitude: ");
    Serial.println(as5600.readMagnitude());

    // Print the magnet strength
    if (as5600.magnetTooWeak())
    {
        Serial.println("[AS5600] Magnet strength is too weak. ---");
    }
    else if (as5600.magnetTooStrong())
    {
        Serial.println("[AS5600] Magnet strength is too strong. +++");
    }
    else
    {
        Serial.println("[AS5600] Magnet strength is just right!");
    }

    digitalWrite(LED_BUILTIN, LOW);  // LED off when ready
    Serial.println("[AS5600] Encoder test ready - rotate the pendulum");
    Serial.println("[Motor] Driver disabled (not heating)");
}

void loop()
{
    // Increment counter
    counter_plot++;

    // Get the pendulum position
    float pendulum_actual_deg = convertRawAngleToDegrees();

    // Print to the serial at reduced frequency
    if (counter_plot % FREQUENCY_PLOT == 0)
    {
        // Flash LED briefly to indicate activity
        digitalWrite(LED_BUILTIN, HIGH);

        // Format for Arduino Serial Plotter: "label:value"
        Serial.print("pendulum_deg:");
        Serial.println(pendulum_actual_deg);

        digitalWrite(LED_BUILTIN, LOW);
    }
}

/*
 * Convert the raw angle from the AS5600 magnetic encoder to degrees.
 * Handles multi-revolution tracking with wraparound detection.
 */
float convertRawAngleToDegrees()
{
    // AS5600 constants
    const long AS5600_RESOLUTION = 4096;
    const long WRAPAROUND_THRESHOLD = AS5600_RESOLUTION / 2;  // 2048
    const float DEG_PER_SEGMENT = 360.0f / AS5600_RESOLUTION; // 0.087890625

    static long raw_prev = 0;
    static bool first_reading = true;
    static float position = 0.0f;

    // Get the current position of the AS5600
    long raw = as5600.rawAngle();

    if (first_reading)
    {
        raw_prev = raw;
        first_reading = false;
    }

    long delta = raw - raw_prev;

    // Handle wraparound (movement > 180 deg between samples indicates wraparound)
    if (delta > WRAPAROUND_THRESHOLD)  delta -= AS5600_RESOLUTION;
    if (delta < -WRAPAROUND_THRESHOLD) delta += AS5600_RESOLUTION;

    position += (float)delta * DEG_PER_SEGMENT;

    // Save the current raw angle for the next iteration
    raw_prev = raw;

    return position;
}
