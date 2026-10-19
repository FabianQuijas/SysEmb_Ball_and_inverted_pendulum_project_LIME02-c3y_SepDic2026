#include <math.h>

const long BAUD_RATE = 1000000; // Baud rate

float time_counter = 0.0;        // Time variable for wave tracking
const float frequency = 1.0;     // Frequency of the sine/cosine waves
unsigned long previous_time = 0; // Previous time in microseconds

void setup()
{
    Serial.begin(BAUD_RATE);  // Initialize serial communication
    while (!Serial) { ; }     // Wait for the serial port to connect
    previous_time = micros(); // Initialize the previous time
}

void loop()
{
    unsigned long current_time = micros();                     // Get current time in microseconds
    float elapsed_time = (current_time - previous_time) / 1e6; // Calculate elapsed time in seconds
    previous_time = current_time;                              // Update the previous time

    time_counter += elapsed_time; // Increment time_counter by the actual elapsed time

    if (Serial.available() > 0)
    {
        char request = Serial.read(); // Read the incoming request character

        // Respond based on the request
        if (request == 'R')
        {
            // Respond with 'R' to indicate the Arduino is ready
            Serial.write('R');
        }
        else if (request == 'S')
        {
            // Calculate sine value and scale to 2-byte signed integer
            float sine_value = sin(2 * PI * frequency * time_counter);
            int16_t scaled_sine = (int16_t)round(sine_value * 32767.0);
            // Send the 2 bytes of the scaled value
            Serial.write((byte *)&scaled_sine, sizeof(scaled_sine));
        }
        else if (request == 'C')
        {
            // Calculate cosine value and scale to 2-byte signed integer
            float cosine_value = cos(2 * PI * frequency * time_counter);
            int16_t scaled_cosine = (int16_t)round(cosine_value * 32767.0);
            // Send the 2 bytes of the scaled value
            Serial.write((byte *)&scaled_cosine, sizeof(scaled_cosine));
        }
    }
}
