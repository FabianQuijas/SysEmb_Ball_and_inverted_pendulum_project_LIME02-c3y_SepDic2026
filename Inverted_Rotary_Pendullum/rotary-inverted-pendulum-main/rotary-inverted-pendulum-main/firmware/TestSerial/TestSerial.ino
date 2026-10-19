const long BAUD_RATE = 115200;

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);  // LED on during setup

    Serial.begin(BAUD_RATE); // Start serial communication at defined baud rate
    while (!Serial) { ; }    // Wait for the serial port to connect

    // Flush any garbage data from buffer
    while (Serial.available() > 0) {
        Serial.read();
    }

    digitalWrite(LED_BUILTIN, LOW);  // LED off when ready
}

void loop()
{
    if (Serial.available() > 0)
    {
        digitalWrite(LED_BUILTIN, HIGH);  // Flash LED when receiving

        char incomingByte = Serial.read(); // Read the incoming byte
        Serial.write(incomingByte);        // Echo it back to the sender

        digitalWrite(LED_BUILTIN, LOW);
    }
}
