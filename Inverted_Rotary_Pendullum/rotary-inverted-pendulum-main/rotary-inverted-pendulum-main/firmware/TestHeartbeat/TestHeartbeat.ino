// Simple heartbeat test - blinks LED in a double-pulse pattern
// Useful for verifying Arduino is alive and running

unsigned long heartbeat_count = 0;

void setup()
{
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.println("Heartbeat Test");
    Serial.println("Double-pulse pattern: ON-OFF-ON-OFF-pause");
}

void loop()
{
    heartbeat_count++;

    // Double pulse pattern
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);

    // Print heartbeat count
    Serial.print("Heartbeat #");
    Serial.println(heartbeat_count);
}
