/* Blinks the green LED (PA5/D13) so we can confirm code is running
   independently of serial, then tries both Serial and Serial2. */

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    Serial2.begin(115200);
    delay(2000);

    Serial.println("Serial OK");
    Serial2.println("Serial2 OK");
}

void loop()
{
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("running (Serial)");
    Serial2.println("running (Serial2)");
    delay(500);

    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
}
