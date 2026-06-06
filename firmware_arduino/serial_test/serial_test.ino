void setup()
{
    Serial.begin(115200);
    delay(3000);
    Serial.println("Hello from Nucleo G474RE");
}

void loop()
{
    Serial.println("running...");
    delay(1000);
}
