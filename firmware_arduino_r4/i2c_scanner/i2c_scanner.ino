#include <Wire.h>

void setup()
{
    Serial.begin(115200);
    while (!Serial);
    Wire.begin();
    Serial.println("Scanning I2C bus...");

    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("  Device at 0x");
            Serial.println(addr, HEX);
            found++;
        }
    }

    if (found == 0)
        Serial.println("  No devices found. Check wiring.");
    else
        Serial.print(found), Serial.println(" device(s) found.");
}

void loop() {}
