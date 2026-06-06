#include <Wire.h>
#include "LidarLiteV3.h"

LidarLiteV3 lidar;

void setup()
{
    Serial.begin(115200);
    while (!Serial);   /* wait for USB CDC to connect to host */

    Serial.println("LIDAR-Lite v3 — Arduino Uno R4");

    if (!lidar.begin()) {
        Serial.println("ERROR: LIDAR not found (I2C addr 0x62). Check wiring.");
        while (1) {}
    }
    Serial.println("LIDAR ready");
}

void loop()
{
    uint16_t cm;
    if (lidar.readDistance(cm)) {
        Serial.print("Distance: ");
        Serial.print(cm);
        Serial.println(" cm");
    } else {
        Serial.println("ERROR: measurement failed");
    }
    delay(50);
}
