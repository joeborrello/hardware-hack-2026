#include <Wire.h>
#include "LidarLiteV3.h"

LidarLiteV3 lidar;

void setup()
{
    Serial.begin(115200);
    /* Give the ST-Link VCP time to enumerate on the host */
    delay(2000);

    Serial.println("LIDAR-Lite v3 — Nucleo G474RE");

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
