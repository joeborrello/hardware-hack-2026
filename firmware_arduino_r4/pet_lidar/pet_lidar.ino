/* LIDAR-Lite v3 wiring (Arduino Uno R4):
 *   Pin 1 (Power)        -> 5V
 *   Pin 2 (GND)          -> GND
 *   Pin 3 (Mode control) -> unconnected
 *   Pin 4 (SCL)          -> SCL
 *   Pin 5 (SDA)          -> SDA
 *   Pin 6 (Power enable) -> 5V
 */

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
