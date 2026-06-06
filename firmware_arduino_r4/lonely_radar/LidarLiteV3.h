#pragma once
#include <Wire.h>
#include <stdint.h>

class LidarLiteV3 {
public:
    explicit LidarLiteV3(TwoWire &wire = Wire, uint8_t addr = 0x62);

    /* Call in setup(). Returns false if the sensor is not reachable. */
    bool begin();

    /* Trigger a measurement and return distance in cm.
     * Returns false on I2C error or timeout. */
    bool readDistance(uint16_t &distance_cm);

private:
    TwoWire &_wire;
    uint8_t  _addr;
    uint32_t _readCount;

    bool writeReg(uint8_t reg, uint8_t val);
    bool readReg(uint8_t reg, uint8_t *buf, uint8_t len);
};
