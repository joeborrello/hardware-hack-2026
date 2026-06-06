#include "LidarLiteV3.h"
#include <Arduino.h>

#define REG_ACQ_CMD     0x00
#define REG_STATUS      0x01
#define REG_DIST_BURST  0x8F  /* reads high + low byte in one burst */

#define CMD_BIAS        0x04
#define CMD_NO_BIAS     0x03
#define STATUS_BUSY     0x01
#define BIAS_INTERVAL   100
#define ACQ_TIMEOUT_MS  100

LidarLiteV3::LidarLiteV3(TwoWire &wire, uint8_t addr)
    : _wire(wire), _addr(addr), _readCount(0) {}

bool LidarLiteV3::begin()
{
    _wire.begin();
    _wire.setClock(400000);

    _wire.beginTransmission(_addr);
    return (_wire.endTransmission() == 0);
}

bool LidarLiteV3::writeReg(uint8_t reg, uint8_t val)
{
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.write(val);
    return (_wire.endTransmission() == 0);
}

bool LidarLiteV3::readReg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    if (_wire.endTransmission() != 0) return false;
    if (_wire.requestFrom(_addr, len) != len) return false;
    for (uint8_t i = 0; i < len; i++)
        buf[i] = _wire.read();
    return true;
}

bool LidarLiteV3::readDistance(uint16_t &distance_cm)
{
    uint8_t cmd = (_readCount % BIAS_INTERVAL == 0) ? CMD_BIAS : CMD_NO_BIAS;
    if (!writeReg(REG_ACQ_CMD, cmd)) return false;

    uint32_t deadline = millis() + ACQ_TIMEOUT_MS;
    uint8_t  status;
    do {
        if (!readReg(REG_STATUS, &status, 1)) return false;
        if (millis() > deadline) return false;
    } while (status & STATUS_BUSY);

    uint8_t raw[2];
    if (!readReg(REG_DIST_BURST, raw, 2)) return false;

    distance_cm = ((uint16_t)raw[0] << 8) | raw[1];
    _readCount++;
    return true;
}
