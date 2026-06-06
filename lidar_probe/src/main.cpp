/*
 * lidar_probe/src/main.cpp
 * Garmin LIDAR-Lite v3 distance reader for NUCLEO-G474RE
 *
 * Wiring (verified against UM2505 Table 15 and Garmin LIDAR-Lite v3 datasheet):
 *   LIDAR SDA  -> CN5 pin 9  (Arduino D14 / PB9)  -- I2C1_SDA
 *   LIDAR SCL  -> CN5 pin 10 (Arduino D15 / PB8)  -- I2C1_SCL
 *   LIDAR VCC  -> CN6 pin 5  (5 V)
 *   LIDAR GND  -> CN6 pin 6  (GND)
 *   External 680 Ω pull-ups on SDA and SCL to 3.3 V are required.
 *
 * Serial output -> LPUART1 (PA2/PA3) -> ST-Link VCP (default, SB17/SB23 ON)
 *   variant_NUCLEO_G474RE.h: SERIAL_UART_INSTANCE = 101 (LPUART1)
 *
 * Wire defaults: PIN_WIRE_SDA = 14 (PB9), PIN_WIRE_SCL = 15 (PB8)
 *   Confirmed in cores/arduino/pins_arduino.h and variant_NUCLEO_G474RE.cpp.
 *   No explicit Wire.setSDA()/setSCL() calls are needed.
 *
 * Framework: STM32duino (Arduino for STM32), PlatformIO ststm32 platform.
 */

#include <Arduino.h>
#include <Wire.h>

/* ── LIDAR-Lite v3 constants ─────────────────────────────────────────────── */
#define LIDAR_ADDR                  0x62   /* 7-bit I2C address                */
#define REG_ACQ_COMMAND             0x00   /* Acquisition command register      */
#define REG_STATUS                  0x01   /* Status register (bit 0 = busy)    */
#define REG_DIST_HIGH               0x8F   /* MSB-first burst read (0x0F|0x80)  */
#define CMD_MEASURE_WITH_CORRECTION 0x04   /* Acquire with bias correction      */
#define CMD_MEASURE_NO_CORRECTION   0x03   /* Acquire without bias correction   */
#define LIDAR_TIMEOUT_MS            100    /* Max wait for busy-clear (ms)      */

/* ── Helper: write one byte to a LIDAR register ─────────────────────────── */
/* Returns true on success (Wire.endTransmission() == 0).                    */
static bool lidar_write_reg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(LIDAR_ADDR);
    Wire.write(reg);
    Wire.write(val);
    uint8_t err = Wire.endTransmission(); /* issues STOP */
    if (err != 0) {
        Serial.print("I2C ERROR: ");
        Serial.println(err);
        return false;
    }
    return true;
}

/* ── Helper: read len bytes starting at reg into buf ────────────────────── */
/* Issues a write (reg address) with STOP, then a separate read with STOP.   */
/* Returns true on success.                                                   */
static bool lidar_read_bytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
    /* Write register address — full STOP before read (no repeated START) */
    Wire.beginTransmission(LIDAR_ADDR);
    Wire.write(reg);
    uint8_t err = Wire.endTransmission(); /* STOP */
    if (err != 0) {
        Serial.print("I2C ERROR: ");
        Serial.println(err);
        return false;
    }

    /* Read len bytes */
    uint8_t received = Wire.requestFrom((uint8_t)LIDAR_ADDR, len);
    if (received != len) {
        Serial.print("I2C ERROR: short read (");
        Serial.print(received);
        Serial.println(")");
        return false;
    }
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = Wire.read();
    }
    return true;
}

/* ── setup ───────────────────────────────────────────────────────────────── */
void setup(void)
{
    /* LPUART1 → ST-Link VCP (PA2/PA3, SERIAL_UART_INSTANCE=101) */
    Serial.begin(115200);

    /* Wait up to 2 s for the VCP to enumerate on the host */
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 2000) {
        /* spin */
    }

    /* I2C1: SDA=PB9 (D14), SCL=PB8 (D15) — Wire defaults, no setSDA/setSCL needed */
    Wire.begin();
    Wire.setClock(400000); /* 400 kHz fast mode */

    /* LIDAR-Lite v3 needs ~22 ms after power-on before first command */
    delay(25);

    /* Probe: 0-byte write to check for ACK */
    Wire.beginTransmission(LIDAR_ADDR);
    uint8_t probe_err = Wire.endTransmission();
    if (probe_err != 0) {
        Serial.println("ERROR: LIDAR not found at 0x62");
        /* Halt: blink LED at 2 Hz (250 ms on / 250 ms off) */
        pinMode(LED_BUILTIN, OUTPUT);
        while (true) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(250);
            digitalWrite(LED_BUILTIN, LOW);
            delay(250);
        }
    }

    Serial.println("LIDAR-Lite v3 probe ready");
}

/* ── loop ────────────────────────────────────────────────────────────────── */
void loop(void)
{
    /*
     * measurement_count tracks cycles; every 100th measurement uses
     * CMD_MEASURE_NO_CORRECTION (0x03) to skip bias correction for speed.
     * All other cycles use CMD_MEASURE_WITH_CORRECTION (0x04).
     */
    static uint8_t measurement_count = 0;

    /* Step 1: Trigger acquisition */
    uint8_t cmd = (measurement_count == 0) ? CMD_MEASURE_NO_CORRECTION
                                            : CMD_MEASURE_WITH_CORRECTION;
    if (!lidar_write_reg(REG_ACQ_COMMAND, cmd)) {
        /* Error already printed inside helper */
        delay(100);
        return;
    }

    /* Step 2: Poll STATUS register until bit 0 = 0 (not busy) */
    uint32_t deadline = millis() + LIDAR_TIMEOUT_MS;
    bool timed_out = false;
    while (true) {
        uint8_t status = 0;
        if (!lidar_read_bytes(REG_STATUS, &status, 1)) {
            /* I2C error during poll — abort this cycle */
            delay(100);
            return;
        }
        if ((status & 0x01) == 0) {
            break; /* measurement complete */
        }
        if (millis() >= deadline) {
            timed_out = true;
            break;
        }
    }

    if (timed_out) {
        Serial.println("ERROR: LIDAR timeout waiting for measurement");
        delay(100);
        return;
    }

    /* Step 3: Read 2 bytes from 0x8F (auto-increment: reads 0x0F then 0x10) */
    uint8_t dist_buf[2] = {0, 0};
    if (!lidar_read_bytes(REG_DIST_HIGH, dist_buf, 2)) {
        delay(100);
        return;
    }
    uint16_t distance_cm = ((uint16_t)dist_buf[0] << 8) | dist_buf[1];

    /* Step 4: Print result */
    if (distance_cm == 1) {
        /* Sensor reports 1 cm for invalid / no-target condition */
        Serial.println("DIST: -- (no target)");
    } else {
        Serial.print("DIST: ");
        Serial.print(distance_cm);
        Serial.println(" cm");
    }

    /* Advance counter; reset at 100 so next cycle uses no-correction command */
    measurement_count++;
    if (measurement_count >= 100) {
        measurement_count = 0;
    }

    delay(100); /* ~10 Hz measurement rate */
}
