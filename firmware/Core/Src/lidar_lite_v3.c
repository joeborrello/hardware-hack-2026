#include "lidar_lite_v3.h"

static HAL_StatusTypeDef write_reg(LidarLiteV3 *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return HAL_I2C_Master_Transmit(dev->hi2c, LIDAR_I2C_ADDR, buf, 2, LIDAR_TIMEOUT_MS);
}

static HAL_StatusTypeDef read_reg(LidarLiteV3 *dev, uint8_t reg, uint8_t *out, uint16_t len)
{
    HAL_StatusTypeDef st;
    st = HAL_I2C_Master_Transmit(dev->hi2c, LIDAR_I2C_ADDR, &reg, 1, LIDAR_TIMEOUT_MS);
    if (st != HAL_OK) return st;
    return HAL_I2C_Master_Receive(dev->hi2c, LIDAR_I2C_ADDR, out, len, LIDAR_TIMEOUT_MS);
}

HAL_StatusTypeDef LIDAR_Init(LidarLiteV3 *dev, I2C_HandleTypeDef *hi2c)
{
    dev->hi2c       = hi2c;
    dev->read_count = 0;

    /* Verify the device is reachable */
    return HAL_I2C_IsDeviceReady(hi2c, LIDAR_I2C_ADDR, 3, LIDAR_TIMEOUT_MS);
}

HAL_StatusTypeDef LIDAR_ReadDistance(LidarLiteV3 *dev, uint16_t *distance_cm)
{
    HAL_StatusTypeDef st;

    /* Alternate bias-correction: every LIDAR_BIAS_INTERVAL reads use 0x04 */
    uint8_t cmd = (dev->read_count % LIDAR_BIAS_INTERVAL == 0)
                  ? LIDAR_CMD_MEASURE_BIAS
                  : LIDAR_CMD_MEASURE_NOBIAS;

    st = write_reg(dev, LIDAR_REG_ACQ_CMD, cmd);
    if (st != HAL_OK) return st;

    /* Poll busy bit — typical acquisition time is ~20 ms */
    uint8_t status;
    uint32_t deadline = HAL_GetTick() + LIDAR_TIMEOUT_MS;
    do {
        st = read_reg(dev, LIDAR_REG_STATUS, &status, 1);
        if (st != HAL_OK) return st;
        if (HAL_GetTick() > deadline) return HAL_TIMEOUT;
    } while (status & LIDAR_STATUS_BUSY);

    /* Burst-read both distance bytes in one transaction */
    uint8_t raw[2];
    st = read_reg(dev, LIDAR_REG_DIST_BURST, raw, 2);
    if (st != HAL_OK) return st;

    *distance_cm = ((uint16_t)raw[0] << 8) | raw[1];
    dev->read_count++;
    return HAL_OK;
}
