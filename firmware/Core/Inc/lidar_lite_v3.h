#ifndef LIDAR_LITE_V3_H
#define LIDAR_LITE_V3_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* Default 7-bit I2C address. HAL expects it left-shifted by 1. */
#define LIDAR_I2C_ADDR      (0x62 << 1)

/* Register map */
#define LIDAR_REG_ACQ_CMD   0x00  /* Write here to trigger a measurement */
#define LIDAR_REG_STATUS    0x01  /* Bit 0 = busy */
#define LIDAR_REG_DIST_H    0x0F  /* Distance high byte */
#define LIDAR_REG_DIST_L    0x10  /* Distance low byte  */
#define LIDAR_REG_DIST_BURST 0x8F /* 0x0F | 0x80 — burst-reads both bytes */

/* ACQ_CMD values */
#define LIDAR_CMD_MEASURE_BIAS   0x04
#define LIDAR_CMD_MEASURE_NOBIAS 0x03

#define LIDAR_STATUS_BUSY    0x01

#define LIDAR_TIMEOUT_MS     100
#define LIDAR_BIAS_INTERVAL  100  /* Take a bias-corrected reading every N reads */

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint32_t          read_count;
} LidarLiteV3;

HAL_StatusTypeDef LIDAR_Init(LidarLiteV3 *dev, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef LIDAR_ReadDistance(LidarLiteV3 *dev, uint16_t *distance_cm);

#endif /* LIDAR_LITE_V3_H */
