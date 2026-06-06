/* main.c — STM32 Nucleo G474RE + LIDAR-Lite v3
 *
 * CubeMX settings required before building:
 *   Connectivity → I2C1
 *     Mode:            I2C
 *     Speed Mode:      Fast Mode (400 kHz)  ← LIDAR-Lite v3 supports up to 400 kHz
 *     GPIO:            PB8 = I2C1_SCL (Arduino D15)
 *                      PB9 = I2C1_SDA (Arduino D14)
 *   Connectivity → USART2
 *     Mode:            Asynchronous, 115200 8N1
 *     GPIO:            PA2 = USART2_TX, PA3 = USART2_RX  (routed to ST-Link VCP)
 *   Project Manager → Advanced Settings → USART2: HAL (enable printf redirect below)
 */

#include "main.h"
#include "lidar_lite_v3.h"
#include <stdio.h>

/* CubeMX-generated peripheral handles */
I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart2;

/* Redirect printf to USART2 (ST-Link Virtual COM Port) */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* ---------- Forward declarations for CubeMX init stubs ---------- */
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

/* ---------------------------------------------------------------- */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();

    printf("LIDAR-Lite v3 — Nucleo G474RE\r\n");

    LidarLiteV3 lidar;
    if (LIDAR_Init(&lidar, &hi2c1) != HAL_OK) {
        printf("ERROR: LIDAR not found on I2C1 (0x62). Check wiring.\r\n");
        Error_Handler();
    }
    printf("LIDAR detected OK\r\n");

    uint16_t distance_cm;
    while (1) {
        if (LIDAR_ReadDistance(&lidar, &distance_cm) == HAL_OK) {
            printf("Distance: %u cm\r\n", distance_cm);
        } else {
            printf("ERROR: measurement failed\r\n");
        }
        HAL_Delay(50);  /* ~20 readings/sec */
    }
}

/* ================================================================
 * CubeMX-generated peripheral init — replace with your .ioc output
 * ================================================================ */

void SystemClock_Config(void)
{
    /* STM32G474RE default: HSI 16 MHz → PLL → 170 MHz SYSCLK
     * Replace this body entirely with what CubeMX generates.       */
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState       = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM       = RCC_PLLM_DIV4;
    osc.PLL.PLLN       = 85;
    osc.PLL.PLLP       = RCC_PLLP_DIV2;
    osc.PLL.PLLQ       = RCC_PLLQ_DIV2;
    osc.PLL.PLLR       = RCC_PLLR_DIV2;
    HAL_RCC_OscConfig(&osc);

    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                         RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4);
}

static void MX_I2C1_Init(void)
{
    /* 400 kHz Fast Mode, timing for 170 MHz PCLK1.
     * Value computed by CubeMX; verify against your clock config.  */
    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = 0x00F07BFF; /* 400 kHz @ 170 MHz */
    hi2c1.Init.OwnAddress1      = 0;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
    HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /* I2C1 and USART2 GPIO are configured inside their HAL_*_MspInit callbacks
     * (stm32g4xx_hal_msp.c), which CubeMX generates automatically.            */
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
