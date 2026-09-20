/**
 * @file gpio_oled.c
 * @brief OLED 软件 I2C 的 SCL/SDA 引脚驱动（BSP 层，HAL 实现）。
 *
 * 引脚 SCL=PB8、SDA=PB9 由 CubeMX 生成于 Core/Src/gpio.c 的 MX_GPIO_Init()，
 * 本文件不再重复配置，只负责电平写入与初始化时的总线释放。
 */
#include "gpio_oled.h"
#include "main.h"
#include "delay.h"


void OLED_W_SCL(uint8_t bit_value)
{
    HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin,
                      (bit_value != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void OLED_W_SDA(uint8_t bit_value)
{
    HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin,
                      (bit_value != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void OLED_GPIO_Init(void)
{
    delay_ms(10); /* 等待总线稳定，避免上电时的干扰 */

    /* 释放总线：SCL/SDA 开漏输出配置见 gpio.c 的 MX_GPIO_Init() */
    OLED_W_SCL(1u);
    OLED_W_SDA(1u);
}
