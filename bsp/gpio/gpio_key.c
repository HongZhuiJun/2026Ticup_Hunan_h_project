/**
 * @file gpio_key.c
 * @brief 按键引脚读取（BSP 层，HAL 实现）。
 *
 * 引脚 B1=PA4、B2=PA5、B3=PA6 已由 CubeMX 在 Core/Src/gpio.c 的
 * MX_GPIO_Init() 配置为上拉输入，按下时读到低电平。
 */
#include "gpio_key.h"
#include "main.h"

uint8_t Key1_IsPressed(void)
{
    return (HAL_GPIO_ReadPin(Button1_GPIO_Port, Button1_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
}

uint8_t Key2_IsPressed(void)
{
    return (HAL_GPIO_ReadPin(Button2_GPIO_Port, Button2_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
}

uint8_t Key3_IsPressed(void)
{
    return (HAL_GPIO_ReadPin(Button3_GPIO_Port, Button3_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
}
