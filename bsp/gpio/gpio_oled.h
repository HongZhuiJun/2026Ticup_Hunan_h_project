#ifndef GPIO_OLED_H
#define GPIO_OLED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief OLED SCL 引脚电平写入。
 *
 * 片上外设(GPIO)的封装位于 BSP 层，仅通过 HAL 操作引脚。
 *
 * @param bit_value 电平值：0 -> 低电平；非 0 -> 高电平
 */
void OLED_W_SCL(uint8_t bit_value);

/**
 * @brief OLED SDA 引脚电平写入。
 *
 * 片上外设(GPIO)的封装位于 BSP 层，仅通过 HAL 操作引脚。
 *
 * @param bit_value 电平值：0 -> 低电平；非 0 -> 高电平
 */
void OLED_W_SDA(uint8_t bit_value);

/**
 * @brief OLED GPIO 初始化。
 *
 * 等待 OLED 上电稳定，并释放 SCL/SDA 总线。
 *
 * @note SCL=PB8、SDA=PB9 已在 Core/Src/gpio.c 的 MX_GPIO_Init() 中配置为
 *       开漏输出并带上拉，本函数不重复配置引脚，只做上电延时与总线释放。
 */
void OLED_GPIO_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* GPIO_OLED_H */
