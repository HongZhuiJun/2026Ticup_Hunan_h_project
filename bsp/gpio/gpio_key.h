#ifndef GPIO_KEY_H
#define GPIO_KEY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 读取按键 B1 状态。
 * @return 1=按下，0=未按下
 */
uint8_t Key1_IsPressed(void);

/**
 * @brief 读取按键 B2 状态。
 * @return 1=按下，0=未按下
 */
uint8_t Key2_IsPressed(void);

/**
 * @brief 读取按键 B3 状态。
 * @return 1=按下，0=未按下
 */
uint8_t Key3_IsPressed(void);

#ifdef __cplusplus
}
#endif

#endif /* GPIO_KEY_H */
