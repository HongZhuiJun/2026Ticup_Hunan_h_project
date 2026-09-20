#ifndef KEY_H
#define KEY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Key_GetNum() 返回的键码 */
#define KEY_NUM_NONE 0u
#define KEY_NUM_KEY1 1u
#define KEY_NUM_KEY2 2u
#define KEY_NUM_KEY3 3u

/**
 * @brief 获取被按下并松开的按键键码（带回弹消抖）。
 * @return 键码 KEY_NUM_KEY1/KEY_NUM_KEY2/KEY_NUM_KEY3；
 *         无按键按下时返回 KEY_NUM_NONE。
 */
uint8_t Key_GetNum(void);

#ifdef __cplusplus
}
#endif

#endif /* KEY_H */
