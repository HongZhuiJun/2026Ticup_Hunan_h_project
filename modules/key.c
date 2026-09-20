/**
 * @file key.c
 * @brief 按键扫描模块（模块层）。
 *
 * 通过 BSP 层 gpio_key 读取引脚状态，实现按下识别、回弹消抖
 * 与松手判定，每次完整按压只返回一次键码。
 */
#include "key.h"
#include "gpio_key.h"
#include "delay.h"

/* 消抖等待时间，避开按键按下瞬间的机械抖动 */
#define KEY_DEBOUNCE_TIME_MS 20u


/**
 * @brief 获取被按下并松开的按键键码。
 * @return 键码 KEY_NUM_KEY1/KEY_NUM_KEY2/KEY_NUM_KEY3；
 *         无按键按下时返回 KEY_NUM_NONE。
 */
uint8_t Key_GetNum(void)
{
    uint8_t key_num = KEY_NUM_NONE;

    if (Key1_IsPressed() != 0u) {
        delay_ms(KEY_DEBOUNCE_TIME_MS);
        if (Key1_IsPressed() != 0u) {
            while (Key1_IsPressed() != 0u) { }
            key_num = KEY_NUM_KEY1;
        }
    } else if (Key2_IsPressed() != 0u) {
        delay_ms(KEY_DEBOUNCE_TIME_MS);
        if (Key2_IsPressed() != 0u) {
            while (Key2_IsPressed() != 0u) { }
            key_num = KEY_NUM_KEY2;
        }
    } else if (Key3_IsPressed() != 0u) {
        delay_ms(KEY_DEBOUNCE_TIME_MS);
        if (Key3_IsPressed() != 0u) {
            while (Key3_IsPressed() != 0u) { }
            key_num = KEY_NUM_KEY3;
        }
    }

    return key_num;
}
