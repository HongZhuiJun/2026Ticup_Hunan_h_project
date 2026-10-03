#ifndef BALL_CONTROL_H   /* 防止 ball_control.h 被重复包含 */
#define BALL_CONTROL_H   /* 定义 ball_control.h 的头文件保护宏 */

#include "k230_to_stm32.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file ball_control.h
 * @brief Application 层：钢球平衡控制的赛题任务。
 *
 * 与 OLED 菜单的分工：
 *   modules/oled_menu 只负责显示与按键，不含控制代码；本模块在初始化时把
 *   5 个任务函数注册进菜单，菜单的三级页面每轮调用一次被选中的那个。
 */

/**
 * @brief 初始化钢球控制：注册任务函数、准备控制环。
 *
 * @note 在 OLED 初始化和视觉模块初始化之后调用一次。
 *
 * @param k230_instance K230 视觉模块实例；为 NULL 时控制任务进入安全失效状态。
 */
void BallControlInit(K230ToStm32Instance *k230_instance);

/**
 * @brief 控制主入口：进入菜单，由用户在菜单里选任务。
 *
 * @warning 内部调用 Main_Menu()，**不会返回**。
 */
void BallControlTask(void);

#ifdef __cplusplus
}
#endif

#endif /* BALL_CONTROL_H */
