#ifndef OLED_MENU_H
#define OLED_MENU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file oled_menu.h
 * @brief Module 层：OLED 三级菜单。只负责显示与按键，不含任何控制代码。
 *
 * 与 Application 层的分工：
 *   - 菜单只管「选哪个任务」，任务本身由 Application（ball_control.c）实现，
 *     上电时通过 OLEDMenu_RegisterTask() 注册进来。本模块不 include 任何
 *     Application 头文件，依赖方向固定为 Application -> Module。
 *   - 任务页要显示的小球位置、以及发挥部分的目标位置，通过下面两个
 *     取值/设值接口在两侧传递。
 */

/**
 * @brief 三级菜单的任务编号。
 *
 * 顺序与基础部分的条目一致，Application 层按这个编号注册任务函数。
 */
typedef enum {
    MENU_TASK_STABLE_POSITION = 0,   /* 1.定点稳定控制 */
    MENU_TASK_BACK_AND_FORTH,        /* 2.折返运动与时效 */
    MENU_TASK_CONTINUOUS_TRACK,      /* 3.连续跟踪运动 */
    MENU_TASK_ANTI_DISTURBANCE,      /* 4.抗干扰能力 */
    MENU_TASK_TARGET_POSITION,       /* 发挥部分：指定位置 */
    MENU_TASK_NUM,                   /* 编号个数，不是任务 */
} Menu_Task_e;

/**
 * @brief 任务函数原型，由 Application 层实现后注册给菜单。
 *
 * @warning 任务页每轮循环调用一次，**必须立刻返回**：按键扫描（KEY3 返回
 *          二级菜单）和页面刷新都在菜单里，任务函数里不能有死循环、长延时
 *          或直接调用 OLED 画图（每轮会清屏重绘，画上去也会被覆盖）。
 *          要显示的量用 OLEDMenu_SetBallPositionCm() 推给菜单。
 */
typedef void (*Menu_TaskFunc_t)(void);

/**
 * @brief 注册某个任务编号对应的任务函数。
 *
 * @param task      任务编号，取值 MENU_TASK_xxx。
 * @param task_func 任务函数，传 NULL 表示清除该任务的注册。
 *
 * @note 上电初始化时调用一次即可。编号超出范围时忽略。
 */
void OLEDMenu_RegisterTask(Menu_Task_e task, Menu_TaskFunc_t task_func);

/**
 * @brief 设置三级菜单顶部显示的小球位置。
 *
 * @param position_cm 小球当前位置，单位：cm，水管中点为 0、右侧为正。
 *
 * @note 任务函数每轮调用一次，菜单在刷新页面时读取。
 */
void OLEDMenu_SetBallPositionCm(int8_t position_cm);

/**
 * @brief 读取发挥部分里用户调好的目标位置。
 *
 * @return 目标位置，单位：cm，范围 MENU_TARGET_POSITION_MIN_CM ~ MAX_CM。
 */
int8_t OLEDMenu_GetTargetPositionCm(void);

/**
 * @brief 菜单入口：逐级进入 一级 -> 二级 -> 三级 菜单。
 *
 * 一级菜单（基础部分 / 发挥部分）确认后进入对应的二级菜单，二级菜单确认后
 * 进入三级菜单执行任务，各级菜单的"上一步"返回上一级菜单。函数内部循环
 * 处理菜单显示与按键扫描，不会主动返回。
 *
 * @return 固定返回 0
 */
int Main_Menu(void);

#ifdef __cplusplus
}
#endif

#endif /* OLED_MENU_H */
