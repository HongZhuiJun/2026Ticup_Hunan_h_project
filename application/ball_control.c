/**
 * @file ball_control.c
 * @brief Application 层：钢球平衡控制的 5 个赛题任务。
 *
 * 与菜单的分工（规范 §2.2 的 Application -> Module 单向依赖）：
 *   - 菜单（Module）只管显示和按键，不知道任何控制算法；
 *   - 本模块在 BallControlInit() 里把自己的任务函数注册给菜单，
 *     菜单的三级页面每轮通过函数指针调用一次被选中的那个；
 *   - 任务页要显示的小球位置，用 OLEDMenu_SetBallPositionCm() 推给菜单，
 *     不要在任务函数里直接调 OLED 画图（菜单每轮会清屏重绘）。
 *
 * 每个任务函数都必须是「跑一轮就返回」的写法：
 *   不能有死循环、长延时或阻塞等待，否则菜单的按键扫描会失灵、
 *   KEY3 回不到二级菜单。
 */
#include "ball_control.h"
#include "oled_menu.h"

/**
 * @brief 把当前小球位置推给菜单显示。
 *
 * 位置来自 K230 视觉模块。k230_to_stm32 目前对外只有初始化接口，
 * 还没有给 Application 用的取值接口，补上之后在这里接：
 *     float pos_cm;
 *     if (k230_to_stm32_GetPosCm(&pos_cm) != 0u) {
 *         OLEDMenu_SetBallPositionCm((int8_t)pos_cm);   // 位置无效时保持上次显示
 *     }
 * 注意位置无效（钢球丢失 / 视觉离线）时不要推 0，否则屏幕上会假装球在中间。
 */
static void ball_control_update_menu_position(void)
{
    /* TODO: 读视觉位置并推给菜单 */
}

/* ============================ 任务函数 ============================ */

/**
 * @brief 1. 定点稳定控制：钢球稳定停在指定位置。
 */
static void ball_control_task_stable_position(void)
{
    /* TODO: 位置环 + 电机输出（PID 见 modules/pid，电机见 modules/x_v2） */
    ball_control_update_menu_position();
}

/**
 * @brief 2. 折返运动与时效：钢球在两点间往返，计时。
 */
static void ball_control_task_back_and_forth(void)
{
    /* TODO: 目标位置按折返节奏切换，并记录到达时间 */
    ball_control_update_menu_position();
}

/**
 * @brief 3. 连续跟踪运动：目标位置连续变化时钢球跟随。
 */
static void ball_control_task_continuous_track(void)
{
    /* TODO: 闭环跟随不断变化的目标位置 */
    ball_control_update_menu_position();
}

/**
 * @brief 4. 抗干扰能力：受到扰动后能回到目标位置。
 */
static void ball_control_task_anti_disturbance(void)
{
    /* TODO: 与定点稳定同环，重点在扰动后的恢复 */
    ball_control_update_menu_position();
}

/**
 * @brief 发挥部分：钢球移动到菜单里设定的指定位置。
 */
static void ball_control_task_target_position(void)
{
    /* 目标位置由用户在二级菜单里 ±1cm 调好后确认，用
       OLEDMenu_GetTargetPositionCm() 取回来 */
    /* TODO: 以取回的目标位置做位置闭环 */
    ball_control_update_menu_position();
}

/* ============================ 外部接口 ============================ */

void BallControlInit(void)
{
    OLEDMenu_RegisterTask(MENU_TASK_STABLE_POSITION, ball_control_task_stable_position);
    OLEDMenu_RegisterTask(MENU_TASK_BACK_AND_FORTH, ball_control_task_back_and_forth);
    OLEDMenu_RegisterTask(MENU_TASK_CONTINUOUS_TRACK, ball_control_task_continuous_track);
    OLEDMenu_RegisterTask(MENU_TASK_ANTI_DISTURBANCE, ball_control_task_anti_disturbance);
    OLEDMenu_RegisterTask(MENU_TASK_TARGET_POSITION, ball_control_task_target_position);
}

void BallControlTask(void)
{
    /* 菜单内部是死循环，选任务、执行、返回上级全在里面，不会返回到这里 */
    (void)Main_Menu();
}
