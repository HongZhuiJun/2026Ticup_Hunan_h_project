#ifndef PID_H   /* 防止 pid.h 被重复包含 */
#define PID_H   /* 定义 pid.h 的头文件保护宏 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file pid.h
 * @brief Module 层：通用增量式位置 PID。
 *
 * 本模块只做数学计算，不直接操作电机和视觉硬件。
 * 约定：
 *   - 微分作用在测量值上，避免目标突变造成微分冲击；
 *   - 积分项单独限幅，输出整体再限幅；
 *   - 输出饱和且误差继续增大时，本次不累加积分，抑制积分饱和。
 */

/**
 * @brief PID 运行时实例。
 *
 * 每个 PID 对象独立保存参数、积分状态和上一次测量值，
 * 因此同一个 PID 模块可以同时被多个控制环使用。
 */
typedef struct {
    float kp;                 /* 比例系数，单位由被控量决定 */
    float ki;                 /* 积分系数，输出单位/(误差单位·时间) */
    float kd;                 /* 微分系数，输出单位/(误差单位/时间) */
    float dt_s;               /* 控制周期，单位 s */
    float integral;           /* 积分项，已经乘过 ki，单位与输出相同 */
    float integral_min;       /* 积分项下限，单位与输出相同 */
    float integral_max;       /* 积分项上限，单位与输出相同 */
    float output_min;         /* 输出下限 */
    float output_max;         /* 输出上限 */
    float prev_measurement;   /* 上一次测量值 */
    uint8_t initialized;      /* 是否已经记录过测量值 */
} PIDInstance;

/**
 * @brief 初始化 PID。
 *
 * @param pid            PID 实例，不能为 NULL。
 * @param kp             比例系数。
 * @param ki             积分系数。
 * @param kd             微分系数。
 * @param dt_s           初始控制周期，单位 s。
 * @param output_min     输出下限。
 * @param output_max     输出上限。
 * @param integral_min   积分项下限。
 * @param integral_max   积分项上限。
 */
void PIDInit(PIDInstance *pid,
             float kp,
             float ki,
             float kd,
             float dt_s,
             float output_min,
             float output_max,
             float integral_min,
             float integral_max);

/**
 * @brief 复位 PID 的全部内部状态。
 */
void PIDReset(PIDInstance *pid);

/**
 * @brief 只复位积分项，保留 PID 参数。
 */
void PIDResetIntegral(PIDInstance *pid);

/**
 * @brief 计算一次 PID 输出。
 *
 * @param pid          PID 实例，不能为 NULL。
 * @param target       目标值。
 * @param measurement  测量值。
 * @return 限幅后的控制输出。
 */
float PIDCalculate(PIDInstance *pid, float target, float measurement);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
