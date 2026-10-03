/**
 * @file pid.c
 * @brief Module 层：通用位置 PID 实现。
 *
 * 本文件不使用宏定义，PID 参数通过 PIDInstance 结构体和 PIDInit() 传入。
 */

#include "pid.h"
#include <stddef.h>

/**
 * @brief 把浮点数限制在指定范围内。
 *
 * @param value   待限幅的数值。
 * @param minimum 允许的最小值。
 * @param maximum 允许的最大值。
 * @return 限幅后的数值。
 */
static float PIDClampFloat(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }

    if (value < minimum) {
        return minimum;
    }

    return value;
}

/**
 * @brief 初始化 PID 实例。
 *
 * 保存 PID 参数、输出限幅和积分限幅，并复位内部状态。
 *
 * @param pid          PID 实例。
 * @param kp           比例系数。
 * @param ki           积分系数。
 * @param kd           微分系数。
 * @param dt_s         初始控制周期，单位 s。
 * @param output_min   输出下限。
 * @param output_max   输出上限。
 * @param integral_min 积分项下限。
 * @param integral_max 积分项上限。
 */
void PIDInit(PIDInstance *pid,
             float kp,
             float ki,
             float kd,
             float dt_s,
             float output_min,
             float output_max,
             float integral_min,
             float integral_max)
{
    if (pid == NULL) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt_s = (dt_s > 0.0f) ? dt_s : 0.05f;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
    PIDReset(pid);
}

/**
 * @brief 复位 PID 的全部内部状态。
 *
 * 清零积分项、上一次测量值和初始化标志，保留 PID 参数。
 *
 * @param pid PID 实例。
 */
void PIDReset(PIDInstance *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->initialized = 0u;
}

/**
 * @brief 只清零 PID 积分项。
 *
 * 用于目标切换、视觉失效或防止积分饱和。
 *
 * @param pid PID 实例。
 */
void PIDResetIntegral(PIDInstance *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->integral = 0.0f;
}

/**
 * @brief 计算一次 PID 输出。
 *
 * 计算流程：
 *   1. error = target - measurement；
 *   2. 微分作用在测量值上，避免目标突变造成微分冲击；
 *   3. 积分项单独限幅；
 *   4. 输出限幅；
 *   5. 输出饱和时抑制积分继续累加。
 *
 * @param pid         PID 实例。
 * @param target      目标值。
 * @param measurement 测量值。
 * @return 限幅后的 PID 输出。
 */
float PIDCalculate(PIDInstance *pid, float target, float measurement)
{
    float error;
    float derivative;
    float integral_next;
    float output;

    if (pid == NULL) {
        return 0.0f;
    }

    error = target - measurement;

    if ((pid->initialized != 0u) && (pid->dt_s > 0.0f)) {
        /* 微分作用在测量值上，避免目标阶跃时产生微分冲击。 */
        derivative = -(measurement - pid->prev_measurement) / pid->dt_s;
    } else {
        derivative = 0.0f;
        pid->initialized = 1u;
    }

    integral_next = pid->integral + (pid->ki * error * pid->dt_s);
    integral_next = PIDClampFloat(integral_next, pid->integral_min, pid->integral_max);

    output = (pid->kp * error) + integral_next + (pid->kd * derivative);

    /* 输出已经饱和且误差仍在把输出往同一方向推时，拒绝本次积分累加。 */
    if (((output > pid->output_max) && (error > 0.0f)) ||
        ((output < pid->output_min) && (error < 0.0f))) {
        output = (pid->kp * error) + pid->integral + (pid->kd * derivative);
    } else {
        pid->integral = integral_next;
    }

    output = PIDClampFloat(output, pid->output_min, pid->output_max);
    pid->prev_measurement = measurement;

    return output;
}
