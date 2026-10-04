/**
 * @file ball_control.c
 * @brief Application 层：钢球平衡控制的 5 个赛题任务。
 *
 * 控制约定：
 *   - 水管倾角 > 0：右端下降，球向右滚；
 *   - 水管倾角 < 0：右端上升，球向左滚；
 *   - 电机逻辑角度 = 水管倾角 × 机械比例 K（左右两侧 K 不同）；
 *   - 电机逻辑角度 > 0 使用 dir=1，< 0 使用 dir=0。
 *
 * 当前标定数据（连杆机构左右不对称，因此分开标定）：
 *   - 右端下降（球向右滚）：K_RIGHT = 4.0f；
 *   - 右端上升（球向左滚）：K_LEFT  = 6.0f。
 *
 * 每个任务使用独立的一套参数，见下面的 ball_task_configs[] 参数表。
 *
 * 注意：当前 Makefile 没有单独编译 modules/pid.c，因此这里临时包含 pid.c。
 * 以后把 modules/pid.c 加入 Makefile 的 C_SOURCES 后，删除下面这行 include。
 */
#include "ball_control.h"
#include "oled_menu.h"
#include "k230_to_stm32.h"
#include "x_v2.h"
#include "pid.h"
#include "pid.c"
#include <math.h>
#include <stddef.h>

/* ============================ 机械与控制参数 ============================ */

#define BALL_MOTOR_ADDR                   1u      /* X42S 电机地址，默认地址 1 */
#define BALL_MOTOR_DEG_PER_TUBE_DEG_RIGHT 4.0f    /* 正方向机械比例：右端下降时，水管 1° 对应电机 4° */
#define BALL_MOTOR_DEG_PER_TUBE_DEG_LEFT  6.0f   /* 负方向机械比例：右端上升时，水管 1° 对应电机 6° */
#define BALL_TUBE_ANGLE_LIMIT_DEG         3.5f    /* 最终水管倾角限幅：±3.5° */
#define BALL_MOTOR_SPEED_RPM              60.0f   /* 电机位置模式最大速度：60 RPM */
#define BALL_MOTOR_CURRENT_MA             1200u   /* 电机位置模式最大电流：1200 mA */

/* K230 位置和速度滤波。视觉约 20Hz，控制只在收到新帧时执行。 */
#define BALL_POS_FILTER_ALPHA             0.50f   /* 位置一阶滤波系数：越大越跟随，越小越平滑 */
#define BALL_VEL_FILTER_ALPHA             0.30f   /* 速度一阶滤波系数：越大越灵敏，越小越平滑 */
#define BALL_MAX_VELOCITY_CMPS            50.0f   /* 速度估计限幅：±50 cm/s，防止异常帧造成大速度 */

/* PID 初值：后续必须根据实车曲线继续整定。 */
#define BALL_PID_KP                       0.4f    /* PID 比例系数：水管角度/cm */
#define BALL_PID_KI                       0.3f    /* PID 积分系数：水管角度/(cm·s)，*/
#define BALL_PID_KD                       0.04f    /* PID 微分系数：水管角度/(cm/s)，起刹车阻尼作用 */
#define BALL_PID_OUT_LIMIT_DEG            3.5f    /* PID 输出限幅：±3.5° 水管角度 */
#define BALL_PID_INTEGRAL_LIMIT_DEG       1.4f    /* 积分项限幅，*/
#define BALL_PID_INTEGRAL_ZONE_CM         3.0f    /* 只在位置误差小于 3cm 时才允许积分 */
#define BALL_PID_INTEGRAL_VEL_CMPS        5.0f    /* 只在球速小于 5cm/s 时才允许积分 */

/* 目标位置斜坡：先给球一个平滑的位置参考，避免目标阶跃。 */
#define BALL_REF_POS_GAIN                 2.0f    /* 目标位置斜坡的比例增益：越大参考目标追得越快 */
#define BALL_REF_MAX_VEL_CMPS             6.0f    /* 目标参考位置最大速度：6 cm/s */
#define BALL_REF_ACCEL_CMPS2              12.0f   /* 目标参考位置最大加速度：12 cm/s² */

/* 连续跟踪任务：目标 = 8cm × sin(2πt/4s)。 */
#define BALL_TRACK_AMPLITUDE_CM           8.0f    /* 任务3正弦跟踪幅值：±8 cm */
#define BALL_TRACK_PERIOD_S               4.0f    /* 任务3正弦跟踪周期：4 s */
#define BALL_TRACK_OMEGA_RADPS            (2.0f * 3.14159265f / BALL_TRACK_PERIOD_S) /* 任务3正弦角频率：2π/T，单位 rad/s */

#define BALL_FF_ACCEL_DEG_PER_CMPS2       0.082f  /* 加速度前馈增益：1cm/s² 目标加速度约对应 0.082°水管角度，仅任务3使用 */

/*
 * 上面这个 0.082 的来历（实心球在斜面上纯滚动，实心球转动惯量 I = (2/5)mr²）：
 *   平动：mg·sinθ - f = m·a
 *   转动：f·r = I·(a/r) = (2/5)m·r²·(a/r)  →  f = (2/5)m·a
 *   两式合并：mg·sinθ = m·a + (2/5)m·a = (7/5)m·a
 *   约掉质量 m：a = (5/7)·g·sinθ
 * 角度换成度、g = 980cm/s²、sinθ ≈ θ·π/180：
 *   a = (5/7)×980×(π/180)×θ = 12.22·θ   →   θ = a / 12.22 = 0.0818·a
 */

/* 稳定判据。 */
#define BALL_STABLE_ERROR_CM              1.2f    /* 稳定判据位置误差：≤1.2 cm */
#define BALL_STABLE_VEL_CMPS              2.0f    /* 稳定判据球速：≤2 cm/s */
#define BALL_STABLE_TIME_S                3.0f    /* 稳定持续时间：≥3 s */
#define BALL_FOLD_ERROR_CM                1.2f    /* 任务2折返到位判据：≤1.2 cm */
#define BALL_ANTI_DISTURB_ERROR_CM        0.5f    /* 任务4抗干扰稳定误差预留判据：≤0.5 cm */

/* 同一个任务退出后再次进入时，用视觉帧时间戳判断是否要重新开始。 */
#define BALL_TASK_RESET_GAP_MS            100u    /* 视觉帧间隔超过 100ms，视为重新进入任务并复位状态 */

typedef enum {
    BALL_REF_RAMP = 0,   /* 目标经过速度斜坡生成 */
    BALL_REF_SINE,       /* 正弦目标直接生成 */
} Ball_RefMode_e;

/**
 * @brief 单个任务的 PID、限幅和参考轨迹配置。
 *
 * 每个任务在进入时调用 BallControl_ApplyTaskConfig()，
 * 自动把当前任务对应的一套参数加载到 PID 和输出限制中。
 */
typedef struct {
    float kp;                  /* 比例系数，单位：水管角度/cm */
    float ki;                  /* 积分系数，单位：水管角度/(cm·s) */
    float kd;                  /* 微分系数，单位：水管角度/(cm/s) */
    float vel_ff_gain;         /* 速度前馈增益，单位：水管角度/(cm/s)，与 kd 独立，互不影响 */
    float pid_out_limit_deg;   /* PID 输出限幅，单位：度 */
    float integral_limit_deg;  /* 积分项限幅，单位：度 */
    float integral_zone_cm;    /* 允许积分的位置误差范围，单位：cm */
    float integral_vel_cmps;   /* 允许积分的球速范围，单位：cm/s */
    float tube_angle_limit_deg;/* 最终水管倾角限幅，单位：度 */
    float motor_speed_rpm;     /* 电机位置模式最大速度，单位：RPM */
    uint16_t motor_current_ma; /* 电机位置模式最大电流，单位：mA */
    float ref_pos_gain;        /* 目标位置斜坡比例增益，单位：1/s */
    float ref_max_vel_cmps;    /* 目标参考位置最大速度，单位：cm/s */
    float ref_accel_cmps2;     /* 目标参考位置最大加速度，单位：cm/s² */
    uint8_t feedforward_enabled; /* 1=启用速度和加速度前馈，0=纯 PID */
    float ff_accel_gain;       /* 加速度前馈增益，单位：度/(cm/s²) */
} BallTaskConfig_t;

/**
 * @brief 5 个任务各自的参数表。
 *
 * 数值完全相同，但互相独立：
 *   - 任务1直接引用上面的 BALL_xxx 宏，方便和调参笔记对照；
 *   - 任务2/3/4/5全部写死成具体数字，不引用宏。
 *
 * 这样任务1的宏怎么改都不会影响其他任务；要调哪个任务，
 * 就只改那一格里的数字，其余任务保持原样。
 *
 * 注意：改了任务1的宏之后，如果希望其他任务也跟着变，
 * 必须手动把新数值抄到对应任务的那一格。
 */
static const BallTaskConfig_t ball_task_configs[MENU_TASK_NUM] = {
    [MENU_TASK_STABLE_POSITION] = {          /* 任务1：已调好的基准参数 */
        .kp = BALL_PID_KP,
        .ki = BALL_PID_KI,
        .kd = BALL_PID_KD,
        .vel_ff_gain = 0.05f,                /* 速度前馈增益，与 kd 独立 */
        .pid_out_limit_deg = BALL_PID_OUT_LIMIT_DEG,
        .integral_limit_deg = BALL_PID_INTEGRAL_LIMIT_DEG,
        .integral_zone_cm = BALL_PID_INTEGRAL_ZONE_CM,
        .integral_vel_cmps = BALL_PID_INTEGRAL_VEL_CMPS,
        .tube_angle_limit_deg = BALL_TUBE_ANGLE_LIMIT_DEG,
        .motor_speed_rpm = BALL_MOTOR_SPEED_RPM,
        .motor_current_ma = BALL_MOTOR_CURRENT_MA,
        .ref_pos_gain = BALL_REF_POS_GAIN,
        .ref_max_vel_cmps = BALL_REF_MAX_VEL_CMPS,
        .ref_accel_cmps2 = BALL_REF_ACCEL_CMPS2,
        .feedforward_enabled = 0u,
        .ff_accel_gain = BALL_FF_ACCEL_DEG_PER_CMPS2,
    },
    [MENU_TASK_BACK_AND_FORTH] = {           /* 任务2 */
        .kp = 0.4f,
        .ki = 0.00f,
        .kd = 0.03f,
        .vel_ff_gain = 0.05f,                /* 速度前馈增益，与 kd 独立 */
        .pid_out_limit_deg = 3.5f,
        .integral_limit_deg = 0.0f,
        .integral_zone_cm = 3.0f,
        .integral_vel_cmps = 5.0f,
        .tube_angle_limit_deg = 3.5f,
        .motor_speed_rpm = 60.0f,
        .motor_current_ma = 1200u,
        .ref_pos_gain = 2.0f,
        .ref_max_vel_cmps = 6.0f,
        .ref_accel_cmps2 = 12.0f,
        .feedforward_enabled = 0u,
        .ff_accel_gain = 0.082f,
    },
    [MENU_TASK_CONTINUOUS_TRACK] = {         /* 任务3 */
        .kp = 0.8f,
        .ki = 0.0f,
        .kd = 0.15f,
        .vel_ff_gain = 0.05f,                /* 速度前馈增益，与 kd 独立 */
        .pid_out_limit_deg = 3.5f,
        .integral_limit_deg = 0.0f,
        .integral_zone_cm = 3.0f,
        .integral_vel_cmps = 5.0f,
        .tube_angle_limit_deg = 3.5f,
        .motor_speed_rpm = 60.0f,
        .motor_current_ma = 1200u,
        .ref_pos_gain = 2.0f,
        .ref_max_vel_cmps = 6.0f,
        .ref_accel_cmps2 = 12.0f,
        .feedforward_enabled = 1u,           /* 任务3做正弦跟踪，前馈先留着，暂时不用可改成 0u */
        .ff_accel_gain = 0.082f,
    },
    [MENU_TASK_ANTI_DISTURBANCE] = {         /* 任务4 */
        .kp = 0.4f,
        .ki = 0.0f,
        .kd = 0.1f,
        .vel_ff_gain = 0.05f,                /* 速度前馈增益，与 kd 独立 */
        .pid_out_limit_deg = 3.5f,
        .integral_limit_deg = 0.0f,
        .integral_zone_cm = 3.0f,
        .integral_vel_cmps = 5.0f,
        .tube_angle_limit_deg = 3.5f,
        .motor_speed_rpm = 60.0f,
        .motor_current_ma = 1200u,
        .ref_pos_gain = 2.0f,
        .ref_max_vel_cmps = 6.0f,
        .ref_accel_cmps2 = 12.0f,
        .feedforward_enabled = 0u,
        .ff_accel_gain = 0.082f,
    },
    [MENU_TASK_TARGET_POSITION] = {          /* 任务5 */
        .kp = 0.4f,
        .ki = 0.3f,
        .kd = 0.04f,
        .vel_ff_gain = 0.05f,                /* 速度前馈增益，与 kd 独立 */
        .pid_out_limit_deg = 3.5f,
        .integral_limit_deg = 1.4f,
        .integral_zone_cm = 3.0f,
        .integral_vel_cmps = 5.0f,
        .tube_angle_limit_deg = 3.5f,
        .motor_speed_rpm = 60.0f,
        .motor_current_ma = 1200u,
        .ref_pos_gain = 2.0f,
        .ref_max_vel_cmps = 6.0f,
        .ref_accel_cmps2 = 12.0f,
        .feedforward_enabled = 0u,
        .ff_accel_gain = 0.082f,
    },
};

/**
 * @brief 钢球控制的运行时状态。
 *
 * 保存 K230 数据处理结果、PID 实例、任务状态机、参考轨迹和 OLED 显示缓存。
 * 该结构体只在 ball_control.c 内部使用。
 */
typedef struct {
    K230ToStm32Instance *k230_instance; /* K230 视觉模块实例，NULL 表示视觉不可用 */
    PIDInstance pid;                    /* 球位置 PID 实例 */
    const BallTaskConfig_t *config;     /* 当前任务对应的参数表项 */

    Menu_Task_e active_task;            /* 当前正在执行的任务编号 */
    uint8_t task_active;                /* 是否已经进入过任务：0=未进入，1=已进入 */
    uint8_t task_phase;                 /* 任务阶段，任务1用它区分 +6cm 和 -6cm 两个阶段 */
    float task_elapsed_s;               /* 当前任务已运行时间，单位 s，主要用于任务3正弦相位 */
    float stable_time_s;                /* 连续满足稳定条件的累计时间，单位 s */

    uint8_t filter_initialized;         /* 滤波器和参考位置是否已经初始化 */
    uint8_t have_last_frame;            /* 是否保存过上一帧 K230 时间戳 */
    uint8_t vision_was_valid;           /* 上一次视觉数据是否有效 */
    uint8_t fail_safe_applied;          /* 视觉失效后的安全回零动作是否已经执行过 */
    uint32_t last_frame_time_ms;        /* 上一帧有效 K230 数据的时间戳，单位 ms */
    float last_dt_s;                    /* 本次控制周期，单位 s */

    float filtered_pos_cm;              /* 滤波后的钢球位置，单位 cm */
    float last_filtered_pos_cm;         /* 上一周期滤波后的钢球位置，单位 cm */
    float ball_vel_cmps;                /* 滤波后的钢球速度，单位 cm/s，向右为正 */

    float target_cm;                    /* 当前任务的最终目标位置，单位 cm，斜坡任务使用 */
    float ref_pos_cm;                   /* 经过斜坡或正弦生成后的参考目标位置，单位 cm */
    float ref_vel_cmps;                 /* 参考目标速度，单位 cm/s */
    float ref_acc_cmps2;                /* 参考目标加速度，单位 cm/s² */

    float display_pos_cm;               /* 发送给 OLED 显示的滤波后位置，单位 cm */
    uint8_t display_valid;              /* 显示位置是否有效：1=有效 */
    uint8_t feedforward_enabled;        /* 是否启用前馈：只有任务3为1，其他任务为0 */
} BallControl_t;

static BallControl_t ball_control;

/* ============================ 内部函数 ============================ */

/**
 * @brief 复位钢球控制的运行时状态。
 *
 * 复位 PID、任务阶段、滤波器和参考轨迹，防止上一个任务的状态影响新任务。
 */
static void BallControl_ResetRuntime(void)
{
    PIDReset(&ball_control.pid);

    ball_control.task_phase = 0u;
    ball_control.task_elapsed_s = 0.0f;
    ball_control.stable_time_s = 0.0f;

    ball_control.filter_initialized = 0u;
    ball_control.have_last_frame = 0u;
    ball_control.vision_was_valid = 0u;
    ball_control.fail_safe_applied = 0u;
    ball_control.last_frame_time_ms = 0u;
    ball_control.last_dt_s = 0.05f;

    ball_control.filtered_pos_cm = 0.0f;
    ball_control.last_filtered_pos_cm = 0.0f;
    ball_control.ball_vel_cmps = 0.0f;

    ball_control.target_cm = 0.0f;
    ball_control.ref_pos_cm = 0.0f;
    ball_control.ref_vel_cmps = 0.0f;
    ball_control.ref_acc_cmps2 = 0.0f;

    ball_control.display_pos_cm = 0.0f;
    ball_control.display_valid = 0u;
}

/**
 * @brief 把指定任务的参数加载到 PID 和运行配置中。
 *
 * @param task 任务编号，取值 MENU_TASK_xxx。
 */
static void BallControl_ApplyTaskConfig(Menu_Task_e task)
{
    const BallTaskConfig_t *config;

    if ((uint8_t)task >= (uint8_t)MENU_TASK_NUM) {
        return;
    }

    config = &ball_task_configs[task];
    ball_control.config = config;
    ball_control.feedforward_enabled = config->feedforward_enabled;

    PIDInit(&ball_control.pid,
            config->kp,
            config->ki,
            config->kd,
            0.05f,
            -config->pid_out_limit_deg,
            config->pid_out_limit_deg,
            -config->integral_limit_deg,
            config->integral_limit_deg);
}

/**
 * @brief 进入指定任务并准备该任务的运行状态。
 *
 * 如果任务编号发生变化，会复位上一任务的状态。
 * 同时加载该任务对应的一整套 PID、限幅、轨迹和前馈参数。
 *
 * @param task 当前要执行的任务编号。
 */
static void BallControl_BeginTask(Menu_Task_e task)
{
    if ((ball_control.task_active == 0u) || (ball_control.active_task != task)) {
        ball_control.active_task = task;
        ball_control.task_active = 1u;
        BallControl_ResetRuntime();
        BallControl_ApplyTaskConfig(task);
    }
}

/**
 * @brief 把水管倾角命令转换成电机绝对角度命令。
 *
 * tube_angle_deg > 0 使用 dir=1，右端下降，换算比例 K_RIGHT；
 * tube_angle_deg < 0 使用 dir=0，右端上升，换算比例 K_LEFT。
 *
 * @param tube_angle_deg 水管目标倾角，单位度，正方向为右端下降。
 */
static void BallControl_ApplyTubeAngle(float tube_angle_deg)
{
    const BallTaskConfig_t *config = ball_control.config;
    float motor_angle_deg;
    float pos_deg;
    uint8_t dir;

    if (config == NULL) {
        return;
    }

    if (tube_angle_deg > config->tube_angle_limit_deg) {
        tube_angle_deg = config->tube_angle_limit_deg;
    } else if (tube_angle_deg < -config->tube_angle_limit_deg) {
        tube_angle_deg = -config->tube_angle_limit_deg;
    }

    if (tube_angle_deg >= 0.0f) {
        motor_angle_deg = tube_angle_deg * BALL_MOTOR_DEG_PER_TUBE_DEG_RIGHT;
    } else {
        motor_angle_deg = tube_angle_deg * BALL_MOTOR_DEG_PER_TUBE_DEG_LEFT;
    }
    if (motor_angle_deg >= 0.0f) {
        dir = 1u;
        pos_deg = motor_angle_deg;
    } else {
        dir = 0u;
        pos_deg = -motor_angle_deg;
    }

    X_V2_Bypass_Pos_LV_LC_Control(BALL_MOTOR_ADDR,
                                  dir,
                                  config->motor_speed_rpm,
                                  pos_deg,
                                  1u,
                                  false,
                                  config->motor_current_ma);
}

/**
 * @brief 更新目标位置参考轨迹。
 *
 * 根据最终目标 target_cm，生成平滑的 ref_pos_cm、ref_vel_cmps 和 ref_acc_cmps2，
 * 避免目标位置直接阶跃造成球被甩出去。
 *
 * @param dt_s 本次控制周期，单位 s。
 */
static void BallControl_UpdateReferenceRamp(float dt_s)
{
    const BallTaskConfig_t *config = ball_control.config;
    float error_cm = ball_control.target_cm - ball_control.ref_pos_cm;
    float desired_vel_cmps;
    float dv_cmps;
    float max_dv_cmps;

    if (config == NULL) {
        return;
    }

    desired_vel_cmps = error_cm * config->ref_pos_gain;

    if (desired_vel_cmps > config->ref_max_vel_cmps) {
        desired_vel_cmps = config->ref_max_vel_cmps;
    } else if (desired_vel_cmps < -config->ref_max_vel_cmps) {
        desired_vel_cmps = -config->ref_max_vel_cmps;
    }

    dv_cmps = desired_vel_cmps - ball_control.ref_vel_cmps;
    max_dv_cmps = config->ref_accel_cmps2 * dt_s;
    if (dv_cmps > max_dv_cmps) {
        dv_cmps = max_dv_cmps;
    } else if (dv_cmps < -max_dv_cmps) {
        dv_cmps = -max_dv_cmps;
    }

    ball_control.ref_vel_cmps += dv_cmps;
    ball_control.ref_acc_cmps2 = dv_cmps / dt_s;
    ball_control.ref_pos_cm += ball_control.ref_vel_cmps * dt_s;
}

/**
 * @brief 执行一次位置闭环控制。
 *
 * 读取参考位置和滤波后的球位置，计算 PID 输出，再根据任务类型决定是否叠加：
 *   - 速度补偿；
 *   - 加速度前馈。
 *
 * 任务1/2/4/5 只使用 PID；任务3保留前馈。
 */
static void BallControl_UpdateControlLoop(void)
{
    const BallTaskConfig_t *config = ball_control.config;
    float error_cm = ball_control.ref_pos_cm - ball_control.filtered_pos_cm;
    float pid_out_deg;
    float velocity_trim_deg;
    float accel_ff_deg;
    float tube_angle_deg;

    if (config == NULL) {
        return;
    }

    /* 大误差或高速阶段不积分，避免积分饱和。 */
    if ((fabsf(error_cm) > config->integral_zone_cm) ||
        (fabsf(ball_control.ball_vel_cmps) > config->integral_vel_cmps)) {
        PIDResetIntegral(&ball_control.pid);
    }

    ball_control.pid.dt_s = ball_control.last_dt_s;
    pid_out_deg = PIDCalculate(&ball_control.pid,
                               ball_control.ref_pos_cm,
                               ball_control.filtered_pos_cm);

    /*
     * 任务1/2/4/5 只使用 PID 反馈，不再叠加速度和加速度前馈。
     * 任务3保留这两项，用于后续正弦跟踪调试。
     */
    velocity_trim_deg = 0.0f;
    accel_ff_deg = 0.0f;
    if (ball_control.feedforward_enabled != 0u) {
        /*
         * 速度前馈用独立的 vel_ff_gain，不再借用 kd。
         * 以前两者共用一个系数，调阻尼时会把前馈一起改掉，没法分开整定。
         */
        velocity_trim_deg = config->vel_ff_gain * ball_control.ref_vel_cmps;
        /*
         * 速度前馈的来历：完整微分项 Kd·(de/dt) = Kd·ṙ - Kd·ẋ（其中 e = r - x）。
         * PID 只对测量值求导，内部算的是 -Kd·ẋ，丢掉了 Kd·ṙ，由上面这一行补回。
         */
        accel_ff_deg = config->ff_accel_gain * ball_control.ref_acc_cmps2;
    }

    tube_angle_deg = pid_out_deg + velocity_trim_deg + accel_ff_deg;
    BallControl_ApplyTubeAngle(tube_angle_deg);
}

/**
 * @brief 判断钢球是否达到稳定条件。
 *
 * 稳定条件：
 *   - 球与目标的位置误差不超过 error_limit_cm；
 *   - 球的速度不超过 BALL_STABLE_VEL_CMPS。
 *
 * @param error_limit_cm 允许的位置误差，单位 cm。
 * @return 1 = 稳定；0 = 未稳定。
 */
static uint8_t BallControl_IsStable(float error_limit_cm)
{
    return ((fabsf(ball_control.target_cm - ball_control.filtered_pos_cm) <= error_limit_cm) &&
            (fabsf(ball_control.ball_vel_cmps) <= BALL_STABLE_VEL_CMPS)) ? 1u : 0u;
}

/**
 * @brief 读取一帧 K230 数据并执行一次控制。
 *
 * 处理内容：
 *   - 检查视觉数据是否有效；
 *   - 按视觉帧时间戳计算控制周期；
 *   - 对球位置和球速度做低通滤波；
 *   - 生成参考目标（斜坡或正弦）；
 *   - 调用 BallControl_UpdateControlLoop() 输出电机命令。
 *
 * @param ref_mode 参考目标生成模式：
 *                 BALL_REF_RAMP = 斜坡目标；
 *                 BALL_REF_SINE = 正弦目标。
 * @return 1 = 本次处理了新的有效视觉帧；0 = 没有新数据或视觉无效。
 */
static uint8_t BallControl_Update(Ball_RefMode_e ref_mode)
{
    K230_Data_s data;
    float dt_s;
    float raw_vel_cmps;
    float ref_phase;

    if (ball_control.k230_instance == NULL) {
        BallControl_ApplyTubeAngle(0.0f);
        return 0u;
    }

    if (K230ToStm32GetData(ball_control.k230_instance, &data) == 0u) {
        /* 钢球丢失或视觉超时：只执行一次安全动作，禁止沿用旧位置。 */
        if (ball_control.fail_safe_applied == 0u) {
            PIDReset(&ball_control.pid);
            BallControl_ApplyTubeAngle(0.0f);
            ball_control.fail_safe_applied = 1u;
        }
        ball_control.vision_was_valid = 0u;
        return 0u;
    }

    if ((ball_control.have_last_frame != 0u) &&
        ((data.last_frame_time_ms - ball_control.last_frame_time_ms) > BALL_TASK_RESET_GAP_MS)) {
        /* 任务中间隔太久，按重新进入任务处理。 */
        BallControl_ResetRuntime();
    }

    if ((ball_control.have_last_frame != 0u) &&
        (data.last_frame_time_ms == ball_control.last_frame_time_ms)) {
        return 0u;
    }

    if (ball_control.have_last_frame != 0u) {
        dt_s = (float)(data.last_frame_time_ms - ball_control.last_frame_time_ms) * 0.001f;
        if (dt_s < 0.02f) {
            dt_s = 0.02f;
        } else if (dt_s > 0.10f) {
            dt_s = 0.10f;
        }
    } else {
        dt_s = 0.05f;
    }

    ball_control.last_frame_time_ms = data.last_frame_time_ms;
    ball_control.have_last_frame = 1u;
    ball_control.vision_was_valid = 1u;
    ball_control.fail_safe_applied = 0u;
    ball_control.last_dt_s = dt_s;

    if (ball_control.filter_initialized == 0u) {
        ball_control.filtered_pos_cm = data.pos_cm;
        ball_control.last_filtered_pos_cm = data.pos_cm;
        ball_control.ball_vel_cmps = 0.0f;
        ball_control.ref_pos_cm = data.pos_cm;
        ball_control.ref_vel_cmps = 0.0f;
        ball_control.ref_acc_cmps2 = 0.0f;
        ball_control.filter_initialized = 1u;
    } else {
        ball_control.filtered_pos_cm += BALL_POS_FILTER_ALPHA *
                                        (data.pos_cm - ball_control.filtered_pos_cm);

        raw_vel_cmps = (ball_control.filtered_pos_cm - ball_control.last_filtered_pos_cm) / dt_s;
        ball_control.last_filtered_pos_cm = ball_control.filtered_pos_cm;

        if (raw_vel_cmps > BALL_MAX_VELOCITY_CMPS) {
            raw_vel_cmps = BALL_MAX_VELOCITY_CMPS;
        } else if (raw_vel_cmps < -BALL_MAX_VELOCITY_CMPS) {
            raw_vel_cmps = -BALL_MAX_VELOCITY_CMPS;
        }

        ball_control.ball_vel_cmps += BALL_VEL_FILTER_ALPHA *
                                      (raw_vel_cmps - ball_control.ball_vel_cmps);
    }

    ball_control.display_pos_cm = ball_control.filtered_pos_cm;
    ball_control.display_valid = 1u;

    if (ref_mode == BALL_REF_SINE) {
        ref_phase = BALL_TRACK_OMEGA_RADPS * ball_control.task_elapsed_s;
        ball_control.ref_pos_cm = BALL_TRACK_AMPLITUDE_CM * sinf(ref_phase);
        ball_control.ref_vel_cmps = BALL_TRACK_AMPLITUDE_CM * BALL_TRACK_OMEGA_RADPS * cosf(ref_phase);
        ball_control.ref_acc_cmps2 = -BALL_TRACK_AMPLITUDE_CM * BALL_TRACK_OMEGA_RADPS *
                                     BALL_TRACK_OMEGA_RADPS * sinf(ref_phase);
    } else {
        BallControl_UpdateReferenceRamp(dt_s);
    }

    BallControl_UpdateControlLoop();
    return 1u;
}

/**
 * @brief 把当前球位置、参考目标和球速度推送到 OLED 菜单显示。
 *
 * 只负责显示，不参与控制计算。
 */
static void ball_control_update_menu_position(void)
{
    if (ball_control.display_valid != 0u) {
        OLEDMenu_SetBallPositionCm(ball_control.display_pos_cm);
        OLEDMenu_SetBallTargetCm(ball_control.ref_pos_cm);
        OLEDMenu_SetBallVelocityCmps(ball_control.ball_vel_cmps);
    }
}

/* ============================ 任务函数 ============================ */

/**
 * @brief 任务1：定点稳定控制。
 *
 * 流程：
 *   phase 0：移动到 +6cm，稳定 3 秒；
 *   phase 1：移动到 -6cm，稳定 3 秒；
 *   phase 2：保持在 -6cm。
 */
static void ball_control_task_stable_position(void)
{
    BallControl_BeginTask(MENU_TASK_STABLE_POSITION);

    if (ball_control.task_phase == 0u) {
        ball_control.target_cm = 6.0f;
        if (BallControl_Update(BALL_REF_RAMP) != 0u) {
            if (BallControl_IsStable(BALL_STABLE_ERROR_CM) != 0u) {
                ball_control.stable_time_s += ball_control.last_dt_s;
                if (ball_control.stable_time_s >= BALL_STABLE_TIME_S) {
                    ball_control.task_phase = 1u;
                    ball_control.stable_time_s = 0.0f;
                    PIDResetIntegral(&ball_control.pid);
                }
            } else {
                ball_control.stable_time_s = 0.0f;
            }
        }
    } else if (ball_control.task_phase == 1u) {
        ball_control.target_cm = -6.0f;
        if (BallControl_Update(BALL_REF_RAMP) != 0u) {
            if (BallControl_IsStable(BALL_STABLE_ERROR_CM) != 0u) {
                ball_control.stable_time_s += ball_control.last_dt_s;
                if (ball_control.stable_time_s >= BALL_STABLE_TIME_S) {
                    ball_control.task_phase = 2u;
                    ball_control.stable_time_s = 0.0f;
                    PIDResetIntegral(&ball_control.pid);
                }
            } else {
                ball_control.stable_time_s = 0.0f;
            }
        }
    } else {
        ball_control.target_cm = -6.0f;
        (void)BallControl_Update(BALL_REF_RAMP);
    }

    ball_control_update_menu_position();
}

/**
 * @brief 任务2：折返运动与时效。
 *
 * 流程：
 *   phase 0：从当前位置移动到 +6cm；
 *   phase 1：折返移动到 -6cm；
 *   phase 2：返回 0cm 并保持。
 */
static void ball_control_task_back_and_forth(void)
{
    BallControl_BeginTask(MENU_TASK_BACK_AND_FORTH);

    if (ball_control.task_phase == 0u) {
        ball_control.target_cm = 6.0f;
        if (BallControl_Update(BALL_REF_RAMP) != 0u) {
            if (fabsf(ball_control.filtered_pos_cm - 6.0f) <= BALL_FOLD_ERROR_CM) {
                ball_control.task_phase = 1u;
                PIDResetIntegral(&ball_control.pid);
            }
        }
    } else if (ball_control.task_phase == 1u) {
        ball_control.target_cm = -6.0f;
        if (BallControl_Update(BALL_REF_RAMP) != 0u) {
            if (fabsf(ball_control.filtered_pos_cm + 6.0f) <= BALL_FOLD_ERROR_CM) {
                ball_control.task_phase = 2u;
                PIDResetIntegral(&ball_control.pid);
            }
        }
    } else {
        ball_control.target_cm = 0.0f;
        (void)BallControl_Update(BALL_REF_RAMP);
    }

    ball_control_update_menu_position();
}

/**
 * @brief 任务3：连续跟踪运动。
 *
 * 目标位置按周期 4 秒的正弦信号变化，幅值 ±8cm。
 * 该任务保留速度和加速度前馈，用于后续单独调试。
 */
static void ball_control_task_continuous_track(void)
{
    BallControl_BeginTask(MENU_TASK_CONTINUOUS_TRACK);

    if (BallControl_Update(BALL_REF_SINE) != 0u) {
        ball_control.task_elapsed_s += ball_control.last_dt_s;
        if (ball_control.task_elapsed_s >= BALL_TRACK_PERIOD_S) {
            ball_control.task_elapsed_s -= BALL_TRACK_PERIOD_S;
        }
    }

    ball_control_update_menu_position();
}

/**
 * @brief 任务4：抗干扰能力。
 *
 * 目标保持在 0cm，球被外力推走后由 PID 自动拉回中心。
 */
static void ball_control_task_anti_disturbance(void)
{
    BallControl_BeginTask(MENU_TASK_ANTI_DISTURBANCE);
    ball_control.target_cm = 0.0f;
    (void)BallControl_Update(BALL_REF_RAMP);
    ball_control_update_menu_position();
}

/**
 * @brief 任务5：指定位置控制。
 *
 * 从 OLED 菜单读取用户设定目标位置，然后使用位置 PID 移动到该位置。
 */
static void ball_control_task_target_position(void)
{
    BallControl_BeginTask(MENU_TASK_TARGET_POSITION);
    ball_control.target_cm = (float)OLEDMenu_GetTargetPositionCm();
    (void)BallControl_Update(BALL_REF_RAMP);
    ball_control_update_menu_position();
}

/* ============================ 外部接口 ============================ */

/**
 * @brief 初始化钢球控制模块。
 *
 * 保存 K230 实例、初始化 PID，并把 5 个任务函数注册给 OLED 菜单。
 *
 * @param k230_instance K230 视觉模块实例；为 NULL 时控制任务会进入安全失效状态。
 */
void BallControlInit(K230ToStm32Instance *k230_instance)
{
    ball_control.k230_instance = k230_instance;
    ball_control.active_task = MENU_TASK_STABLE_POSITION;
    ball_control.task_active = 0u;

    BallControl_ResetRuntime();
    BallControl_ApplyTaskConfig(MENU_TASK_STABLE_POSITION);

    OLEDMenu_RegisterTask(MENU_TASK_STABLE_POSITION, ball_control_task_stable_position);
    OLEDMenu_RegisterTask(MENU_TASK_BACK_AND_FORTH, ball_control_task_back_and_forth);
    OLEDMenu_RegisterTask(MENU_TASK_CONTINUOUS_TRACK, ball_control_task_continuous_track);
    OLEDMenu_RegisterTask(MENU_TASK_ANTI_DISTURBANCE, ball_control_task_anti_disturbance);
    OLEDMenu_RegisterTask(MENU_TASK_TARGET_POSITION, ball_control_task_target_position);
}

/**
 * @brief 钢球控制主入口。
 *
 * 进入 OLED 菜单，由菜单持续调用当前选中的任务函数。
 * 该函数内部不会返回。
 */
void BallControlTask(void)
{
    (void)Main_Menu();
}
