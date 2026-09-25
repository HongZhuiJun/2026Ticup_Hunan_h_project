#ifndef K230_TO_STM32_H
#define K230_TO_STM32_H

#include <stdint.h>
#include "usart1_k230.h" /* 需要 UART_HandleTypeDef */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file k230_to_stm32.h
 * @brief Module 层：K230 视觉帧接收、校验与「像素坐标 → 水管物理位置」换算。
 *
 * 数据流：
 *   K230 --USART1 115200--> BSP(usart1_k230) --中断回调--> 本模块状态机
 *        --> K230_Data_s 快照 --主循环读取--> Application（钢珠位置闭环）
 *
 * 协议：13 字节定长帧，大端序
 *   字节 0-3   帧头        0xAA 0x55 0x01 0x08
 *   字节 4-5   current_x   int16_t  钢球当前像素 X 坐标，-1 表示钢球丢失
 *   字节 6-7   target_x    int16_t  目标像素 X 坐标，协议固定为 335
 *   字节 8-9   reserved_1  uint16_t 保留，固定 0
 *   字节 10-11 reserved_2  uint16_t 保留，固定 0
 *   字节 12    checksum    uint8_t  前 12 字节逐字节异或
 */

/* ============================ 协议常量 ============================ */

#define K230_FRAME_LEN              13u  /* 整帧长度(字节) */
#define K230_FRAME_HEADER_LEN        4u  /* 帧头长度(字节) */
#define K230_FRAME_CHECKSUM_INDEX   12u  /* 校验字节在帧内的下标 */

#define K230_FRAME_HEADER_0       0xAAu
#define K230_FRAME_HEADER_1       0x55u
#define K230_FRAME_HEADER_2       0x01u
#define K230_FRAME_HEADER_3       0x08u

/* 载荷字段在帧内的下标 */
#define K230_FRAME_CURRENT_X_INDEX   4u
#define K230_FRAME_TARGET_X_INDEX    6u

/* 钢球丢失标志：协议规定 current_x == -1 表示目标丢失，
   此时**不得**用上一帧位置继续闭环 */
#define K230_BALL_LOST_PX          (-1)

/* ====================== 像素 → 物理位置标定 ====================== */
/*
 * 正负两侧各用一条直线，共用 0 cm 参考点。分两段是因为 2026-09-25 实测
 * 发现两侧比例不同。
 *
 * 【右半轴】335 px 以上，0 ~ +10 cm
 *   把钢珠放在刻度上读 pos：
 *     实际(cm) [ 1     2     3     4     5     6     7     8     9     10  ]
 *     显示(cm) [ 0.86  1.67  2.68  3.67  4.62  5.50  6.32  7.31  8.30  9.08]
 *   反推 K230 报的像素后线性拟合（零点固定 335）得 27.83 px/cm，
 *   即 +10 cm 落在 335 + 278.3 = 613.3 px，而不是原以为的 640 px。
 *   原常数 305/10 = 30.5 px/cm 偏大约 9%，这正是"误差在 0 cm 处为 0、
 *   向 10 cm 线性增大到 0.92 cm"的成因——增益误差的典型特征。
 *   换算过去，相机视场比水管右端多出约 (640-613) × 0.036 ≈ 1 cm。
 *
 * 【左半轴】335 px 以下，0 ~ -11 cm
 *   实测该段误差很小，沿用原始标定：335 px / 11 cm = 30.45 px/cm。
 *
 * ⚠ 两段斜率不同，0 cm 处有约 9% 的增益跳变。若后续实测发现左半轴也存在
 * 同样约 9% 的比例误差（也就是相机视场在水管两端各多出约 1 cm），说明两侧
 * 其实同比例，那就该改成单段全局比例：把 K230_CM_PER_PIXEL_LEFT 换成
 * K230_CM_PER_PIXEL_RIGHT，此时 0 px 会对应约 -11.8 cm 而不是 -11 cm。
 */
#define K230_PIXEL_ZERO_PX       335.0f /* 水管 0 cm 对应的像素（即 TARGET_X） */
#define K230_CM_ZERO               0.0f /* 水管中点 */

/* 左半轴标定点：0 px ↔ -11 cm */
#define K230_PIXEL_LEFT_PX         0.0f
#define K230_CM_LEFT            (-11.0f)

/* 右半轴标定点：613.3 px ↔ +10 cm（实测拟合值，不是画面右端 640 px） */
#define K230_PIXEL_RIGHT_PX      613.3f
#define K230_CM_RIGHT             10.0f

/* 两段各自的比例，单位 cm/px。编译期算好，避免在中断里做软件浮点除法 */
#define K230_CM_PER_PIXEL_LEFT \
    ((K230_CM_ZERO - K230_CM_LEFT) / (K230_PIXEL_ZERO_PX - K230_PIXEL_LEFT_PX))
#define K230_CM_PER_PIXEL_RIGHT \
    ((K230_CM_RIGHT - K230_CM_ZERO) / (K230_PIXEL_RIGHT_PX - K230_PIXEL_ZERO_PX))

/* 合法像素范围：超出视为误检。协议允许 current_x 取到 640。
   注意 613 px 以上已在水管右端之外，会外推出大于 +10 cm 的值 */
#define K230_PIXEL_VALID_MIN      0
#define K230_PIXEL_VALID_MAX    640

/* ============================ 运行参数 ============================ */

/* 视觉链路超时(ms)。K230 约 20Hz 发送，超时取 200ms（约 4 个发包周期），
   既不会误判短暂丢帧，又能在视觉断开后及时把位置判为无效 */
#define K230_VISION_TIMEOUT_MS   200u

/* DMA 接收缓冲(字节)。K230 单帧 13 字节，256 字节约合 19 帧。
   - Normal 模式：只要上位机不在一次回调间隔内连发 19 帧就够用
   - Circular 模式：决定"消费端最多能被阻塞多久"= 256B / 260B/s ≈ 1 秒，
     超过这个时间缓冲会被静默覆盖丢数据，所以不要在主循环里放长延时 */
#define K230_RX_DMA_BUF_SIZE     256u

/* 实例池大小。每个实例内含 256 字节 DMA 缓冲，占 RAM 较多，
   所以按当前实际需要（仅一路 K230）分配；后续增加视觉路数时改这里即可 */
#define K230_TO_STM32_MAX_INSTANCE 1u

/* ============================== 类型 ============================== */

typedef struct K230ToStm32Instance K230ToStm32Instance;

/** @brief 注册参数。 */
typedef struct {
    UART_HandleTypeDef *uart_handle; /* 例如 &huart1 */
} K230ToStm32_Init_Config_s;

/**
 * @brief 一帧视觉数据解析结果（快照）。
 *
 * @note 本结构体由接收中断写入、主循环读取。读取请统一走
 *       K230ToStm32GetData()，它内部做了临界区保护；不要直接访问实例字段。
 */
typedef struct {
    int16_t current_x_px;        /* 钢球当前像素 X 坐标，丢失时为负值 */
    int16_t target_x_px;         /* K230 给出的目标像素 X 坐标，协议固定 335 */
    float   pos_cm;              /* 钢球物理位置(cm)；pos_valid 为 0 时无意义 */
    float   target_pos_cm;       /* 目标物理位置(cm)，按同一标定换算，通常为 0 */
    uint8_t ball_detected;       /* 1 = 本帧检测到钢球，0 = 钢球丢失 */
    uint8_t pos_valid;           /* 1 = pos_cm 可信且未超时，可用于闭环 */
    uint32_t last_frame_time_ms; /* 最近一帧合法数据的 HAL_GetTick() 时刻 */
} K230_Data_s;

/** @brief 模块统计，用于联调定位「协议不对」还是「链路不通」。 */
typedef struct {
    uint32_t frame_count;          /* 通过校验的合法帧总数 */
    uint32_t checksum_error_count; /* 校验失败丢弃的帧数 */
    uint32_t lost_frame_count;     /* 合法帧中 current_x 表示钢球丢失的帧数 */
} K230ToStm32_Stats_s;

/* ============================ 外部接口 ============================ */

/**
 * @brief 注册 K230 视觉接收模块（内部会一并注册 BSP 层串口实例）。
 *
 * @param config 注册参数，不可为 NULL。
 * @return 模块实例指针；参数非法、串口 DMA 未配置或实例池已满时返回 NULL。
 *
 * @note 注册不启动接收，需要再调用 K230ToStm32Start()。
 */
K230ToStm32Instance *K230ToStm32Register(const K230ToStm32_Init_Config_s *config);

/**
 * @brief 启动视觉接收。
 *
 * @param instance 模块实例，可为 NULL（此时直接返回 0）。
 * @return 1 = 启动成功，0 = 失败。
 *
 * @note 上电初始化时调用一次即可，后续由驱动自动重挂 DMA。
 */
uint8_t K230ToStm32Start(K230ToStm32Instance *instance);

/**
 * @brief 读取最新一帧数据快照。
 *
 * @param instance 模块实例，不可为 NULL。
 * @param data_out 输出结构体，不可为 NULL。
 * @return 1 = 数据可用（pos_cm 可信，可闭环）；0 = 不可用。
 *
 * @note 返回值恒等于 data_out->pos_valid。即使返回 0，结构体中的
 *       current_x_px 等调试字段仍然是有效的，方便区分「钢球丢失」和
 *       「视觉离线」两种情况。
 */
uint8_t K230ToStm32GetData(K230ToStm32Instance *instance, K230_Data_s *data_out);

/**
 * @brief 读取钢球物理位置(单位 cm)，是给控制环用的最简接口。
 *
 * @param instance   模块实例，不可为 NULL。
 * @param pos_cm_out 输出物理位置(cm)，不可为 NULL。
 * @return 1 = 位置有效并已写入 pos_cm_out；0 = 无效，**不会**写出任何值。
 *
 * @note 返回 0 时钢球丢失或视觉离线，控制环必须停止位置闭环、
 *       按安全策略处理，禁止沿用上一次的位置（开发规范 §12.1）。
 */
uint8_t K230ToStm32GetPosCm(K230ToStm32Instance *instance, float *pos_cm_out);

/**
 * @brief 查询视觉链路是否在线（超时判定）。
 *
 * @param instance 模块实例，可为 NULL。
 * @return 1 = 在线（K230_VISION_TIMEOUT_MS 内收到过合法帧），0 = 离线。
 */
uint8_t K230ToStm32IsOnline(K230ToStm32Instance *instance);

/**
 * @brief 读取模块统计。
 *
 * @param instance  模块实例，不可为 NULL。
 * @param stats_out 输出结构体，不可为 NULL。
 */
void K230ToStm32GetStats(K230ToStm32Instance *instance, K230ToStm32_Stats_s *stats_out);

#ifdef __cplusplus
}
#endif

#endif /* K230_TO_STM32_H */
