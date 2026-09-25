#ifndef USART1_K230_H
#define USART1_K230_H

#include <stdint.h>
#include "usart.h" /* 需要 UART_HandleTypeDef / DMA_HandleTypeDef */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file usart1_k230.h
 * @brief BSP 层：USART1 + DMA1_Channel5「空闲中断不定长接收」驱动。
 *
 * 用途：承载 K230 视觉模块发来的二进制帧流。上位机以固定周期发包，
 * 帧与帧之间存在总线空闲，因此用 IDLE 空闲中断切分数据块，比「定长接收」
 * 更抗抖动：上位机一次多发几帧、或帧长将来变化，都不需要改驱动。
 *
 * 依赖 CubeMX 配置：
 *   - USART1：115200 8N1，使能 global interrupt
 *   - USART1_RX：DMA1_Channel5，PeriphToMemory，MINC，Normal 或 Circular 均可
 *   - NVIC：USART1_IRQn 与 DMA1_Channel5_IRQn 均已使能
 *
 * 数据流：DMA 把字节搬进 rx_dma_buf → 总线空闲时 HAL 触发
 *         HAL_UARTEx_RxEventCallback → 本驱动回调上层 rx_callback。
 *
 * 两种 DMA 模式的取舍（驱动都已支持，改 CubeMX 即可切换）：
 *   - Normal：每次空闲事件后 DMA 停止，驱动重挂。失败是"响"的——
 *     一旦重挂失败或缓冲收满，都会体现在 error_count 上，不会静默丢数据。
 *   - Circular：DMA 永不停，少一处需要恢复的状态。代价是消费不及时时
 *     缓冲会被**静默覆盖**，且需要自己维护读指针与回绕处理。
 *     本工程 20Hz / 13 字节 / 256 字节缓冲 ≈ 1 秒余量，正常不会追上。
 *
 * @note 本模块是唯一直接操作 USART/DMA 的地方（开发规范 §2.4），
 *       Module 层与 Application 层不得出现 HAL_UART_xxx 调用。
 */

/* 已注册实例上限。本工程只有一路 K230，留 2 个槽位是为了后续增加第二路
   串口视觉时不必改动接口。 */
#define K230_USART_MAX_INSTANCE 2u

typedef struct K230UsartInstance K230UsartInstance;

/**
 * @brief 接收回调，**在 USART1 中断上下文中执行**。
 *
 * @param instance 触发的实例。
 * @param data     本次交付的数据，指向本实例的 DMA 缓冲。
 * @param len      字节数，取值 1 ~ rx_dma_buf_size。
 *
 * @warning 回调返回后 data 指向的缓冲随时可能被下一次 DMA 覆盖，
 *          需要保留或延后处理的数据必须在回调内自行拷贝。
 * @warning 回调内禁止延时、打印、阻塞发送（开发规范 §8.1），
 *          只允许做短小的解析或复制。
 * @warning Circular 模式缓冲回绕时，一次中断会**连续调用本回调两次**
 *          （先缓冲尾部、再缓冲头部）。上层回调必须是可重入的字节流
 *          消费者，不能假设"一次回调 = 一帧"。
 */
typedef void (*K230UsartRxCallback)(K230UsartInstance *instance, const uint8_t *data, uint16_t len);

/** @brief 实例注册参数。缓冲区由调用者提供，避免驱动内部隐式占用 RAM。 */
typedef struct {
    UART_HandleTypeDef *uart_handle;    /* 例如 &huart1，其 hdmarx 必须已由 CubeMX 关联 */
    uint8_t  *rx_dma_buf;               /* DMA 接收缓冲，由调用者提供并保证生命周期 */
    uint16_t  rx_dma_buf_size;          /* 缓冲大小(字节)，建议 >= 2 帧长 */
    void     *parent;                   /* 上层实例指针，回调时原样回传 */
    K230UsartRxCallback rx_callback;    /* 收到数据时的回调，可为 NULL */
} K230Usart_Init_Config_s;

/** @brief 链路统计，用于联调时区分「没数据」和「数据被丢弃」。 */
typedef struct {
    uint32_t rx_chunk_count; /* 向上层交付数据的次数；Circular 回绕时分片会多算一次 */
    uint32_t rx_byte_count;  /* 累计收到字节数，115200 8N1 下理论上限约 11520 B/s */
    uint32_t error_count;    /* 过载/帧/噪声错误次数 + 重新挂载接收失败次数 */
} K230Usart_Stats_s;

struct K230UsartInstance {
    UART_HandleTypeDef *uart_handle;
    DMA_HandleTypeDef  *dma_handle;
    uint8_t  *rx_dma_buf;
    uint16_t  rx_dma_buf_size;
    void     *parent;
    K230UsartRxCallback rx_callback;

    uint8_t  dma_circular;  /* 1 = DMA 配置为 Circular（注册时缓存，避免中断里反复读结构体） */
    uint16_t rx_read_pos;   /* Circular 模式：上次已交付到的写指针位置 */

    /* 统计量：rx_* 只由中断写，主循环读单字是原子的，无需加锁 */
    volatile uint32_t rx_chunk_count;
    volatile uint32_t rx_byte_count;
    volatile uint32_t error_count;
};

/**
 * @brief 注册一个 USART+DMA 接收实例。
 *
 * @param config 注册参数，不可为 NULL。
 * @return 实例指针；参数非法、DMA 未关联或实例池已满时返回 NULL。
 *
 * @note 注册只是登记信息，不会立刻开始接收，需要再调用 K230UsartStart()。
 */
K230UsartInstance *K230UsartRegister(const K230Usart_Init_Config_s *config);

/**
 * @brief 启动接收（挂载 DMA + 使能空闲中断）。
 *
 * @param instance 实例指针。
 * @return HAL_OK 表示成功；HAL_ERROR/HAL_BUSY 表示失败，需检查串口与 DMA 配置。
 *
 * @note 上电初始化时调用一次即可。Normal 模式 DMA 在每次空闲事件后
 *       由驱动内部自动重挂，上层不需要反复调用本函数。
 */
HAL_StatusTypeDef K230UsartStart(K230UsartInstance *instance);

/**
 * @brief 读取链路统计。
 *
 * @param instance  实例指针。
 * @param stats_out 输出结构体，不可为 NULL。
 */
void K230UsartGetStats(const K230UsartInstance *instance, K230Usart_Stats_s *stats_out);

#ifdef __cplusplus
}
#endif

#endif /* USART1_K230_H */
