/**
 * @file k230_to_stm32.c
 * @brief Module 层：K230 视觉帧接收、校验与物理位置换算。
 *
 * 解析器沿用逐字节状态机的写法：帧头匹配 → 收满载荷 → 校验 → 更新数据。
 * 状态机天然抗错位（失配后能重新同步），且一次回调收到多帧或半帧都能正确处理。
 *
 * 并发说明（开发规范 §8.3）：
 *   - 解析状态机（parse_state / parse_index / frame_buf）只被接收中断访问，
 *     主循环不读，因此不需要保护。
 *   - 数据快照 data 由接收中断写、主循环读，属于单写多读，
 *     读取侧用临界区保护（见 K230ToStm32GetData）。
 */
#include "k230_to_stm32.h"
#include <string.h>

/* 帧头字节表，供状态机逐字节比对 */
static const uint8_t k230_frame_header[K230_FRAME_HEADER_LEN] = {
    K230_FRAME_HEADER_0,
    K230_FRAME_HEADER_1,
    K230_FRAME_HEADER_2,
    K230_FRAME_HEADER_3,
};

struct K230ToStm32Instance {
    K230UsartInstance *usart_instance;

    /* 解析状态机：仅接收中断访问 */
    uint8_t parse_index;                   /* 已匹配/已接收的字节数，0 表示等待帧头 */
    uint8_t frame_buf[K230_FRAME_LEN];     /* 整帧暂存 */
    uint8_t rx_dma_buf[K230_RX_DMA_BUF_SIZE]; /* BSP 层 DMA 接收缓冲，由本实例持有 */

    /* 数据快照：中断写，主循环读 */
    volatile K230_Data_s data;

    /* 统计量：仅接收中断写，主循环读单字是原子的 */
    volatile uint32_t frame_count;
    volatile uint32_t checksum_error_count;
    volatile uint32_t lost_frame_count;
};

/* 实例池：静态分配，控制周期内不做动态内存（开发规范 §7.3） */
static K230ToStm32Instance k230_to_stm32_pool[K230_TO_STM32_MAX_INSTANCE];

/* k230_to_stm32_Init() 成功注册的实例。调用方（main）不再持有这个指针，
   后续取数据 / 查在线状态的接口要拿它，所以在这里留一份 */
static K230ToStm32Instance *k230_to_stm32_instance = NULL;

/* ============================ 内部函数 ============================ */

/**
 * @brief 从字节流读取 int16（大端序）。
 */
static int16_t K230ToStm32ReadInt16BE(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

/* =============== 像素 → 物理位置标定表（唯一要改的地方） ===============
 *
 * 下标 i 对应物理位置 K230_CAL_MIN_CM + i * K230_CAL_STEP_CM。
 *
 * 2026-09-28 实测：把钢珠逐个放在水管 1 cm 刻度上，记录 K230 报出的像素 X。
 * 数据来源为 K230 侧 firmware/main_zdyz_k230.py 的 CAL_X_* 常量，
 * 两边必须一致——换镜头或挪动相机后重新标定，两个文件一起改。
 *
 * 表内相邻两点的差值就是该处的像素比例，畸变一眼可见：
 *     中心 352-318=34, 318-284=34  → 34 px/cm（放大率最大）
 *     两端 27-6=21,   623-598=25   → 21~25 px/cm（被压缩）
 * 中心比边缘大 62%，所以用直线拟合必然有一头对不准。
 *
 * 硬性要求：必须随 cm 单调递增。转换函数依赖这一点做区间查找，
 * 否则插值结果会回跳。改完请肉眼确认一遍相邻值在变大。
 * ====================================================================== */
#define K230_CAL_MIN_CM     (-11) /* 下标 0 对应的物理位置(cm) */
#define K230_CAL_STEP_CM       (1) /* 相邻下标之间的物理间距(cm) */
#define K230_CAL_POINT_NUM    (22)

/* 下标 i 对应的物理位置(cm) */
#define K230_CAL_CM_OF_INDEX(i) (K230_CAL_MIN_CM + (int)(i) * K230_CAL_STEP_CM)

static const int16_t k230_cal_px[K230_CAL_POINT_NUM] = {
    /* -11  -10   -9   -8   -7   -6   -5   -4   -3   -2  cm */
        6,  27,  50,  74, 100, 128, 156, 187, 219, 251,
    /*  -1    0   +1   +2   +3   +4   +5   +6   +7   +8  cm */
      284, 318, 352, 384, 418, 451, 485, 514, 543, 571,
    /*  +9  +10  cm */
      598, 623,
};

/**
 * @brief 像素横坐标 → 水管物理位置：查标定表 + 相邻两点线性插值。
 *
 * @param current_x_px 钢球像素横坐标。
 * @return 物理位置，单位 cm。
 *
 * @note 在接收中断中执行。22 项顺序查找即可（20Hz 下开销可忽略），
 *       不必为二分查找增加复杂度。
 */
static float K230ToStm32PixelToCm(int16_t current_x_px)
{
    float pixel = (float)current_x_px;

    /* 表外：按最近一段的比例外推。钢珠正常不会越出标定范围，
       越界基本是误检，外推出界值交给上层按物理范围处理即可 */
    if (pixel <= (float)k230_cal_px[0]) {
        float slope = (float)K230_CAL_STEP_CM
                    / ((float)k230_cal_px[1] - (float)k230_cal_px[0]);
        return (float)K230_CAL_CM_OF_INDEX(0) + (pixel - (float)k230_cal_px[0]) * slope;
    }
    if (pixel >= (float)k230_cal_px[K230_CAL_POINT_NUM - 1]) {
        float slope = (float)K230_CAL_STEP_CM
                    / ((float)k230_cal_px[K230_CAL_POINT_NUM - 1]
                       - (float)k230_cal_px[K230_CAL_POINT_NUM - 2]);
        return (float)K230_CAL_CM_OF_INDEX(K230_CAL_POINT_NUM - 1)
             + (pixel - (float)k230_cal_px[K230_CAL_POINT_NUM - 1]) * slope;
    }

    /* 表内：找到 pixel 落在哪两个标定点之间，再线性插值 */
    for (uint8_t i = 0u; i < (K230_CAL_POINT_NUM - 1); i++) {
        float px_low  = (float)k230_cal_px[i];
        float px_high = (float)k230_cal_px[i + 1];

        if ((pixel >= px_low) && (pixel < px_high)) {
            float offset = (pixel - px_low) / (px_high - px_low);
            return (float)K230_CAL_CM_OF_INDEX(i) + offset * (float)K230_CAL_STEP_CM;
        }
    }

    /* 表已保证单调递增，正常走不到这里；返回 0 兜底，避免返回未定义值 */
    return 0.0f;
}

/**
 * @brief 判断像素坐标是否为有效的钢球位置。
 *
 * @param current_x_px 钢球像素横坐标。
 * @return 1 = 有效，0 = 无效（钢球丢失或误检）。
 */
static uint8_t K230ToStm32IsPixelValid(int16_t current_x_px)
{
    /* 协议规定 current_x == -1 表示钢球丢失。任何负值都按「未检测到」
       处理更安全：即便上位机将来用别的负值表示其他异常，也不会被
       当成有效位置送进控制环 */
    if (current_x_px < K230_PIXEL_VALID_MIN) {
        return 0u;
    }

    /* 超出画面范围的像素是误检，直接丢弃而不是限幅 ——
       限幅会把误检变成一个「看起来合理」的位置，控制环会去追它 */
    if (current_x_px > K230_PIXEL_VALID_MAX) {
        return 0u;
    }

    return 1u;
}

/**
 * @brief 校验并处理一整帧。
 *
 * @note 在接收中断中执行，必须保持短小（开发规范 §8.1）。
 */
static void K230ToStm32HandleFrame(K230ToStm32Instance *instance)
{
    /* 校验：前 12 字节逐字节异或 */
    uint8_t checksum = 0u;
    for (uint8_t i = 0u; i < K230_FRAME_CHECKSUM_INDEX; i++) {
        checksum ^= instance->frame_buf[i];
    }

    if (checksum != instance->frame_buf[K230_FRAME_CHECKSUM_INDEX]) {
        /* 校验失败整帧丢弃。宁可少一帧数据，也不能把可疑数据交给控制环 */
        instance->checksum_error_count++;
        return;
    }

    int16_t current_x_px = K230ToStm32ReadInt16BE(&instance->frame_buf[K230_FRAME_CURRENT_X_INDEX]);
    int16_t target_x_px  = K230ToStm32ReadInt16BE(&instance->frame_buf[K230_FRAME_TARGET_X_INDEX]);

    instance->frame_count++;
    instance->data.current_x_px       = current_x_px;
    instance->data.target_x_px        = target_x_px;
    instance->data.target_pos_cm      = K230ToStm32PixelToCm(target_x_px);
    instance->data.last_frame_time_ms = HAL_GetTick();

    if (K230ToStm32IsPixelValid(current_x_px) != 0u) {
        instance->data.pos_cm        = K230ToStm32PixelToCm(current_x_px);
        instance->data.ball_detected = 1u;
        instance->data.pos_valid     = 1u;
    } else {
        /* 钢球丢失：必须清掉 pos_valid。保留 pos_cm 旧值只是为了联调时
           能看到球是在哪个位置丢的，上层一律以 pos_valid 为准 */
        instance->data.ball_detected = 0u;
        instance->data.pos_valid     = 0u;
        instance->lost_frame_count++;
    }
}

/**
 * @brief 逐字节喂入解析状态机。
 *
 * parse_index 表示「已经收了多少字节」：
 *   0 ~ 3  : 正在匹配 4 字节帧头。失配时回退重试当前字节，这样即使出现
 *            ... AA AA 55 01 08 ... 这种帧头被前缀污染的流，也不会漏掉
 *            后面那一帧的帧头。
 *   4 ~ 11 : 正在收集 8 字节载荷。
 *   12     : 收到校验字节，交给 K230ToStm32HandleFrame() 处理。
 *
 * @note 在接收中断中执行，必须保持短小（开发规范 §8.1）。
 */
static void K230ToStm32FeedByte(K230ToStm32Instance *instance, uint8_t byte)
{
    /* ---- 阶段一：匹配帧头 ---- */
    if (instance->parse_index < K230_FRAME_HEADER_LEN) {
        if (byte == k230_frame_header[instance->parse_index]) {
            instance->frame_buf[instance->parse_index] = byte;
            instance->parse_index++;
        } else {
            /* 失配，重新开始同步。当前字节本身可能是新帧头的第一个字节，
               所以要用 header[0] 再比对一次，不能直接丢弃 */
            instance->parse_index = 0u;
            if (byte == k230_frame_header[0]) {
                instance->frame_buf[0] = byte;
                instance->parse_index  = 1u;
            }
        }
        return;
    }

    /* ---- 阶段二：收集载荷与校验字节 ---- */
    instance->frame_buf[instance->parse_index] = byte;
    instance->parse_index++;

    if (instance->parse_index < K230_FRAME_LEN) {
        return;
    }

    /* 一帧收齐，先复位状态机再处理，保证下一字节重新从帧头开始匹配 */
    instance->parse_index = 0u;
    K230ToStm32HandleFrame(instance);
}

/**
 * @brief BSP 接收回调，**在 USART1 中断上下文中执行**。
 *
 * @param usart_instance 触发的 BSP 实例。
 * @param data           本次收到的数据块。
 * @param len            字节数。
 */
static void K230ToStm32OnRxData(K230UsartInstance *usart_instance, const uint8_t *data, uint16_t len)
{
    if ((usart_instance == NULL) || (data == NULL) || (len == 0u)) {
        return;
    }

    /* BSP 层通过 parent 指针回传上层实例，BSP 不需要包含本模块头文件
       （开发规范 §6.4） */
    K230ToStm32Instance *instance = (K230ToStm32Instance *)usart_instance->parent;
    if (instance == NULL) {
        return;
    }

    for (uint16_t i = 0u; i < len; i++) {
        K230ToStm32FeedByte(instance, data[i]);
    }
}

/* ============================ 外部接口 ============================ */

uint8_t k230_to_stm32_Init(UART_HandleTypeDef *uart_handle)
{
    K230ToStm32_Init_Config_s config = {
        .uart_handle = uart_handle,
    };

    /* 传入 NULL 时 K230ToStm32Register() 会直接返回 NULL，统一走下面的失败分支 */
    k230_to_stm32_instance = K230ToStm32Register(&config);
    if (k230_to_stm32_instance == NULL) {
        return 0u;
    }

    return K230ToStm32Start(k230_to_stm32_instance);
}

/**
 * @brief 获取 k230_to_stm32_Init() 创建的默认 K230 实例。
 *
 * @return 默认实例指针；尚未初始化或初始化失败时返回 NULL。
 */
K230ToStm32Instance *K230ToStm32GetInstance(void)
{
    return k230_to_stm32_instance;
}

K230ToStm32Instance *K230ToStm32Register(const K230ToStm32_Init_Config_s *config)
{
    if ((config == NULL) || (config->uart_handle == NULL)) {
        return NULL;
    }

    K230ToStm32Instance *instance = NULL;
    for (uint8_t i = 0u; i < K230_TO_STM32_MAX_INSTANCE; i++) {
        if (k230_to_stm32_pool[i].usart_instance == NULL) {
            instance = &k230_to_stm32_pool[i];
            break;
        }
    }
    if (instance == NULL) {
        return NULL; /* 实例池已满 */
    }

    memset(instance, 0, sizeof(K230ToStm32Instance));

    /* 把本模块挂到 BSP 层：数据到达后由 BSP 回调 K230ToStm32OnRxData */
    K230Usart_Init_Config_s usart_config = {
        .uart_handle     = config->uart_handle,
        .rx_dma_buf      = instance->rx_dma_buf,
        .rx_dma_buf_size = K230_RX_DMA_BUF_SIZE,
        .parent          = instance,
        .rx_callback     = K230ToStm32OnRxData,
    };

    instance->usart_instance = K230UsartRegister(&usart_config);
    if (instance->usart_instance == NULL) {
        return NULL;
    }

    return instance;
}

uint8_t K230ToStm32Start(K230ToStm32Instance *instance)
{
    if (instance == NULL) {
        return 0u;
    }

    return (K230UsartStart(instance->usart_instance) == HAL_OK) ? 1u : 0u;
}

/**
 * @brief 按给定的帧计数与时间戳判断链路是否在线。
 *
 * @note 抽成独立函数是为了让「读取快照」和「实时查询」两条路径共用同一套
 *       超时判据，避免两处阈值逻辑各写一遍后走偏。
 */
static uint8_t K230ToStm32IsOnlineAt(uint32_t frame_count, uint32_t last_frame_time_ms)
{
    /* 开机后一帧都没收到过，不能算在线 */
    if (frame_count == 0u) {
        return 0u;
    }

    /* 无符号相减，HAL_GetTick() 回绕（约 49.7 天）时结果依然正确 */
    uint32_t elapsed_ms = HAL_GetTick() - last_frame_time_ms;

    return (elapsed_ms <= K230_VISION_TIMEOUT_MS) ? 1u : 0u;
}

uint8_t K230ToStm32GetData(K230ToStm32Instance *instance, K230_Data_s *data_out)
{
    if ((instance == NULL) || (data_out == NULL)) {
        return 0u;
    }

    /* 单写(中断)多读(主循环)：读取期间关中断，避免拷到半新半旧的快照。
       临界区内只有一次结构体赋值，无延时（开发规范 §8.2） */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *data_out = instance->data;
    __set_PRIMASK(primask);

    /* 用快照自己的时间戳判在线，保证返回值与快照出自同一时刻 */
    if (K230ToStm32IsOnlineAt(instance->frame_count, data_out->last_frame_time_ms) == 0u) {
        /* 链路超时后位置不再可信：即使最后一帧仍检测到钢球，也必须把
           pos_valid 清掉，否则控制环会拿着过期位置闭环（规范 §12.1） */
        data_out->pos_valid     = 0u;
        data_out->ball_detected = 0u;
    }

    return data_out->pos_valid;
}

uint8_t K230ToStm32GetPosCm(K230ToStm32Instance *instance, float *pos_cm_out)
{
    if ((instance == NULL) || (pos_cm_out == NULL)) {
        return 0u;
    }

    K230_Data_s data;
    if (K230ToStm32GetData(instance, &data) == 0u) {
        /* 钢球丢失或视觉离线：不写出任何值，上层拿到 0 就知道不能用 */
        return 0u;
    }

    *pos_cm_out = data.pos_cm;
    return 1u;
}

uint8_t K230ToStm32IsOnline(K230ToStm32Instance *instance)
{
    if (instance == NULL) {
        return 0u;
    }

    return K230ToStm32IsOnlineAt(instance->frame_count, instance->data.last_frame_time_ms);
}

void K230ToStm32GetStats(K230ToStm32Instance *instance, K230ToStm32_Stats_s *stats_out)
{
    if ((instance == NULL) || (stats_out == NULL)) {
        return;
    }

    stats_out->frame_count          = instance->frame_count;
    stats_out->checksum_error_count = instance->checksum_error_count;
    stats_out->lost_frame_count     = instance->lost_frame_count;
}
