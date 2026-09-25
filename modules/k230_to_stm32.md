# k230_to_stm32 —— K230 视觉接收模块

2026 电赛 H 题（钢珠运动控制）视觉接收端。负责接收 K230 通过 USART1 发来的
二进制帧，校验后换算出钢珠在水管中的物理位置（cm），供位置闭环使用。

---

## 1. 功能说明

- 接收：USART1 + DMA1_Channel5「空闲中断不定长接收」，一次事件收到多少字节就处理多少字节
- 解析：逐字节状态机切分 13 字节定长帧，抗错位、能自动重新同步
- 校验：逐字节异或，校验失败整帧丢弃，不把可疑数据交给控制环
- 换算：像素坐标 → 水管物理位置（cm），线性插值
- 安全：钢球丢失（`current_x == -1`）或视觉超时后，位置一律判为无效，禁止沿用旧值闭环

---

## 2. 文件结构

```
bsp/usart/usart1_k230.h           BSP：串口/DMA 接收驱动接口
bsp/usart/usart1_k230.c           BSP：串口/DMA 接收驱动实现（唯一接触 HAL_UART_xxx 的业务文件）
modules/k230_to_stm32.h           协议常量、标定参数、对外接口
modules/k230_to_stm32.c           状态机解析、校验、物理位置换算
modules/k230_to_stm32.md          本文档
```

依赖方向：`Application → modules/k230_to_stm32 → bsp/usart/usart1_k230 → HAL`

---

## 3. 通信协议（13 字节，大端序）

| 字节 | 字段 | 类型 | 说明 |
|---|---|---|---|
| 0-3 | 帧头 | — | 固定 `0xAA 0x55 0x01 0x08` |
| 4-5 | `current_x` | int16_t | 钢球当前像素 X 坐标，**-1 表示钢球丢失** |
| 6-7 | `target_x` | int16_t | 目标像素 X 坐标，协议固定 335 |
| 8-9 | `reserved_1` | uint16_t | 保留，固定 0（当前版本忽略不校验） |
| 10-11 | `reserved_2` | uint16_t | 保留，固定 0（当前版本忽略不校验） |
| 12 | `checksum` | uint8_t | 前 12 字节逐字节异或 |

发送频率约 20 Hz。

---

## 4. 物理位置换算

以 0 cm（335 px）为分界，**正负两侧各用一条直线**，因为实测发现两侧比例不同。

| 半轴 | 标定 | 比例 |
|---|---|---|
| 右半轴 | 335 px ↔ 0 cm，**613.3 px** ↔ +10 cm | 10 / 278.3 = **0.03593 cm/px** |
| 左半轴 | 0 px ↔ -11 cm，335 px ↔ 0 cm | 11 / 335 = **0.03284 cm/px** |

```
px > 335 :  pos_cm = (current_x - 335) × K230_CM_PER_PIXEL_RIGHT
px < 335 :  pos_cm = (current_x - 335) × K230_CM_PER_PIXEL_LEFT
```

### 右半轴的 613.3 px 是怎么来的

2026-09-25 实测（把钢珠放在水管的 cm 刻度上，读 OLED 上的 pos）：

| 实际 (cm) | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
|---|---|---|---|---|---|---|---|---|---|---|
| 显示 (cm) | 0.86 | 1.67 | 2.68 | 3.67 | 4.62 | 5.50 | 6.32 | 7.31 | 8.30 | 9.08 |
| 反推 px | 361 | 386 | 417 | 447 | 476 | 503 | 528 | 558 | 588 | 612 |

10 cm 处对应 px≈612，**不是**画面右端 640 px。线性拟合（零点固定 335）得
27.83 px/cm，而旧常数 `305/10 = 30.5 px/cm` **偏大约 9%**。

误差在 0 cm 处为 0、随后线性增大到 10 cm 处的 0.92 cm —— 这正是**增益误差**的
特征（误差 = 距离 × 9%），而不是零点偏移或非线性。

### 注意事项

- **两侧斜率不同，0 cm 处有约 9% 的增益跳变。** 如果后续实测发现左半轴也有
  同样约 9% 的比例误差（即相机视场在水管两端各多出约 1 cm），那说明两侧其实
  同比例，应改成单段全局比例（把 `K230_CM_PER_PIXEL_LEFT` 换成
  `K230_CM_PER_PIXEL_RIGHT`），届时 0 px 会对应约 -11.8 cm。
- 613 px 以上已在水管右端之外，会外推出大于 +10 cm 的值（钢珠不可能在那里，
  出现即为误检）。

标定参数集中在 `modules/k230_to_stm32.h`，改动时只改那一处：

```c
#define K230_PIXEL_ZERO_PX       335.0f   /* 水管 0 cm */
#define K230_PIXEL_LEFT_PX         0.0f   /* ↔ -11 cm */
#define K230_PIXEL_RIGHT_PX      613.3f   /* ↔ +10 cm，实测值 */
```

---

## 5. 外部接口

### 初始化和生命周期

```c
K230ToStm32Instance *K230ToStm32Register(const K230ToStm32_Init_Config_s *config);
uint8_t              K230ToStm32Start(K230ToStm32Instance *instance);   // 1=成功
```

### 取数据（给控制环用）

```c
/* 最简：直接取物理位置。返回 0 时不会写出任何值 */
uint8_t K230ToStm32GetPosCm(K230ToStm32Instance *instance, float *pos_cm_out);

/* 完整快照，便于调试 */
uint8_t K230ToStm32GetData(K230ToStm32Instance *instance, K230_Data_s *data_out);

/* 链路是否在线（超过 K230_VISION_TIMEOUT_MS = 200ms 没有合法帧即离线） */
uint8_t K230ToStm32IsOnline(K230ToStm32Instance *instance);

/* 统计：帧数 / 校验失败数 / 丢球帧数 */
void    K230ToStm32GetStats(K230ToStm32Instance *instance, K230ToStm32_Stats_s *stats_out);
```

`K230_Data_s` 字段：

| 字段 | 单位 | 说明 |
|---|---|---|
| `current_x_px` | px | 钢球像素 X 坐标，丢失时为负 |
| `target_x_px` | px | 目标像素 X 坐标 |
| `pos_cm` | cm | 钢球物理位置，`pos_valid` 为 0 时无意义 |
| `target_pos_cm` | cm | 目标物理位置，按同一标定换算，通常为 0 |
| `ball_detected` | — | 1 = 本帧检测到钢球 |
| `pos_valid` | — | 1 = `pos_cm` 可信且未超时，可用于闭环 |
| `last_frame_time_ms` | ms | 最近一帧合法数据的 `HAL_GetTick()` |

---

## 6. 初始化示例

```c
/* 必须在 MX_USART1_UART_Init() 之后（注册时要读已关联的 DMA 句柄） */
K230ToStm32_Init_Config_s k230_config = {
    .uart_handle = &huart1,
};
k230_vision = K230ToStm32Register(&k230_config);
if (k230_vision == NULL) {
    /* 串口 RX DMA 未配置，或实例池已满 */
} else if (K230ToStm32Start(k230_vision) == 0u) {
    /* 串口状态异常，启动失败 */
}
```

`main.c` 中已按上述方式接线（`USER CODE BEGIN 2`）。

---

## 7. 典型使用流程

```c
/* 控制任务内，按控制周期调用 */
float ball_pos_cm = 0.0f;

if (K230ToStm32GetPosCm(k230_vision, &ball_pos_cm) != 0u) {
    /* 位置有效：ball_pos_cm 单位 cm，水管中点为 0，右侧为正 */
    /* pid_ref = 0.0f; feedback = ball_pos_cm; 走正常闭环 */
} else {
    /* 钢球丢失或视觉离线：禁止使用上一次的位置！
       按安全策略处理（输出归零 / 保持 / 进入搜索状态） */
}
```

**关键约定：`GetPosCm()` 返回 0 时不会写出任何值**，调用方必须检查返回值。
这样即使忘记判断，也拿不到过期数据去闭环。

---

## 8. 注意事项

1. **接收回调在中断上下文执行**。解析和换算都在 USART1 中断里完成，
   耗时在微秒级（20 Hz，无压力）。不要在回调里加延时或打印。

2. **数据快照的并发保护**。`K230_Data_s` 由接收中断写、主循环读，属单写多读。
   读取统一走 `K230ToStm32GetData()`，它内部用临界区拷贝（只关中断约十几条指令）。
   不要绕过接口直接访问实例字段。

3. **Normal 模式 DMA 必须重挂**。CubeMX 里 USART1_RX 的 DMA 是 Normal 模式，
   一次空闲事件后 DMA 就停了。驱动在 `HAL_UARTEx_RxEventCallback()` 里自动重挂，
   上层不需要管。若把 CubeMX 的 DMA 改成 **Circular 模式**，重挂逻辑会自动跳过，
   可靠性更高（少一处需要恢复的状态），有时间建议改。

4. **串口错误会自动恢复**。过载/帧/噪声错误会中断本轮接收，驱动在
   `HAL_UART_ErrorCallback()` 里清标志并重挂，一次线路干扰不会让数据永久停更。

5. **CubeMX 重新生成代码不受影响**。所有逻辑都在 `bsp/` 和 `modules/` 下，
   `Core/Src/stm32f1xx_it.c` 一行都没改（IDLE 中断由 HAL 的
   `HAL_UARTEx_RxEventCallback` 通路处理）。但 **`Makefile` 里手工加的三项
   （`bsp/usart/usart1_k230.c`、`modules/k230_to_stm32.c`、`-Ibsp/usart`）
   在 CubeMX 重新生成后需要检查是否还在**。

6. **编译要求 0 warning**。`Makefile` 已开启 `-Wall -Wextra -Werror`。
   唯一的定点豁免是第三方字库 `modules/oled_data.c`（`-Wno-missing-braces`）。

---

## 9. 常见错误

联调时先看 OLED 上显示的四行信息（`main.c` 的调试显示），或读 `GetStats()`：

| 现象 | 可能原因 | 排查 |
|---|---|---|
| `px` 一直是 0 / 不动 | 没收到任何数据 | 查接线（K230 TX → PA10）、波特率 115200 8N1、K230 是否在发；读 BSP 统计 `rx_byte_count` 是否增长 |
| `er` 持续增长，`px` 偶发更新 | 波特率不对或协议不匹配 | 校验一直在失败；确认 K230 端帧格式是 13 字节 `AA 55 01 08` 开头 |
| `rx_byte_count` 增长但 `fr` 不增长 | 收到数据但不是这个协议 | K230 发的是别的帧格式（例如旧的多目标协议），需对齐协议 |
| `px` 有值但第 1 行是 `lost` | 钢球丢失（`current_x == -1`） | K230 侧目标检测问题：光照、阈值、钢珠反光；正常现象时属预期行为 |
| `link:offline` | 超过 200ms 没收到合法帧 | 链路断了或 K230 停了；阈值见 `K230_VISION_TIMEOUT_MS` |
| `pos` 数值明显偏 | 标定参数不对 | 用 `px` 手算验证：核对 `K230_PIXEL_ZERO_PX` / `K230_PIXEL_RIGHT_PX`；误差随距离线性增大 = 比例错，各处等量偏移 = 零点错 |
| 开机不久后数据停更 | DMA 重挂失败 | 读 BSP `error_count`；确认未被其他代码占用 USART1 或 DMA1_Channel5 |

---

## 10. 单元测试

用 Python 逐行转写了本模块的状态机与换算公式，覆盖：换算端点、正常帧、
丢失帧、坏校验、帧头重叠重同步、坏帧后恢复、多帧/半帧/逐字节喂入、
越界像素、载荷含帧头字节、边界像素。共 38 项检查，全部通过。

换算端点实测：`335px → 0.0000cm`，`640px → 10.0000cm`，`0px → -10.9836cm`。

上板后按下列顺序验证（对应开发规范 §16.2 的模块级测试）：

```
1. 只上串口，K230 正常发球位置，OLED 看 px 是否跟随钢珠
2. 移动钢珠到水管中点，确认 pos ≈ 0.00cm
3. 移到右侧 10cm 刻度，确认 pos ≈ 10.00cm
4. 用手挡住钢珠，确认显示 lost 而不是停在最后一个位置
5. 拔掉 K230 信号线，确认 200ms 内变成 offline
```
