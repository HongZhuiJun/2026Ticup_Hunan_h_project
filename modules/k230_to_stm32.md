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
| 6-7 | `target_x` | int16_t | 目标像素 X 坐标，当前固定 318 |
| 8-9 | `reserved_1` | uint16_t | 保留，固定 0（当前版本忽略不校验） |
| 10-11 | `reserved_2` | uint16_t | 保留，固定 0（当前版本忽略不校验） |
| 12 | `checksum` | uint8_t | 前 12 字节逐字节异或 |

发送频率约 20 Hz。

---

## 4. 物理位置换算

**查标定表 + 相邻两点线性插值**，不用直线拟合。

### 为什么必须查表

镜头是广角，桶形畸变下像素比例沿画面变化很大。2026-09-28 实测（把钢珠逐个
放在水管 1 cm 刻度上，记录 K230 报出的像素 X）得到：

| 位置 | 像素比例 |
|---|---|
| 中心（px 284 ~ 485） | 33 ~ 34 px/cm（放大率最大） |
| 两端（px 6 / 623） | 21 ~ 25 px/cm（被压缩） |

**中心比边缘大 62%**。无论用一条直线还是分两段的两条直线，总有一头对不准——
用旧的两段直线标定，实测最大误差 **0.69 cm**；换成 22 点查表后归零。

### 标定数据（2026-09-28 实测）

| cm | -11 | -10 | -9 | -8 | -7 | -6 | -5 | -4 | -3 | -2 | -1 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| px | 6 | 27 | 50 | 74 | 100 | 128 | 156 | 187 | 219 | 251 | 284 |

| cm | 0 | +1 | +2 | +3 | +4 | +5 | +6 | +7 | +8 | +9 | +10 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| px | 318 | 352 | 384 | 418 | 451 | 485 | 514 | 543 | 571 | 598 | 623 |

注意 0 cm 落在 **318 px**（即 `target_x`），水管两端分别在 px 6 和 px 623，
**并不是画面边缘的 0 / 640**。

### 换算方法

```
在表内：找到 px 落在哪两个标定点之间，按位置比例插值出 cm
在表外：按最近一段的比例外推（标定覆盖 6 ~ 623 px，越界一般是误检）
```

### 标定表在哪里

**只在 `modules/k230_to_stm32.c` 的 `k230_cal_px[]`**，这是唯一要改的地方：

```c
static const int16_t k230_cal_px[K230_CAL_POINT_NUM] = {
    /* -11  -10   -9   -8   -7   -6   -5   -4   -3   -2  cm */
        6,  27,  50,  74, 100, 128, 156, 187, 219, 251,
    /*  -1    0   +1   +2   +3   +4   +5   +6   +7   +8  cm */
      284, 318, 352, 384, 418, 451, 485, 514, 543, 571,
    /*  +9  +10  cm */
      598, 623,
};
```

### 注意事项

- **必须随 cm 单调递增。** 转换函数依赖这一点做区间查找，否则插值结果会回跳。
  改完肉眼确认一遍相邻值在变大。
- **两处数据必须一致**：这里和 K230 侧 `firmware/main_zdyz_k230.py` 的
  `CAL_X_*` 常量。换镜头或挪动相机后重新标定，两个文件一起改。
- 重新标定：K230 侧把 `SHOW_CALIBRATION_LINES` 改回 `True`，LCD 会按当前值
  画出 22 条黄线，逐条对准水管刻度，再把实测值同步到两边。

---

## 5. 外部接口

### 初始化和生命周期

```c
/* 一步初始化：注册 + 启动接收。main.c 里只用这一个 */
uint8_t              k230_to_stm32_Init(UART_HandleTypeDef *uart_handle);  // 1=已在后台接收

/* 分成两步，需要区分「注册失败」还是「启动失败」时用 */
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

`main.c` 里就一行（`USER CODE BEGIN 2`）：

```c
/* 必须在 MX_USART1_UART_Init() 之后 —— 注册时要读 CubeMX 已关联的 RX DMA 句柄 */
(void)k230_to_stm32_Init(&huart1);
```

`k230_to_stm32_Init()` 就是下面这几步，只是没有对外输出，失败原因只体现在返回值上：

```c
/* 展开后就是这样，只是没有对外输出，失败原因只体现在返回值上 */
K230ToStm32_Init_Config_s config = { .uart_handle = &huart1 };
k230_to_stm32_instance = K230ToStm32Register(&config);   /* NULL = DMA 未配 / 实例池满 */
if (k230_to_stm32_instance != NULL) {
    K230ToStm32Start(k230_to_stm32_instance);            /* 0 = 串口状态异常 */
}
```

启动之后帧解析全在 USART1 中断里完成，**主循环不需要轮询、也不需要做任何事**，
`main.c` 的 `while(1)` 是空的。

注册出来的实例指针由模块自己保存（`k230_to_stm32.c` 的静态 `k230_to_stm32_instance`），
不再由 `main.c` 持有。

> 待办：外部还没有取这个实例的接口。等接控制环时，要么补一个单例版取数接口
> （`uint8_t k230_to_stm32_GetPosCm(float *pos_cm_out);`），要么让
> `k230_to_stm32_Init()` 把实例指针返回给调用方。

---

## 7. 典型使用流程

下面的 `instance` 指 `K230ToStm32Register()` 返回的实例。用
`k230_to_stm32_Init()` 初始化时，这个指针在模块内部（见 §6 的待办）。

```c
/* 控制任务内，按控制周期调用 */
float ball_pos_cm = 0.0f;

if (K230ToStm32GetPosCm(instance, &ball_pos_cm) != 0u) {
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

`main.c` 里原有的 OLED 调试显示已随初始化收拢一起删除，现在只能靠
`K230ToStm32GetData()` / `K230ToStm32GetStats()` 读数据判断。
需要肉眼盯着看时，在 `main.c` 主循环里加一段临时显示即可——
删除前那版在 git 里还有一个（`git show HEAD:Core/Src/main.c`），
四行分别是：`pos`（cm，丢失时 `lost`）、`px`（原始像素，含 `t:` 目标）、
`link`（在线状态）、`fr`/`er`（合法帧数 / 校验失败数）。

下表按「看哪一指标」重写，字段名对应 `K230_Data_s` 和 `K230ToStm32_Stats_s`：

| 现象 | 可能原因 | 排查 |
|---|---|---|
| `current_x_px` 一直是 0 / 不动 | 没收到任何数据 | 查接线（K230 TX → PA10）、波特率 115200 8N1、K230 是否在发；读 BSP 统计 `rx_byte_count` 是否增长 |
| `checksum_error_count` 持续增长，`current_x_px` 偶发更新 | 波特率不对或协议不匹配 | 校验一直在失败；确认 K230 端帧格式是 13 字节 `AA 55 01 08` 开头 |
| `rx_byte_count` 增长但 `frame_count` 不增长 | 收到数据但不是这个协议 | K230 发的是别的帧格式（例如旧的多目标协议），需对齐协议 |
| `current_x_px` 有值但 `pos_valid` 为 0 | 钢球丢失（`current_x == -1`）或视觉超时 | 看 `ball_detected`：为 0 说明是 K230 侧目标检测问题（光照、阈值、钢珠反光），属预期行为；为 1 则是超时，见下一行 |
| `K230ToStm32IsOnline()` 返回 0 | 超过 200ms 没收到合法帧 | 链路断了或 K230 停了；阈值见 `K230_VISION_TIMEOUT_MS` |
| `pos_cm` 数值明显偏 | 标定表不对 | 用 `current_x_px` 手算验证：核对 `modules/k230_to_stm32.c` 的 `k230_cal_px[]`，以及 K230 侧 `CAL_X_*` 是否与它一致。某一段偏 = 表里对应几项量错；整体偏移 = 零点量错 |
| 开机不久后数据停更 | DMA 重挂失败 | 读 BSP `error_count`；确认未被其他代码占用 USART1 或 DMA1_Channel5 |

---

## 10. 单元测试

用 Python 逐行转写了本模块的状态机与换算公式，覆盖：标定点回代、中点插值、
表外外推、单调性、正常帧、丢失帧、坏校验、帧头重叠重同步、坏帧后恢复、
多帧/半帧/逐字节喂入、越界像素、载荷含帧头字节、边界像素。

换算部分实测：22 个标定点回代误差 0.000000 cm，21 个中点插值全部正确，
0~640 px 全程严格单调；旧的两段直线标定最大误差 0.69 cm，换查表后归零。

上板后按下列顺序验证（对应开发规范 §16.2 的模块级测试）。
观察手段：临时 OLED 显示（见 §9）、调试器 watch，或把值塞进 `GetData()` 的快照：

```
1. 只上串口，K230 正常发球位置：current_x_px 应跟随钢珠移动
2. 移动钢珠到水管中点：pos_cm ≈ 0.00cm
3. 移到右侧 10cm 刻度：pos_cm ≈ 10.00cm
4. 用手挡住钢珠：pos_valid 变 0、ball_detected 变 0，而不是停在最后一个位置
5. 拔掉 K230 信号线：K230ToStm32IsOnline() 在 200ms 内变为 0
```
