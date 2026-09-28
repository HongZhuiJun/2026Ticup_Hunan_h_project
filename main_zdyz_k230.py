"""ATK K230D steel-ball detector with LCD output and UART2 feedback.

The board detects the highest-confidence steel ball inside the configured
region, displays the result on a 640x480 ST7701 LCD, and sends its center X
coordinate to an STM32 over UART2. Wi-Fi, RTSP, touch, and task-selection
code are not used.
"""

from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.Utils import ScopedTiming

from media.display import Display
from media.media import MediaManager
from media.sensor import Sensor, CAM_CHN_ID_0, CAM_CHN_ID_2
from machine import UART, FPIOA

import aidemo
import gc
import image
import nncase_runtime as nn
import os
import struct
import time
import ulab.numpy as np


# ============================================================
# User settings
# ============================================================

# After converting the trained model, copy it to this path.
KMODEL_PATH = "/sdcard/steel_ball_yolov8n_320_v2_best.kmodel"

LABELS = ["ball"]
MODEL_INPUT_SIZE = [320, 320]
AI_WIDTH = 320
AI_HEIGHT = 320

# Detection output coordinate space used by the KModel postprocessor.
OUTPUT_WIDTH = 640
OUTPUT_HEIGHT = 480

# ATK ST7701 LCD. The official LCD example uses 640x480 on this board.
LCD_WIDTH = 640
LCD_HEIGHT = 480
LCD_DRAW_EVERY_N_FRAMES = 2

# Detect across the full width and only inside the configured vertical strip
# in the 640x480 detector output.
BALL_X_MIN = 0
BALL_X_MAX = 640
BALL_Y_MIN = 215
BALL_Y_MAX = 266

CONFIDENCE_THRESHOLD = 0.15
NMS_THRESHOLD = 0.45
MAX_BOXES_NUM = 5

# Horizontal target in the 640-pixel detector coordinate space.
TARGET_X = 318

# ---------------------------------------------------------------------------
# 整数 cm 标定值（2026-09-28 实测，已冻结）
#
# 每个常数 = 钢珠正好停在该 cm 刻度上时，K230 报出的 detector 像素 X。
#
# 这份数据已同步到 STM32 侧的 modules/k230_to_stm32.c 里的 k230_cal_px[] 表，
# 两边必须一致，改动时一起改。
#
# 实测畸变特征（广角镜头桶形畸变），相邻整数 cm 的像素间距：
#     中心 (px 284~485)  33~34 px/cm   <- 放大率最大
#     两端 (px 6 / 623)  21~25 px/cm   <- 被压缩
#   中心比边缘大 62%，所以 px 与 cm 严重非线性，用一条直线（甚至两段直线）
#   拟合必然有一头对不准，只能查表插值。
#
#   22 个点全部单调递增（-11 最小、+10 最大），这是 STM32 侧插值的前提。
#   实测端点：-11 cm @ px 6，0 cm @ px 318（= TARGET_X），+10 cm @ px 623
#
# 重新标定（只有换镜头或挪动相机后才需要）：
#   1. 把下面 SHOW_CALIBRATION_LINES 改回 True，LCD 会按这些值画出黄线
#   2. 把钢珠放到各 cm 刻度上，把没对准的常数改成钢珠此刻的 X
#   3. 改完必须保持单调递增，然后把新数据同步到 STM32 的 k230_cal_px[]
# ---------------------------------------------------------------------------
CAL_X_NEG_11CM =   6   # -11 cm
CAL_X_NEG_10CM =  27   # -10 cm
CAL_X_NEG_9CM  =  50   #  -9 cm
CAL_X_NEG_8CM  =  74   #  -8 cm
CAL_X_NEG_7CM  = 100   #  -7 cm
CAL_X_NEG_6CM  = 128   #  -6 cm
CAL_X_NEG_5CM  = 156   #  -5 cm
CAL_X_NEG_4CM  = 187   #  -4 cm
CAL_X_NEG_3CM  = 219   #  -3 cm
CAL_X_NEG_2CM  = 251   #  -2 cm
CAL_X_NEG_1CM  = 284   #  -1 cm
CAL_X_0CM      = TARGET_X
CAL_X_1CM      = 352
CAL_X_2CM      = 384
CAL_X_3CM      = 418
CAL_X_4CM      = 451
CAL_X_5CM      = 485
CAL_X_6CM      = 514
CAL_X_7CM      = 543
CAL_X_8CM      = 571
CAL_X_9CM      = 598
CAL_X_10CM     = 623

# 绘图参数
# 标定已完成，黄线关闭，LCD 上只保留 0 cm 的 target 线（控制目标，更粗）。
# 需要复标时改回 True，会用上面 CAL_X_* 的值重新画出 22 条黄线。
SHOW_CALIBRATION_LINES = False
# 本板 LCD 上渲染为黄色的颜色元组，沿用现有 target 线用的值
CAL_LINE_COLOR = (255, 255, 255, 0)
CAL_LABEL_Y = 74
CAL_LABEL_SIZE = 16

# 由上面的常数生成绘图用的 (cm, px) 表，顺序自 -11 到 +10
CALIBRATION_LINES = [
    (-11, CAL_X_NEG_11CM),
    (-10, CAL_X_NEG_10CM),
    ( -9, CAL_X_NEG_9CM),
    ( -8, CAL_X_NEG_8CM),
    ( -7, CAL_X_NEG_7CM),
    ( -6, CAL_X_NEG_6CM),
    ( -5, CAL_X_NEG_5CM),
    ( -4, CAL_X_NEG_4CM),
    ( -3, CAL_X_NEG_3CM),
    ( -2, CAL_X_NEG_2CM),
    ( -1, CAL_X_NEG_1CM),
    (  0, CAL_X_0CM),
    (  1, CAL_X_1CM),
    (  2, CAL_X_2CM),
    (  3, CAL_X_3CM),
    (  4, CAL_X_4CM),
    (  5, CAL_X_5CM),
    (  6, CAL_X_6CM),
    (  7, CAL_X_7CM),
    (  8, CAL_X_8CM),
    (  9, CAL_X_9CM),
    ( 10, CAL_X_10CM),
]

# ATK K230D UART2 pin assignment from the official UART example:
# GPIO44 = UART2_TXD
# GPIO45 = UART2_RXD
UART_TX_GPIO = 44
UART_RX_GPIO = 45
UART_BAUDRATE = 115200

# Send one frame after every inference.
SEND_EVERY_N_FRAMES = 1
PRINT_EVERY_N_FRAMES = 30
GC_EVERY_N_FRAMES = 30

# Use the board's default camera interface. Connect only the desired GC2093
# camera; installing two GC2093 cameras at the same time prevents the board
# from detecting either sensor.
SENSOR_ID = None
SENSOR_WIDTH = 1280
SENSOR_HEIGHT = 960
SENSOR_FPS = 30

# If no valid ball is detected, send current_x=-1. This tells the STM32
# not to use a stale position for closed-loop control.
LOST_BALL_X = -1


def align_up(value, alignment):
    return (value + alignment - 1) // alignment * alignment


def create_sensor():
    if SENSOR_ID is None:
        sensor = Sensor(
            width=SENSOR_WIDTH,
            height=SENSOR_HEIGHT,
            fps=SENSOR_FPS,
        )
    else:
        sensor = Sensor(
            id=SENSOR_ID,
            width=SENSOR_WIDTH,
            height=SENSOR_HEIGHT,
            fps=SENSOR_FPS,
        )

    sensor.reset()

    # Channel 0 is the hardware video layer shown directly on the LCD.
    sensor.set_framesize(
        width=LCD_WIDTH,
        height=LCD_HEIGHT,
        chn=CAM_CHN_ID_0,
    )
    sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_0)

    # Channel 2 is the 320x320 RGB planar image used by the NPU.
    sensor.set_framesize(
        width=AI_WIDTH,
        height=AI_HEIGHT,
        alignment=12,
        chn=CAM_CHN_ID_2,
    )
    sensor.set_pixformat(Sensor.RGBP888, chn=CAM_CHN_ID_2)
    return sensor


def create_uart():
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_GPIO, FPIOA.UART2_TXD)
    fpioa.set_function(UART_RX_GPIO, FPIOA.UART2_RXD)

    uart = UART(
        UART.UART2,
        baudrate=UART_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )

    print(
        "UART2 ready: TX=GPIO%d RX=GPIO%d %d"
        % (UART_TX_GPIO, UART_RX_GPIO, UART_BAUDRATE)
    )
    return uart


def send_target_frame(uart, current_x, target_x, reserved_1=0, reserved_2=0):
    """Send the agreed 13-byte frame.

    Layout:
      AA 55 01 08 | current_x int16 | target_x int16 | u16 | u16 | checksum
    """
    current_x = max(-32768, min(int(current_x), 32767))
    target_x = max(-32768, min(int(target_x), 32767))
    reserved_1 = max(0, min(int(reserved_1), 65535))
    reserved_2 = max(0, min(int(reserved_2), 65535))

    payload = struct.pack(
        ">hhHH",
        current_x,
        target_x,
        reserved_1,
        reserved_2,
    )
    frame = bytes((0xAA, 0x55, 0x01, 0x08)) + payload

    checksum = 0
    for value in frame:
        checksum ^= value
    frame += bytes((checksum,))

    uart.write(frame)
    return frame


class SteelBallDetector(AIBase):
    def __init__(
        self,
        kmodel_path,
        model_input_size,
        rgb888p_size,
        output_size,
        debug_mode=0,
    ):
        super().__init__(
            kmodel_path,
            model_input_size,
            rgb888p_size,
            debug_mode,
        )

        self.model_input_size = model_input_size
        self.rgb888p_size = [
            align_up(rgb888p_size[0], 16),
            rgb888p_size[1],
        ]
        self.output_size = [
            align_up(output_size[0], 16),
            output_size[1],
        ]

        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8,
            np.uint8,
        )

    def config_preprocess(self):
        self.ai2d.build(
            [1, 3, self.rgb888p_size[1], self.rgb888p_size[0]],
            [1, 3, self.model_input_size[1], self.model_input_size[0]],
        )

    def preprocess(self, input_np):
        return [nn.from_numpy(input_np)]

    def postprocess(self, results):
        output = results[0][0].transpose()
        return aidemo.yolov8_det_postprocess(
            output.copy(),
            [self.rgb888p_size[1], self.rgb888p_size[0]],
            [self.model_input_size[1], self.model_input_size[0]],
            [self.output_size[1], self.output_size[0]],
            len(LABELS),
            CONFIDENCE_THRESHOLD,
            NMS_THRESHOLD,
            MAX_BOXES_NUM,
        )


def choose_best_detection(detections):
    """Return (center_x, center_y, width, height, score) or None."""
    if not detections:
        return None

    boxes = detections[0]
    scores = detections[2]

    if len(boxes) == 0:
        return None

    best_index = None
    best_score = -1.0

    for index in range(len(boxes)):
        x, y, width, height = map(
            lambda value: int(round(value, 0)),
            boxes[index],
        )
        center_x = x + width // 2
        center_y = y + height // 2

        if (
            center_x < BALL_X_MIN
            or center_x > BALL_X_MAX
            or center_y < BALL_Y_MIN
            or center_y > BALL_Y_MAX
        ):
            continue

        score = float(scores[index])
        if score > best_score:
            best_score = score
            best_index = index

    if best_index is None:
        return None

    x, y, width, height = map(
        lambda value: int(round(value, 0)),
        boxes[best_index],
    )
    center_x = x + width // 2
    center_y = y + height // 2
    return (center_x, center_y, width, height, best_score)


def draw_calibration_lines(osd_image, scale_x):
    """在每个整数 cm 的标定像素位置画一条黄色竖线。

    线旁的黄色数字是这条线当前代表的 cm 值，用来在 22 条线里认出目标那条。

    0 cm 由 target 线单独绘制（更粗、且是控制目标），这里跳过避免重复。
    """
    for cm, px in CALIBRATION_LINES:
        if cm == 0:
            continue

        line_x = int(px * scale_x)
        if line_x < 0 or line_x >= LCD_WIDTH:
            continue

        osd_image.draw_line(
            line_x,
            0,
            line_x,
            LCD_HEIGHT - 1,
            color=CAL_LINE_COLOR,
            thickness=1,
        )

        # 标签按估算字宽居中在线两侧，并夹在屏幕内避免边缘被裁掉
        label = "%d" % cm
        label_w = (len(label) * CAL_LABEL_SIZE) // 2
        label_x = max(0, min(line_x - label_w // 2, LCD_WIDTH - label_w))
        osd_image.draw_string_advanced(
            label_x,
            CAL_LABEL_Y,
            CAL_LABEL_SIZE,
            label,
            color=CAL_LINE_COLOR,
        )


def draw_lcd_result(osd_image, best, display_fps):
    """Draw the detection box, current X, ROI, and target X on the LCD."""
    osd_image.clear()

    scale_x = LCD_WIDTH / OUTPUT_WIDTH
    scale_y = LCD_HEIGHT / OUTPUT_HEIGHT

    roi_left = int(BALL_X_MIN * scale_x)
    roi_right = int(BALL_X_MAX * scale_x)
    roi_top = int(BALL_Y_MIN * scale_y)
    roi_bottom = int(BALL_Y_MAX * scale_y)

    osd_image.draw_rectangle(
        roi_left,
        roi_top,
        roi_right - roi_left,
        roi_bottom - roi_top,
        color=(255, 255, 255, 0),
        thickness=2,
    )

    target_px = int(TARGET_X * scale_x)
    osd_image.draw_line(
        target_px,
        0,
        target_px,
        LCD_HEIGHT - 1,
        color=(255, 255, 255, 0),
        thickness=2,
    )

    # 整数 cm 标定黄线。画在 target 线之后、文字和球框之前，
    # 这样 FPS/状态文字和钢珠标记始终压在线上面，不会被线遮住
    if SHOW_CALIBRATION_LINES:
        draw_calibration_lines(osd_image, scale_x)

    osd_image.draw_string_advanced(
        8,
        8,
        28,
        "FPS: %.1f" % display_fps,
        color=(255, 255, 255, 255),
    )

    if best is None:
        osd_image.draw_string_advanced(
            8,
            42,
            28,
            "BALL LOST  TARGET=%d" % TARGET_X,
            color=(255, 255, 0, 0),
        )
    else:
        center_x, center_y, width, height, score = best

        box_x = int((center_x - width / 2) * scale_x)
        box_y = int((center_y - height / 2) * scale_y)
        box_w = int(width * scale_x)
        box_h = int(height * scale_y)
        display_x = int(center_x * scale_x)
        display_y = int(center_y * scale_y)

        osd_image.draw_rectangle(
            box_x,
            box_y,
            box_w,
            box_h,
            color=(255, 0, 255, 0),
            thickness=3,
        )
        osd_image.draw_circle(
            display_x,
            display_y,
            5,
            color=(255, 255, 255, 0),
            thickness=2,
            fill=True,
        )
        osd_image.draw_string_advanced(
            8,
            42,
            28,
            "X=%d  TARGET=%d  SCORE=%.2f"
            % (center_x, TARGET_X, score),
            color=(255, 0, 255, 0),
        )

    Display.show_image(
        osd_image,
        0,
        0,
        Display.LAYER_OSD3,
    )


def main():
    uart = None
    sensor = None
    detector = None
    osd_image = None
    lcd_started = False
    media_initialized = False

    frame_count = 0
    fps_count = 0
    fps_start = time.ticks_ms()
    display_fps = 0.0

    try:
        os.exitpoint(os.EXITPOINT_ENABLE)

        uart = create_uart()
        sensor = create_sensor()
        detector = SteelBallDetector(
            KMODEL_PATH,
            MODEL_INPUT_SIZE,
            [AI_WIDTH, AI_HEIGHT],
            [OUTPUT_WIDTH, OUTPUT_HEIGHT],
            debug_mode=0,
        )
        detector.config_preprocess()

        try:
            lcd_source = sensor.bind_info(
                x=0,
                y=0,
                chn=CAM_CHN_ID_0,
            )
            # bind_layer must be called before Display.init.
            Display.bind_layer(
                **lcd_source,
                layer=Display.LAYER_VIDEO1,
            )
            Display.init(
                Display.ST7701,
                width=LCD_WIDTH,
                height=LCD_HEIGHT,
                osd_num=1,
                to_ide=False,
            )
            osd_image = image.Image(
                LCD_WIDTH,
                LCD_HEIGHT,
                image.ARGB8888,
            )
            lcd_started = True
        except BaseException as error:
            # Keep UART detection working if the LCD is not connected.
            print("LCD init failed; continuing without display:", error)

        MediaManager.init()
        media_initialized = True
        sensor.run()

        print("Steel-ball UART detector started")
        print("Model:", KMODEL_PATH)
        print(
            "Camera: %s"
            % (
                "default interface"
                if SENSOR_ID is None
                else "Sensor(id=%d)" % SENSOR_ID
            )
        )
        print(
            "LCD: %s"
            % (
                "%dx%d ST7701 enabled" % (LCD_WIDTH, LCD_HEIGHT)
                if lcd_started
                else "disabled"
            )
        )
        print(
            "Detection region: x=%d..%d y=%d..%d"
            % (BALL_X_MIN, BALL_X_MAX, BALL_Y_MIN, BALL_Y_MAX)
        )
        print("Target X:", TARGET_X)
        print("UART frame: current_x, target_x, 0, 0")
        print("Lost marker: current_x=%d" % LOST_BALL_X)

        while True:
            os.exitpoint()

            frame = sensor.snapshot(chn=CAM_CHN_ID_2)
            input_np = frame.to_numpy_ref()

            with ScopedTiming("total", 0):
                detections = detector.run(input_np)

            best = choose_best_detection(detections)
            frame_count += 1

            if (
                lcd_started
                and frame_count % LCD_DRAW_EVERY_N_FRAMES == 0
            ):
                draw_lcd_result(osd_image, best, display_fps)

            if frame_count % SEND_EVERY_N_FRAMES == 0:
                if best is None:
                    current_x = LOST_BALL_X
                else:
                    # Only center X is used for the one-dimensional control.
                    current_x = best[0]

                send_target_frame(
                    uart,
                    current_x,
                    TARGET_X,
                    0,
                    0,
                )

            if frame_count % PRINT_EVERY_N_FRAMES == 0:
                if best is None:
                    print("UART TX: current_x=%d target_x=%d"
                          % (LOST_BALL_X, TARGET_X))
                else:
                    print(
                        "UART TX: current_x=%d target_x=%d score=%.3f"
                        % (best[0], TARGET_X, best[4])
                    )

            del input_np
            del frame

            fps_count += 1
            now = time.ticks_ms()
            elapsed = time.ticks_diff(now, fps_start)
            if elapsed >= 1000:
                display_fps = fps_count * 1000.0 / elapsed
                print("AI FPS: %.2f" % display_fps)
                fps_count = 0
                fps_start = now

            if frame_count % GC_EVERY_N_FRAMES == 0:
                gc.collect()

    except KeyboardInterrupt:
        print("User stopped")
    except BaseException as error:
        import sys
        sys.print_exception(error)
    finally:
        if detector is not None:
            try:
                detector.deinit()
            except Exception:
                pass

        if sensor is not None:
            try:
                sensor.stop()
            except Exception:
                pass

        if lcd_started:
            try:
                Display.deinit()
            except Exception:
                pass

        if uart is not None:
            try:
                uart.deinit()
            except Exception:
                pass

        if media_initialized:
            try:
                MediaManager.deinit()
            except Exception:
                pass

        gc.collect()
        print("Resources released")


if __name__ == "__main__":
    main()
