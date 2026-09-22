/**
 * @file oled_menu.c
 * @brief OLED 三级菜单框架（模块层）。
 *
 * 菜单结构：
 *   一级菜单：基础部分 / 发挥部分，光标只在第 3、4 行之间移动
 *   二级菜单：基础部分 = 4 个任务 + "上一步"
 *             发挥部分 = 指定位置(确认即执行任务) + "上一步" + 位置 +1cm / -1cm
 *   三级菜单：任务执行页，循环显示小球位置，确认后返回二级菜单
 *
 * 按键（见 key.h）：KEY1 上移，KEY2 下移，KEY3 确认。
 *   光标在首项时按 KEY1 回绕到末项，在末项时按 KEY2 回绕到首项。
 *
 * 三级菜单里的任务执行函数还没有写，已经用 TODO 注释标出填入位置，
 * 该注释所在的 switch 分支在任务页里每轮循环执行一次。
 *
 * 注意：
 *   1. 一帧只在最后调用一次 OLED_Update()，刷新过慢会导致按键漏检。
 *   2. 任务页每轮循环都会清屏重绘，任务代码只要更新 location_cm 即可，
 *      不要在任务里直接调用 OLED 画图，否则会被清掉。
 */

#include "oled_menu.h"
#include "oled.h"
#include "key.h"

/*-------------------------------- 显示布局 --------------------------------*/
#define MENU_ROW_HEIGHT   16u   /* OLED_8X16 字体行高，单位：像素 */
#define MENU_ROW_NUM       4u   /* 屏幕可显示的行数 */
#define MENU_ITEM_WIDTH  128u   /* 光标反相区域的宽度，单位：像素 */

/*-------------------------------- 一级菜单 --------------------------------*/
#define MENU1_ENTRY_NUM          2u   /* 基础部分、发挥部分 */
#define MENU1_ENTRY_BASIC        0u
#define MENU1_ENTRY_PERFORMANCE  1u
#define MENU1_FIRST_ROW          2u   /* 两个选项显示在第 3、4 行，行号从 0 开始 */

/*--------------------------- 二级菜单：基础部分 ----------------------------*/
#define MENU2_BASIC_TASK_NUM     4u                            /* 基础任务数量 */
#define MENU2_BASIC_ENTRY_NUM    (MENU2_BASIC_TASK_NUM + 1u)   /* 末项是"上一步" */
#define MENU2_BASIC_BACK_ENTRY   MENU2_BASIC_TASK_NUM          /* "上一步"的条目序号 */

/*--------------------------- 二级菜单：发挥部分 ----------------------------*/
#define MENU2_PERF_ENTRY_NUM     4u
#define MENU2_PERF_ENTRY_TARGET  0u   /* 第 1 行：指定位置，确认即执行任务 */
#define MENU2_PERF_ENTRY_BACK    1u   /* 第 2 行：上一步 */
#define MENU2_PERF_ENTRY_INC     2u   /* 第 3 行：目标位置 +1cm */
#define MENU2_PERF_ENTRY_DEC     3u   /* 第 4 行：目标位置 -1cm */

/*----------------------------- 目标位置调节 -------------------------------*/
#define MENU_TARGET_POSITION_STEP_CM 1       /* 每次调节的步长，单位：cm */
#define MENU_TARGET_POSITION_MAX_CM  10      /* 目标位置上限，单位：cm */
#define MENU_TARGET_POSITION_MIN_CM  (-10)   /* 目标位置下限，单位：cm */

/*-------------------------------- 三级菜单 --------------------------------*/
#define MENU3_BACK_HINT_ROW   3u   /* "上一步"提示显示在第 4 行，行号从 0 开始 */
#define MENU3_BACK_HINT_TEXT  "<-上一步        "

/* "上一步"条目文本，补足空格把整行填满，避免残留上一帧的字符 */
#define MENU_BACK_TEXT        "<-上一步        "

/**
 * @brief 三级菜单的任务编号。
 */
typedef enum {
    MENU_TASK_STABLE_POSITION = 0,   /* 1.定点稳定控制 */
    MENU_TASK_BACK_AND_FORTH,        /* 2.折返运动与时效 */
    MENU_TASK_CONTINUOUS_TRACK,      /* 3.连续跟踪运动 */
    MENU_TASK_ANTI_DISTURBANCE,      /* 4.抗干扰能力 */
    MENU_TASK_TARGET_POSITION,       /* 发挥部分：指定位置 */
} Menu_Task_e;

/**
 * @brief 菜单条目。
 */
typedef struct {
    char *text;          /* 条目显示文本 */
    Menu_Task_e task;    /* 确认后要执行的任务编号 */
} Menu_Item_s;

/* 一级菜单页面文本，行号 0~3 依次对应 */
static char *const kMenu1Text[MENU_ROW_NUM] = {
    "电子2503 李骏宏 ",
    "管球平衡控制系统 ",
    "1.基础部分      ",
    "2.发挥部分      ",
};

/* 基础部分任务列表，顺序与 MENU_TASK_xxx 一致 */
static const Menu_Item_s kMenu2BasicItem[MENU2_BASIC_TASK_NUM] = {
    { "1.定点稳定控制  ", MENU_TASK_STABLE_POSITION },
    { "2.折返运动与时效", MENU_TASK_BACK_AND_FORTH },
    { "3.连续跟踪运动  ", MENU_TASK_CONTINUOUS_TRACK },
    { "4.抗干扰能力    ", MENU_TASK_ANTI_DISTURBANCE },
};

/* 一级菜单光标，静态保存，从下级菜单返回后仍停在原来那一项 */
static uint8_t menu1_cursor = MENU1_ENTRY_BASIC;

/* 指定小球位置，单位：cm，范围 MENU_TARGET_POSITION_MIN_CM ~ MENU_TARGET_POSITION_MAX_CM */
static int8_t specified_location_cm = 0;

/* 当前小球位置，单位：cm，由任务代码更新 */
static uint8_t location_cm = 0u;

/**
 * @brief 计算某一行的纵向像素坐标。
 *
 * @param row 行号，从 0 开始，范围 0~3
 * @return 该行左上角的纵坐标，单位：像素
 */
static int16_t menu_row_to_y(uint8_t row)
{
    return (int16_t)(row * MENU_ROW_HEIGHT);
}

/**
 * @brief 光标上移一项，首项回绕到末项。
 *
 * @param entry 当前条目序号，从 0 开始
 * @param entry_num 条目总数
 * @return 移动后的条目序号
 */
static uint8_t menu_move_prev(uint8_t entry, uint8_t entry_num)
{
    if (entry == 0u) {
        return (uint8_t)(entry_num - 1u);
    }

    return (uint8_t)(entry - 1u);
}

/**
 * @brief 光标下移一项，末项回绕到首项。
 *
 * @param entry 当前条目序号，从 0 开始
 * @param entry_num 条目总数
 * @return 移动后的条目序号
 */
static uint8_t menu_move_next(uint8_t entry, uint8_t entry_num)
{
    return (uint8_t)((entry + 1u) % entry_num);
}

/**
 * @brief 计算列表滚动偏移。
 *
 * 条目数比屏幕行数多时，光标越过后把窗口上滚，保证光标所在条目可见。
 *
 * @param entry 光标所在条目序号，从 0 开始
 * @param entry_num 条目总数
 * @return 屏幕第 1 行显示的条目序号
 */
static uint8_t menu_scroll_offset(uint8_t entry, uint8_t entry_num)
{
    if ((entry_num > MENU_ROW_NUM) && (entry >= MENU_ROW_NUM)) {
        return (uint8_t)(entry - MENU_ROW_NUM + 1u);
    }

    return 0u;
}

/**
 * @brief 反相显示光标所在行，作为选中效果。
 *
 * 每帧的页面文都是重新绘制的，所以这里只需要反相当前行，
 * 上一帧的反相会被重绘覆盖掉，不需要额外清除。
 *
 * @param row 行号，从 0 开始，范围 0~3
 */
static void menu_highlight(uint8_t row)
{
    OLED_ReverseArea(0, menu_row_to_y(row), MENU_ITEM_WIDTH, MENU_ROW_HEIGHT);
}

/**
 * @brief 目标位置加一个步长，到上限后保持在上限。
 *
 * @param position_cm 当前目标位置，单位：cm
 * @return 调节后的目标位置，单位：cm
 */
static int8_t menu_target_position_inc(int8_t position_cm)
{
    if (position_cm >= MENU_TARGET_POSITION_MAX_CM) {
        return (int8_t)MENU_TARGET_POSITION_MAX_CM;
    }

    return (int8_t)(position_cm + MENU_TARGET_POSITION_STEP_CM);
}

/**
 * @brief 目标位置减一个步长，到下限后保持在下限。
 *
 * @param position_cm 当前目标位置，单位：cm
 * @return 调节后的目标位置，单位：cm
 */
static int8_t menu_target_position_dec(int8_t position_cm)
{
    if (position_cm <= MENU_TARGET_POSITION_MIN_CM) {
        return (int8_t)MENU_TARGET_POSITION_MIN_CM;
    }

    return (int8_t)(position_cm - MENU_TARGET_POSITION_STEP_CM);
}

/**
 * @brief 绘制一级菜单页面文本。
 */
static void menu1_show(uint8_t entry)
{
    OLED_Clear();
    uint8_t row;

    for (row = 0u; row < MENU_ROW_NUM; row++) {
        OLED_ShowString(0, menu_row_to_y(row), kMenu1Text[row], OLED_8X16);
    }

    menu_highlight((uint8_t)(MENU1_FIRST_ROW + entry));
    OLED_Update();
}

/**
 * @brief 绘制基础部分二级菜单页面。
 *
 * @param offset 滚动偏移，屏幕第 1 行显示的条目序号
 * @param entry 光标所在条目序号，从 0 开始
 */
static void menu2_basic_show(uint8_t offset, uint8_t entry)
{
    OLED_Clear();
    uint8_t row;

    for (row = 0u; row < MENU_ROW_NUM; row++) {
        uint8_t item = (uint8_t)(offset + row);

        if (item < MENU2_BASIC_TASK_NUM) {
            OLED_ShowString(0, menu_row_to_y(row), kMenu2BasicItem[item].text, OLED_8X16);
        } else {
            OLED_ShowString(0, menu_row_to_y(row), MENU_BACK_TEXT, OLED_8X16);
        }
    }

    menu_highlight((uint8_t)(entry - offset));
    OLED_Update();
}

/**
 * @brief 绘制发挥部分二级菜单页面。
 *
 * @param entry 光标所在条目序号，从 0 开始
 */
static void menu2_performance_show(uint8_t entry)
{
    OLED_Clear();
    /* 目标位置就显示在第 1 行，光标停在这一行按确认即开始执行任务 */
    OLED_Printf(0, menu_row_to_y(0u), OLED_8X16, "指定位置:%dcm   ", specified_location_cm);
    OLED_ShowString(0, menu_row_to_y(1u), MENU_BACK_TEXT, OLED_8X16);
    OLED_ShowString(0, menu_row_to_y(2u), "  +1cm          ", OLED_8X16);
    OLED_ShowString(0, menu_row_to_y(3u), "  -1cm          ", OLED_8X16);

    menu_highlight(entry);
    OLED_Update();
}

/**
 * @brief 绘制三级菜单页面文本。
 */
static void menu3_show(void)
{
    OLED_Clear();
    OLED_Printf(0, 0, OLED_8X16, "小球位置:%dcm   ", location_cm);
    OLED_ShowString(0, menu_row_to_y(MENU3_BACK_HINT_ROW), MENU3_BACK_HINT_TEXT, OLED_8X16);
    menu_highlight(MENU3_BACK_HINT_ROW);
    OLED_Update();
}

static void menu3_run(Menu_Task_e task)
{
    uint8_t key_num;

    while (1) {
        switch (task) {
        case MENU_TASK_STABLE_POSITION:
            /* TODO: 在这里填入"定点稳定控制"的任务执行函数 */
            break;

        case MENU_TASK_BACK_AND_FORTH:
            /* TODO: 在这里填入"折返运动与时效"的任务执行函数 */
            break;

        case MENU_TASK_CONTINUOUS_TRACK:
            /* TODO: 在这里填入"连续跟踪运动"的任务执行函数 */
            break;

        case MENU_TASK_ANTI_DISTURBANCE:
            /* TODO: 在这里填入"抗干扰能力"的任务执行函数 */
            break;

        case MENU_TASK_TARGET_POSITION:
            /* TODO: 在这里填入"指定小球位置"的任务执行函数 */
            break;

        default:
            /* 未定义的任务编号：不执行任何动作，只显示小球位置 */
            break;
        }

        key_num = Key_GetNum();
        if (key_num == KEY_NUM_KEY3) {
            return;                 /* 上一步：回到二级菜单 */
        }

        menu3_show();
    }
}

/**
 * @brief 一级菜单循环：显示菜单并等待确认。
 *
 * @return 选中的条目序号 MENU1_ENTRY_BASIC / MENU1_ENTRY_PERFORMANCE
 */
static uint8_t menu1_select(void)
{
    uint8_t key_num;

    while (1) {
        key_num = Key_GetNum();

        if (key_num == KEY_NUM_KEY3) {
            return menu1_cursor;
        }

        if (key_num == KEY_NUM_KEY1) {
            menu1_cursor = menu_move_prev(menu1_cursor, MENU1_ENTRY_NUM);
        } else if (key_num == KEY_NUM_KEY2) {
            menu1_cursor = menu_move_next(menu1_cursor, MENU1_ENTRY_NUM);
        } else {
            /* 无按键按下，保持当前光标位置 */
        }

        menu1_show(menu1_cursor);
    }
}

/**
 * @brief 基础部分二级菜单循环。
 *
 * 确认前 4 项进入三级菜单执行任务，确认"上一步"回到一级菜单。
 */
static void menu2_basic_run(void)
{
    uint8_t entry = 0u;
    uint8_t key_num;

    while (1) {
        key_num = Key_GetNum();

        if (key_num == KEY_NUM_KEY1) {
            entry = menu_move_prev(entry, MENU2_BASIC_ENTRY_NUM);
        } else if (key_num == KEY_NUM_KEY2) {
            entry = menu_move_next(entry, MENU2_BASIC_ENTRY_NUM);
        } else if (key_num == KEY_NUM_KEY3) {
            if (entry == MENU2_BASIC_BACK_ENTRY) {
                return;                                     /* 上一步：回到一级菜单 */
            }

            menu3_run(kMenu2BasicItem[entry].task);         /* 执行任务，返回后重新显示本页 */
        } else {
            /* 无按键按下，保持当前光标位置 */
        }

        menu2_basic_show(menu_scroll_offset(entry, MENU2_BASIC_ENTRY_NUM), entry);
    }
}

/**
 * @brief 发挥部分二级菜单循环。
 *
 * 确认目标位置行进入三级菜单执行任务，确认 +1cm / -1cm 行调节目标位置，
 * 确认"上一步"回到一级菜单。
 */
static void menu2_performance_run(void)
{
    uint8_t entry = MENU2_PERF_ENTRY_TARGET;
    uint8_t key_num;

    while (1) {
        key_num = Key_GetNum();

        if (key_num == KEY_NUM_KEY1) {
            entry = menu_move_prev(entry, MENU2_PERF_ENTRY_NUM);
        } else if (key_num == KEY_NUM_KEY2) {
            entry = menu_move_next(entry, MENU2_PERF_ENTRY_NUM);
        } else if (key_num == KEY_NUM_KEY3) {
            switch (entry) {
            case MENU2_PERF_ENTRY_TARGET:
                menu3_run(MENU_TASK_TARGET_POSITION);       /* 执行任务，返回后重新显示本页 */
                break;

            case MENU2_PERF_ENTRY_BACK:
                return;                                     /* 上一步：回到一级菜单 */

            case MENU2_PERF_ENTRY_INC:
                specified_location_cm = menu_target_position_inc(specified_location_cm);
                break;

            case MENU2_PERF_ENTRY_DEC:
                specified_location_cm = menu_target_position_dec(specified_location_cm);
                break;

            default:
                /* 条目序号超出定义范围：不做任何动作 */
                break;
            }
        } else {
            /* 无按键按下，保持当前光标位置 */
        }

        menu2_performance_show(entry);
    }
}

/**
 * @brief 菜单入口：逐级进入 一级 -> 二级 -> 三级 菜单。
 *
 * 一级菜单确认后进入对应的二级菜单，二级菜单确认后进入三级菜单执行任务，
 * 各级菜单的"上一步"返回上一级。函数内部循环处理，不会主动返回。
 *
 * @return 固定返回 0
 */
int Main_Menu(void)
{
    while (1) {
        if (menu1_select() == MENU1_ENTRY_BASIC) {
            menu2_basic_run();
        } else {
            menu2_performance_run();
        }
    }

    return 0;
}
