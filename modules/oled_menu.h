#ifndef OLED_MENU_H
#define OLED_MENU_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 菜单入口：逐级进入 一级 -> 二级 -> 三级 菜单。
 *
 * 一级菜单（基础部分 / 发挥部分）确认后进入对应的二级菜单，二级菜单确认后
 * 进入三级菜单执行任务，各级菜单的"上一步"返回上一级菜单。函数内部循环
 * 处理菜单显示与按键扫描，不会主动返回。
 *
 * @return 固定返回 0
 */
int Main_Menu(void);

#ifdef __cplusplus
}
#endif

#endif /* OLED_MENU_H */
