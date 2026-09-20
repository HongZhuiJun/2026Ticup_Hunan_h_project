#ifndef OLED_MENU_H
#define OLED_MENU_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 显示菜单页1。
 *
 * 菜单页显示完成后进入自身循环，返回下一个要进入的菜单编号；
 * 具体跳转逻辑由调用方(应用层)根据返回值处理。
 *
 * @return 下一个菜单编号
 */

void menu1_show(void);
void menu2_1_show(void);
void menu2_2_show(void);
int menu1(void);

#ifdef __cplusplus
}
#endif

#endif /* OLED_MENU_H */
