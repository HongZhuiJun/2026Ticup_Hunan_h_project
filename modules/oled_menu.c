#include "oled_menu.h"
#include "oled.h"
#include "key.h"
#include "main.h"

uint8_t KeyNum=0;					//用于存储键码值
uint8_t D=2;					//用于存储目前按键是上还是下,默认下一项
uint8_t specified_location_cm=0;		//用于存储指定的小球位置,单位cm
uint8_t location_cm=0;				//用于存储当前小球位置,单位cm

/***************************************************************************************/
/*把一级菜单的flag放到全局变量,静态存储,不会丢失数据,所以从二级菜单回来以后
仍能回到那一行,而不是每次都回到第一行*/
uint8_t flag=3;
/***************************************************************************************/

void menu1_show(void)
{
    OLED_ShowString(0,  0, "电子2503 李骏宏 ", OLED_8X16);
    OLED_ShowString(0, 16, "管球平衡控制系统 ", OLED_8X16);
    OLED_ShowString(0, 32, "1.基础部分      ", OLED_8X16);
    OLED_ShowString(0, 48, "2.发挥部分      ", OLED_8X16);
    OLED_Update();
}

void menu2_basic_show(void)
{
    OLED_ShowString(0,  0, "1.定点稳定控制   ", OLED_8X16);
    OLED_ShowString(0, 16, "2.折返运动与时效 ", OLED_8X16);
    OLED_ShowString(0, 32, "3.连续跟踪运动   ", OLED_8X16);
    OLED_ShowString(0, 48, "4.抗干扰能力     ", OLED_8X16);
    OLED_Update();
}

void menu2_performance_show(void)
{
	OLED_ShowString(0,  0, "指定小球目标位置:", OLED_8X16);
	OLED_Printf(0,16,OLED_8X16,"位置:%dcm   ",specified_location_cm);
	OLED_Update();
}

void menu3_show(void)
{
	OLED_Printf(0,0,OLED_8X16,"小球位置:%dcm   ",location_cm);
	OLED_Update();
}

int menu1(void)
{   
    menu1_show();
    while (1)  
    {
        KeyNum = Key_GetNum();
		if(KeyNum == 1)//上一项
		{
			flag--;
			if(flag == 2){flag = 4;}
			D = 1;//上一项标志
			OLED_AnimUpdate();//按下按钮,刷新一次动画效果
		}
		if(KeyNum == 2)//下一项
		{
			flag++;
			if(flag == 5){flag = 3;}
			D = 2;//下一项标志
			OLED_AnimUpdate();//按下按钮,刷新一次动画效果
		}
		if(KeyNum == 3)//确认
		{
			OLED_Clear();
			OLED_Update();
			OLED_AnimUpdate();//按下按钮,刷新一次动画效果
			return flag;
		}

        switch(flag)
		{
			case 3:
			{
				//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
				menu1_show();
				if(D==2){OLED_Animation(0,16,64,16,0,32,64,16);}
				if(D==1){OLED_Animation(0,48,64,16,0,32,64,16);}			
				break;
			}
			case 4:
			{
				//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
				menu1_show();
				if(D==2){OLED_Animation(0,32,64,16,0,48,64,16);}
				if(D==1){OLED_Animation(0,0 ,64,16,0,48,64,16);}
				break;
			}
        }
    }
}

