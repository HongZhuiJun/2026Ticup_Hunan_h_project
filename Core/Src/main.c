/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "delay.h"
#include "oled_menu.h"
#include "key.h"
#include "k230_to_stm32.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 视觉调试显示的刷新周期(ms)，避免 OLED 刷新占满主循环 */
#define K230_DEBUG_DISPLAY_PERIOD_MS 50u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static K230ToStm32Instance *k230_vision = NULL; /* K230 视觉接收实例 */
static uint32_t k230_display_last_ms = 0u;      /* 上次刷新调试显示的时刻 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();

  /* 初始化 K230 视觉接收：USART1 + DMA1_Channel5 空闲中断不定长接收。
     必须在 MX_USART1_UART_Init() 之后调用（注册时要读取已关联的 DMA 句柄） */
  K230ToStm32_Init_Config_s k230_config = {
      .uart_handle = &huart1,
  };
  k230_vision = K230ToStm32Register(&k230_config);
  if (k230_vision == NULL) {
      /* 注册失败：串口 RX DMA 未配置或实例池已满 */
      OLED_ShowString(0, 0, "K230 reg fail", OLED_8X16);
      OLED_Update();
  } else if (K230ToStm32Start(k230_vision) == 0u) {
      /* 启动失败：串口状态异常 */
      OLED_ShowString(0, 0, "K230 start fail", OLED_8X16);
      OLED_Update();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* ===== K230 视觉接收联调观察（临时显示，接入控制环后可删）=====
       第 1 行：钢球在水管上的物理位置(cm)：右为正、左为负、0 为水管中点；
               位置无效（钢球丢失或视觉离线）时显示 lost
       第 2 行：原始像素，px 固定不动说明没收到数据；
               px 有值但第 1 行是 lost 说明钢球丢失（current_x = -1）
       第 3 行：视觉链路在线状态（超过 200ms 没收到合法帧即离线）
       第 4 行：合法帧数 / 校验失败帧数，校验数持续增长说明协议或波特率不对 */
    uint32_t now_ms = HAL_GetTick();
    if ((k230_vision != NULL) &&
        ((uint32_t)(now_ms - k230_display_last_ms) >= K230_DEBUG_DISPLAY_PERIOD_MS)) {
        k230_display_last_ms = now_ms;

        /* 先清零：注册失败时 GetData() 会直接返回且不填结构体，
           此时下面仍会读 current_x_px，清零可避免读到未初始化内存 */
        K230_Data_s k230_data = {0};
        if (K230ToStm32GetData(k230_vision, &k230_data) != 0u) {
            /* 浮点拆成整数显示：newlib-nano 默认不带 %f 支持。
               符号必须单独取，不能借"整数部分的符号"：位置在 (-1,0) cm 时
               整数部分为 0，会丢掉负号把 -0.50 显示成 +00.50，
               而这恰好是控制目标附近的区间 */
            int32_t pos_cm_x100 = (int32_t)(k230_data.pos_cm * 100.0f);
            uint32_t pos_magnitude = (pos_cm_x100 < 0) ? (uint32_t)(-pos_cm_x100)
                                                       : (uint32_t)pos_cm_x100;
            /* 坐标按 8px/字符排；符号 1 格 + 整数 2 格 + 小数点 1 格 + 小数 2 格 */
            OLED_ShowString(0, 0, "pos:", OLED_8X16);                        /* x   0..31  */
            OLED_ShowChar(32, 0, (pos_cm_x100 < 0) ? '-' : '+', OLED_8X16);  /* x  32..39  */
            OLED_ShowNum(40, 0, pos_magnitude / 100u, 2, OLED_8X16);         /* x  40..55  */
            OLED_ShowString(56, 0, ".", OLED_8X16);                          /* x  56..63  */
            OLED_ShowNum(64, 0, pos_magnitude % 100u, 2, OLED_8X16);         /* x  64..79  */
            OLED_ShowString(88, 0, "cm", OLED_8X16);                         /* x  88..103 */
        } else {
            /* 用空格补足 16 字符(128px)，盖掉上次显示残留的像素 */
            OLED_ShowString(0, 0, "pos:  lost      ", OLED_8X16);
        }

        OLED_Printf(0, 16, OLED_8X16, "px:%-5d t:%d ", k230_data.current_x_px, k230_data.target_x_px);
        OLED_Printf(0, 32, OLED_8X16, "link:%s",
                    (K230ToStm32IsOnline(k230_vision) != 0u) ? "online " : "offline");

        K230ToStm32_Stats_s k230_stats;
        K230ToStm32GetStats(k230_vision, &k230_stats);
        OLED_Printf(0, 48, OLED_8X16, "fr:%-5lu er:%lu ",
                    (unsigned long)(k230_stats.frame_count % 100000UL),
                    (unsigned long)(k230_stats.checksum_error_count % 1000UL));

        OLED_Update();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
