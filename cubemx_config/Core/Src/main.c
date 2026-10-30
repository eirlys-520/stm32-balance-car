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
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "pt_public.h"
#include "bt_uart.h"
#include "display.h"
#include "oled.h"
#include "pt_mpu6050.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "encode.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tb6612.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
PtGlobalData_t gGlobalData;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART2)
    {
        BT_UART_ProcessByte(g_bt_rx.rx_byte);
        HAL_UART_Receive_IT(&huart2, &g_bt_rx.rx_byte, 1);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C2_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();

    Car_Go_High();
    startpwm();

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    Display_Init();
    BT_UART_Init();
    MPU6050_Init();
    Display_ShowStartupPage();
    HAL_Delay(1000);

    //    gGlobalData.BatVolt = 12.4f;
    //    gGlobalData.PitchAngle = -1.8f;
    //    gGlobalData.CurSpeed = 135;
    //    gGlobalData.TargetMode = PT_WORKMODE_FORWARD;
    //    gGlobalData.TargetSpeed = PT_WORKSPEED_HIGH;
    HAL_TIM_Base_Start_IT(&htim1);
    while (1)
    {
		OLED_Clear();
        //        /* 先显示主页面 */
        //Display_ShowMainPage(&gGlobalData);

        //        /* 如果收到蓝牙一帧数据，可以在这里做调试显示 */
        if (g_bt_rx.frame_ready)
        {
            OLED_Clear();
            OLED_ShowString(0, 0, "BT RX:");
            OLED_ShowString(0, 16, (char*)g_bt_rx.rx_buffer);
            OLED_Refresh();

            HAL_Delay(500);

            BT_UART_ClearFrame();
        }
		char buf[24]={0};
		 memset(buf, 0, sizeof(buf));
        snprintf(buf, sizeof(buf), "A %f", gGlobalData.PitchAngle);
        OLED_ShowString(0, 10, buf);

        memset(buf, 0, sizeof(buf));
        snprintf(buf, sizeof(buf), "Y %f", gGlobalData.data.gyro_y);
        OLED_ShowString(0, 20, buf);
        //pwm_sudu = sudu(left_encode, left_encode);
//        outpwm = pwm_zhili + pwm_sudu;

//        memset(buf, 0, sizeof(buf));
//        snprintf(buf, sizeof(buf), "%d", outpwm);
//        OLED_ShowString(0, 50, buf);

        OLED_Refresh();
		
        HAL_Delay(100);
		

    }
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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
