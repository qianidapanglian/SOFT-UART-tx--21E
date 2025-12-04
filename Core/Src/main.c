/* USER CODE BEGIN Header */
/**
软件串口发送实现21e的数字编码

软件串口为PB11，开始发送按键为PB12，结束发送按键为PB7，这两个按键设置为外部中断.
切换数据按键PA2，这三个按键都是输入低电平触发。

开始按键按下后，先发送起始数据包，然后循环发送编码数字的数据包，按下结束按键后停止循环，并且发送一个结束数据包。
软件串口的最高速率为1000bit/s。

  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t receiveData[50]={0};
extern uint16_t size=0;
#define test_len 3//如果定义为5，但实际只有3个元素，那么数据为0的字节也会发送过去
uint8_t testData[ test_len ] = {0x01,0x65,0x13};


#define start0_len 1
uint8_t start0_buffer[start0_len]={0xAA};//每个数据包的起始帧



#define start_len 2
uint8_t start_buffer[start_len]={0xc8,0xc8};//传输开始数据帧


#define data_len 2
uint8_t data_buffer[data_len]={0x12,0x34};//编码数据数据帧

#define stop_len 2
uint8_t stop_buffer[stop_len]={0xc9,0xc9};//传输结束数据帧

uint16_t key_start=0;
uint16_t key_stop=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//阻塞式软件串口方式，最高bit率：1000bit/s（hal——delay函数的最小延时为1ms）。
void SoftwareUART_Transmit(GPIO_TypeDef* txPort, uint16_t txPin, 
                          uint8_t* data, uint16_t length, uint16_t bitDelayMs)
{
    // 确保HAL库已初始化
    if(txPort == NULL || data == NULL || length == 0) return;
    
//    // 计算半位延时(用于更精确的时序)
//    uint16_t halfBitDelay = bitDelayMs / 2;
//    我改
    for(uint16_t i = 0; i < length; i++)
    {
        uint8_t byte = data[i];
        
        // 发送起始位(低电平)
        HAL_GPIO_WritePin(txPort, txPin, GPIO_PIN_RESET);
        HAL_Delay(bitDelayMs-1);
        
        // 发送8位数据(LSB first)
        for(uint8_t bit = 0; bit < 8; bit++)
        {
            if(byte & (1 << bit))
            {
                HAL_GPIO_WritePin(txPort, txPin, GPIO_PIN_SET);
            }
            else
            {
                HAL_GPIO_WritePin(txPort, txPin, GPIO_PIN_RESET);
            }
            HAL_Delay(bitDelayMs-1);
        }
        
        // 发送停止位(高电平)
        HAL_GPIO_WritePin(txPort, txPin, GPIO_PIN_SET);
        HAL_Delay(bitDelayMs-1);
    }
}




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
  /* USER CODE BEGIN 2 */
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(key_start==1)//按下PB12开始按键
		 {
			 
			 key_start=0;
			 SoftwareUART_Transmit(SOFT_UART_GPIO_Port, SOFT_UART_Pin, 
                          start0_buffer,start0_len,10);//发送数据包起始帧
			SoftwareUART_Transmit(SOFT_UART_GPIO_Port, SOFT_UART_Pin, 
                          start_buffer,start_len,10);//发送传输开始数据帧
			 
			 
			 
		while(1)//循环发送数据帧
		{
			HAL_Delay(1000);
			
			SoftwareUART_Transmit(SOFT_UART_GPIO_Port, SOFT_UART_Pin, 
                          start0_buffer,start0_len,10);//发送数据包起始帧
			SoftwareUART_Transmit(SOFT_UART_GPIO_Port, SOFT_UART_Pin, 
                          data_buffer,data_len,10);//发送编码数据数据帧
			if(key_stop==1)//按下PB7停止按键
			{
				key_start=0;
				key_stop=0;
				HAL_Delay(1000);
				SoftwareUART_Transmit(SOFT_UART_GPIO_Port, SOFT_UART_Pin, 
                          start0_buffer,start0_len,10);//发送数据包起始帧
				SoftwareUART_Transmit(SOFT_UART_GPIO_Port, SOFT_UART_Pin, 
                          stop_buffer,stop_len,10);//发送传输结束数据帧
			break;
			}
		}
		
			 
		 }//PB11为串口发送
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

#ifdef  USE_FULL_ASSERT
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
