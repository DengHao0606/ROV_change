/* USER CODE BEGIN Header */
/**
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "cJSON.h"
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
#define RX_BUFFER_SIZE 256
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_char;
uint16_t rx_index = 0;
uint16_t short3 = 0;
typedef struct {
  char cmd[16];
  int value;
} CommandData;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void parse_json_with_cjson(uint8_t *json_str);
//重定向printf
int fputc(int ch,FILE *f)
{
  uint8_t temp[1]={ch};
  HAL_UART_Transmit(&huart4,temp,1,2);
  return ch;
}

// 使用 cJSON 解析 JSON 数据
void parse_json_with_cjson(uint8_t *json_str)
{
    cJSON *root = cJSON_Parse((char *)json_str);
    if (root == NULL)
    {
        printf("Error: Invalid JSON!\r\n");
        return;
    }

    // 提取 "cmd" 字段
    cJSON *cmd_item = cJSON_GetObjectItem(root, "cmd");
    if (cmd_item == NULL)
    {
        printf("Error: 'cmd' not found!\r\n");
        cJSON_Delete(root);
        return;
    }

    // 提取 "value" 字段
    cJSON *value_item = cJSON_GetObjectItem(root, "value");
    if (value_item == NULL)
    {
        printf("Error: 'value' not found!\r\n");
        cJSON_Delete(root);
        return;
    }
    //提取"name" 字段
    cJSON *name_item = cJSON_GetObjectItem(root, "name");
    if (value_item == NULL)
    {
        printf("Error: 'value' not found!\r\n");
        cJSON_Delete(root);
        return;
    }
    cJSON *number_item = cJSON_GetObjectItem(root, "number");
    if (number_item == NULL)
    {
        printf("Error: 'value' not found!\r\n");
        cJSON_Delete(root);
        return;
    }
    cJSON *ps2_item = cJSON_GetObjectItem(root, "ps2");
    if (number_item == NULL)
    {
        printf("Error: 'value' not found!\r\n");
        cJSON_Delete(root);
        return;
    }
    // 打印解析结果
    printf("CMD: %s, VALUE: %d, NAME:%s, NUMBER: %d, PS2: %f\r\n", cmd_item->valuestring, value_item->valueint,
      name_item->valuestring, number_item->valueint, ps2_item->valuedouble);

    // 执行命令（示例：控制 LED�?
    if (strcmp(cmd_item->valuestring, "led") == 0)
    {
        if (value_item->valueint == 1)
        {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);  // LED ON
            printf("LED ON\r\n");
        }
        else if (value_item->valueint == 0)
        {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);  // LED OFF
            printf("LED OFF\r\n");
        }
    }

    cJSON_Delete(root);  // 释放 cJSON 内存
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
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  printf("STM32F407 USART4 JSON Parser Ready\r\n");
  HAL_Delay(2000);
  HAL_UART_Receive_IT(&huart5, &rx_char, 1);  // 启动接收中断
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// UART5 接收中断回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        if (rx_char == '\n')  // �?测到换行符（�?条完�? JSON�?
        {
            rx_buffer[rx_index] = '\0';  // 字符串结束符
            printf("JSON: ");
            printf((char *)rx_buffer);
            printf("\r\n");

            parse_json_with_cjson(rx_buffer);  // 解析 JSON
            rx_index = 0;  // 重置缓冲�?
        }
        else
        {
            if (rx_index < RX_BUFFER_SIZE - 1)
                rx_buffer[rx_index++] = rx_char;
            else
            {
              printf("Error: RX Buffer Overflow!\r\n");
                rx_index = 0;  // 缓冲区溢出，重置
            }
        }
        HAL_UART_Receive_IT(&huart5, &rx_char, 1);  // 重新启用接收
    }
}


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
    printf("Error");
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);  // LED 闪烁表示错误
    HAL_Delay(500);
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
