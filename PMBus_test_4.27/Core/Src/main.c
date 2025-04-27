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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// ???????
typedef struct {
  float voltage;  // ??(V)
  float current;  // ??(A)
  float power;    // ??(W)
  uint8_t valid;  // ??????
} PowerData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PMBUS_ADDR 0x60        // PMBus????,???96
#define MAX_RETRIES 3          // ??????
#define PMBUS_TIMEOUT 100      // ????(ms)

// PMBus????
#define PMBUS_CMD_VOUT_MODE   0x20
#define PMBUS_CMD_READ_VOUT   0x88
#define PMBUS_CMD_READ_IOUT   0x8D
#define PMBUS_CMD_READ_POUT   0x96
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//I2C_HandleTypeDef hi2c1;  // I2C??
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// ????
int fputc(int ch, FILE *f);
void PMBus_ScanDevices(void);
HAL_StatusTypeDef PMBus_ReadWord(uint8_t devAddr, uint8_t command, uint16_t *data);
float PMBus_ReadVout(uint8_t devAddr);
float PMBus_ReadIout(uint8_t devAddr);
PowerData_t PMBus_ReadPowerData(uint8_t devAddr);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//???printf
int fputc(int ch,FILE *f)
{
  uint8_t temp[1]={ch};
  HAL_UART_Transmit(&huart1,temp,1,2);
  return ch;
}

/**
  * @brief  ??I2C??????
  */
void PMBus_ScanDevices(void)
{
  uint8_t i, ret;
  printf("I2C Device Scan:\r\n");
  
  for(i=1; i<128; i++)
  {
    ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);
    if(ret == HAL_OK)
    {
      printf("Device found at 0x%02X\r\n", i);
    }
  }
}

/**
  * @brief  ?PMBus???????(16?)
  * @param  devAddr: ????(7?)
  * @param  command: PMBus??
  * @param  data: ?????????
  * @retval HAL??
  */
HAL_StatusTypeDef PMBus_ReadWord(uint8_t devAddr, uint8_t command, uint16_t *data)
{
  uint8_t buffer[2];
  HAL_StatusTypeDef status;
  
  status = HAL_I2C_Master_Transmit(&hi2c1, devAddr << 1, &command, 1, PMBUS_TIMEOUT);
  if(status != HAL_OK) return status;
  
  status = HAL_I2C_Master_Receive(&hi2c1, devAddr << 1, buffer, 2, PMBUS_TIMEOUT);
  if(status == HAL_OK) {
    *data = (uint16_t)((buffer[1] << 8) | buffer[0]);
  }
  return status;
}

/**
  * @brief  ??????
  * @param  devAddr: ????(7?)
  * @retval ????(V)?????-1.0
  */
 float PMBus_ReadVout(uint8_t devAddr)
 {
   uint16_t raw;
   uint8_t mode;
   uint8_t retry = MAX_RETRIES;
   
   while(retry--)
   {
     if(HAL_I2C_Mem_Read(&hi2c1, devAddr << 1, PMBUS_CMD_VOUT_MODE, 
                        I2C_MEMADD_SIZE_8BIT, &mode, 1, PMBUS_TIMEOUT) == HAL_OK)
     {
       if(PMBus_ReadWord(devAddr, PMBUS_CMD_READ_VOUT, &raw) == HAL_OK)
       {
         if(mode & 0x40) { // ????
           int exponent = (int8_t)(mode & 0x1F) - 24;
           return raw * powf(2.0f, (float)exponent);
         }
       }
     }
     HAL_Delay(10);
   }
   return -1.0f;
 }

/**
  * @brief  ??????
  * @param  devAddr: ????(7?)
  * @retval ????(A)?????-1.0
  */
 float PMBus_ReadIout(uint8_t devAddr)
 {
   uint16_t raw;
   uint8_t mode;
   uint8_t retry = MAX_RETRIES;
   
   while(retry--)
   {
     if(HAL_I2C_Mem_Read(&hi2c1, devAddr << 1, PMBUS_CMD_VOUT_MODE, 
                        I2C_MEMADD_SIZE_8BIT, &mode, 1, PMBUS_TIMEOUT) == HAL_OK)
     {
       if(PMBus_ReadWord(devAddr, PMBUS_CMD_READ_IOUT, &raw) == HAL_OK)
       {
         if(mode & 0x40) { // ????
           int exponent = (int8_t)(mode & 0x1F) - 24;
           return raw * powf(2.0f, (float)exponent);
         }
       }
     }
     HAL_Delay(10);
   }
   return -1.0f;
 }

/**
  * @brief  ?????????(????????)
  * @param  devAddr: ????(7?)
  * @retval PowerData_t???
  */
 PowerData_t PMBus_ReadPowerData(uint8_t devAddr)
 {
   PowerData_t data = {0};
   uint16_t raw;
   uint8_t mode;
   uint8_t retry = MAX_RETRIES;
   
   // ????
   while(retry--)
   {
     if(HAL_I2C_Mem_Read(&hi2c1, devAddr << 1, PMBUS_CMD_VOUT_MODE, 
                        I2C_MEMADD_SIZE_8BIT, &mode, 1, PMBUS_TIMEOUT) == HAL_OK)
     {
       break;
     }
     HAL_Delay(10);
   }
   if(retry == 0) return data;
   
   // ????
   retry = MAX_RETRIES;
   while(retry--)
   {
     if(PMBus_ReadWord(devAddr, PMBUS_CMD_READ_VOUT, &raw) == HAL_OK)
     {
       if(mode & 0x40) {
         int exponent = (int8_t)(mode & 0x1F) - 24;
         data.voltage = raw * powf(2.0f, (float)exponent);
         break;
       }
     }
     HAL_Delay(10);
   }
   
   // ????
   retry = MAX_RETRIES;
   while(retry--)
   {
     if(PMBus_ReadWord(devAddr, PMBUS_CMD_READ_IOUT, &raw) == HAL_OK)
     {
       if(mode & 0x40) {
         int exponent = (int8_t)(mode & 0x1F) - 24;
         data.current = raw * powf(2.0f, (float)exponent);
         break;
       }
     }
     HAL_Delay(10);
   }
   
   // ????
   if(data.voltage > 0 && data.current > 0) {
     data.power = data.voltage * data.current;
     data.valid = 1;
   }
   
   return data;
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("System Initialized\r\n");
  PMBus_ScanDevices();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* ?????? */
    PowerData_t power = PMBus_ReadPowerData(PMBUS_ADDR);
    
    if(power.valid) {
      DEBUG_PRINT("V: %.2fV, I: %.3fA, P: %.2fW\r\n", 
                 power.voltage, power.current, power.power);
    } else {
      DEBUG_PRINT("Power data read failed\r\n");
    }

    /* LED???????? */
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    
    HAL_Delay(1000);
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
