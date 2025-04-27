/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "voltage.h"
#include "shtc3.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#pragma pack(1)
typedef struct {
    uint8_t Leak;
    float   Voltage;
    float   Temp;
    float   Hum;
    uint8_t   State; // 向agx发送电磁铁状态
    float   angle[2]; // 向agx发送舵机状态
} Status;

uint8_t uart_buff[256];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_UP 2500.0
#define SERVO_DOWN 500.0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Status DeviceStatus;

uint8_t dvlstate = 0;

float servo[2] = {0.5, 0.0};
uint8_t magnet_state = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t uart_buff[256];
uint8_t rec_count = 0;

/*
 * 函数�??: dvl_shutdown
 * 描述  : 关闭Dvl
 * 输入  : /
 * 输出  : /
 * 备注  : /
 */
void dvl_shutdown(UART_HandleTypeDef *huart)
{
    uint8_t BUFDVLSHUTDOWN[14] = {0xFC, 0xCF, 0x03, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x30, 0xFD, 0xDF};
    HAL_UART_Transmit(huart, BUFDVLSHUTDOWN, 14, 10); // 关闭DVL
}

/*
 * 函数�??: dvl_startup
 * 描述  : �??启Dvl
 * 输入  : /
 * 输出  : /
 * 备注  : /
 */
void dvl_startup(UART_HandleTypeDef *huart)
{
    uint8_t BUFDVLPOWERON[14] = {0xFC, 0xCF, 0x03, 0x01, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00, 0x31, 0xFD, 0xDF};
    HAL_UART_Transmit(huart, BUFDVLPOWERON, 14, 10); // 关闭DVL
}


// 下行数据
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    float Servo_angle;
    if (huart == &huart4)
    {
        if (uart_buff[rec_count - 1] == 0xfa && uart_buff[rec_count] == 0xaf && rec_count > 1)
        {
            rec_count    = 1;
            uart_buff[0] = 0xfa;
            uart_buff[1] = 0xaf;
        }
        if (uart_buff[rec_count - 1] == 0xfb && uart_buff[rec_count] == 0xbf && rec_count > 0)
        {
            if (uart_buff[2] == 0x06)
            {
                memcpy(&Servo_angle , uart_buff + 4, 4);
                if(uart_buff[3] == 0x00)
                    servo[0] = Servo_angle;
                if(uart_buff[3] == 0x01)
                    servo[1] = Servo_angle;
            }
            else if (uart_buff[2] == 0x09)
            {
                if (uart_buff[3] == 0x00)
                    magnet_state = 0;
                else
                    magnet_state = 1;              
            }
            else if (uart_buff[2] == 0x04)
            {
                if (uart_buff[3] == 1) { dvl_startup(&huart5); }
                if (uart_buff[3] == 0) { dvl_shutdown(&huart5); }
            }

            rec_count = 200;
        }
        if (rec_count > 100)
            rec_count = 0;
        else
            rec_count++;

        __HAL_UART_CLEAR_OREFLAG(&huart4);
        huart4.RxState = HAL_UART_STATE_READY;
        huart4.Lock    = HAL_UNLOCKED;

        HAL_UART_Receive_IT(&huart4, uart_buff + rec_count, 1);
    }
}

uint8_t WaterCheck(void) // 漏水
{
    static uint8_t       counter;
    static GPIO_PinState leak;

    leak = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);

    if (leak == GPIO_PIN_RESET)
    {
        counter++;
        if (counter >= 20)
            return 0x01;
        else
            return 0x00;
    }
    else
    {
        counter = 0;
        return 0x00;
    }
}

/*
 * 函数�??: imu_setmode
 * 描述  : 设置室内导航模式
 * 输入  : /
 * 输出  : /
 * 备注  : /
 */
void imu_setmode(UART_HandleTypeDef *huart)
{
    uint8_t BUFIMUSETMODE[14] = {0xFC, 0xCF, 0x02, 0x00, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00, 0x31, 0xFD, 0xDF};
    HAL_UART_Transmit(huart, BUFIMUSETMODE, 14, 10); // 设置室内导航模式
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
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_TIM4_Init();
    MX_UART4_Init();
    MX_UART5_Init();
    /* USER CODE BEGIN 2 */
    // 通信
    HAL_UART_Receive_IT(&huart4, (uint8_t *)&uart_buff, 1);
    Dev_Shtc3_init();

    // 电磁铁初始化
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
    //舵机初始化
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 1500);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

    uint8_t life      = 0;
    uint8_t Buff[256] = {0}; // 上行数据缓冲�??

    Buff[0]  = 0xfa;         // 帧头
    Buff[1]  = 0xaf;
    Buff[2]  = 0X01;         // 设备标识�??
    Buff[25] = 0xfb;         // 帧尾
    Buff[26] = 0xbf;


    uint8_t vol_1 = 0, cnt = 0;
    uint32_t volmax = 0;

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while(1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        // DVL Manage

        Dev_Shtc3_Refresh();
        DeviceStatus.Temp = SHTC3_Data.temp; // 写入温度
        DeviceStatus.Hum  = SHTC3_Data.hum;  // 写入湿度

        vol_1 = Dev_Voltage_Refresh(hadc1, 3, 4);

        if (volmax < vol_1) volmax = vol_1;
        cnt++;
        if (cnt == 200)
        {
            DeviceStatus.Voltage = volmax*10000; //为了避免c到python的精度损失，这里乘以了10000。
            volmax = cnt = 0;
        }

        //舵机控制
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 
                             (int)(servo[0] * (SERVO_UP - SERVO_DOWN) + SERVO_DOWN));

        //电磁铁控制
        if (magnet_state == 0)
            __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
        else
            __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 20000);

        DeviceStatus.State = magnet_state;
        DeviceStatus.angle[0] = servo[0];
        DeviceStatus.angle[1] = servo[1];

        DeviceStatus.Leak = WaterCheck();

        // 数据上行
        memcpy(Buff + 3, &DeviceStatus, 22);
        HAL_UART_Transmit(&huart4, Buff, 27, 10);


        if (life)
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
        else
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
        life = !life;
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
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 8;
    RCC_OscInitStruct.PLL.PLLN            = 168;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) { Error_Handler(); }
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
    while (1) {}
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
