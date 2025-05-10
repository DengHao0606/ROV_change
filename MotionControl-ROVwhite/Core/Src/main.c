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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "comm.h"
#include "thrust.h"
#include "ms5837.h"
#include "usart.h"
#include "filter.h"

#include "autocontrol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define thrust_mean_filter_length 20

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
CoordinateSystems robot;
CoordinateSystems target;

float Vx = 0, Vy = 0, Ry = 0, Tx = 0, Ty = 0, Tz = 0, Mx = 0, My = 0, Mz = 0;


CoordinateSystems robot_pos; // çĺŽçćşĺ¨äşş

CoordinateSystems robot_im_pos;    // ĺçć°´ĺšłçćşĺ¨äşş,ç¨äşčŽĄçŽćşĺ¨äşşć°´ĺšłä˝ç˝ŽčŻŻďż??
CoordinateSystems robot_im_spd;    // éĺşŚçŠşé´ä¸­çćşĺ¨ďż??,ç¨äşčŽĄçŽćşĺ¨äşşć°´ĺšłć¨Şç§ťďż˝?ďż˝ĺčżďż˝?ďż˝ĺşŚ
CoordinateSystems robot_im_thrust; // ć¨ĺçŠşé´ä¸­ççćşĺ¨äşş,ç¨äşčŽĄçŽć¨ĺ
CoordinateVector  required_thrust = {0, 0, 0, 0, 0, 0};


RobotController robot_controller;
float           openloop_thrust[6] = {0}; // 0~5 correspond x y z rx ry rz

MeanFilter meanfilter[6];

int led_motion   = 0;
int led_dataup   = 0;
int led_uart4    = 0;
int led_uart7    = 0;
int led_main     = 0;
int led_watchdog = 0;
int led_ms5837   = 0;

int threadmonitor_tim2  = 30;
int threadmonitor_tim3  = 30;
int threadmonitor_uart4 = 300;
int threadmonitor_uart7 = 300;

int start = 0;

uint8_t transbuf[157] = {0};

float measureddepth = 0;
float realdepth     = 0;
float startdepth    = 0;
float checkeddepth  = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * ĺ˝ć°ďż????????: HAL_TIM_PeriodElapsedCallback
 * ćčż°  : ĺŽćśĺ¨ä¸­ć­ĺ¤ďż????????
 * čžĺĽ  : TIM_HandleTypeDef *htim ĺŽćśĺ¨ĺ°ďż????????
 * čžĺş  : /
 * ĺ¤ćł¨  : ç¨äşĺ¤çć°ćŽ
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static float motorthrust[6]        = {0}; // 0~5 correspond motor0~5
    static float askedthrust[6]        = {0};
    static float motorthrust_filted[6] = {0};


    //  1ĺˇĺŽćśĺ¨ä¸­ć­
    //  é˘ç 20hz
    if (htim == (&htim1))
    { // watch dog
        threadmonitor_tim2--;
        threadmonitor_tim3--;
        threadmonitor_uart4--;
        threadmonitor_uart7--;

        if (threadmonitor_tim2 <= 0)
        {
            __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
            HAL_TIM_Base_Start_IT(&htim2);
            threadmonitor_tim2 = 30;
        }
        if (threadmonitor_tim3 <= 0)
        {
            __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
            HAL_TIM_Base_Start_IT(&htim3);
            threadmonitor_tim3 = 30;
        }
        // if (threadmonitor_uart4 <= 0)
        // {
        //     HAL_UART_Receive_IT(&huart4, uart4rec.buf, 1);
        //     __HAL_UART_CLEAR_OREFLAG(&huart4);
        //     threadmonitor_uart4 = 30;
        // }
        if (threadmonitor_uart7 <= 0)
        {
            MX_UART7_Init();
            HAL_UART_Receive_IT(&huart7, uart7rec.buf + uart7rec.cnt, 1);
            threadmonitor_uart7 = 50;
        }
        // led
        if (led_watchdog)
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
        led_watchdog = !led_watchdog;
    }
    //  2ĺˇĺŽćśĺ¨ä¸­ć­
    //  é˘ç 16hz
    else if (htim == (&htim2))
    {
        threadmonitor_tim3 = 20;

        memcpy(transbuf + 3, &(robot.base.vector), 24);
        // memcpy(transbuf + 27, &(robot.target_inbase.vector), 24);
        memcpy(transbuf + 27, &(robot_im_pos.target_inbase.vector), 12);
        memcpy(transbuf + 39, (uint8_t *)&(robot_pos.target_inbase.vector.rx), 12);

        memcpy(transbuf + 51, &(robot.target_inworld.vector), 24);
        memcpy(transbuf + 75, &(robot_controller.state), 6);
        memcpy(transbuf + 81, &(imu), 50);
        memcpy(transbuf + 131, &(motorthrust_filted), 24);

        // __HAL_UNLOCK(&huart4);
        HAL_UART_Transmit(&huart4, transbuf, 157, 10);

        // led
        if (led_dataup)
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
        led_dataup = !led_dataup;
    }
    //  3ĺˇĺŽćśĺ¨ä¸­ć­
    //  é˘ç 40hz
    else if (htim == (&htim3))
    {
        // threadmonitor_tim2 = 20;

        // // Cs data refresh
        robot.base.vector.x  = imu.pos.x;
        robot.base.vector.y  = imu.pos.y;
        robot.base.vector.z  = imu.pos.z;
        robot.base.vector.rx = imu.pos.rx;
        robot.base.vector.ry = imu.pos.ry;
        robot.base.vector.rz = imu.pos.rz;

        // // refresh CoordinateSystems
        // robot.base.extract(&(robot.base));

        // // Cs transform
        // robot.world2base(&robot);

        // čŽĄçŽĺć§ĺśĺ¨ďż??ďż??ćľéĺźä¸čŻŻĺˇŽĺźDF
        // ćşĺ¨äşşĺďż??
        robot_im_pos.base.vector.x = robot_pos.base.vector.x = imu.pos.x;
        robot_im_pos.base.vector.y = robot_pos.base.vector.y = imu.pos.y;
        robot_im_pos.base.vector.z = robot_pos.base.vector.z = imu.pos.z;

        // çŽć ä˝ç˝Ž
        robot_im_pos.target_inworld.vector.x = robot_pos.target_inworld.vector.x = robot.target_inworld.vector.x;
        robot_im_pos.target_inworld.vector.y = robot_pos.target_inworld.vector.y = robot.target_inworld.vector.y;
        robot_im_pos.target_inworld.vector.z = robot_pos.target_inworld.vector.z = robot.target_inworld.vector.z;
        robot_im_pos.target_inworld.vector.rz = robot_pos.target_inworld.vector.rz = robot.target_inworld.vector.rz;
        robot_im_pos.target_inworld.extract(&(robot_im_pos.target_inworld));

        // ĺ§żďż˝??
        robot_pos.base.vector.rx = imu.pos.rx;
        robot_pos.base.vector.ry = imu.pos.ry;
        robot_pos.base.vector.rz = imu.pos.rz;
        robot_pos.base.extract(&(robot_pos.base));

        robot_im_spd.base.vector.rx = robot_im_thrust.base.vector.rx = robot_pos.base.vector.rx;
        robot_im_spd.base.vector.ry = robot_im_thrust.base.vector.ry = robot_pos.base.vector.ry;
        robot_im_pos.base.vector.rz                                  = robot_pos.base.vector.rz;


        // ĺŻźĺşĺĺčçłťĺć˘ĺ¨ĺŻščąĄçĺşĺşçŠéľ
        robot_im_pos.base.extract(&(robot_im_pos.base));
        robot_im_spd.base.extract(&(robot_im_spd.base));
        robot_im_thrust.base.extract(&(robot_im_thrust.base));

        // čŽĄçŽćşĺ¨äşşçć°´ĺšłć¨Şç§ťä¸ĺčżďż˝?ďż˝ĺşŚ
        robot_im_spd.target_inbase.vector.x = imu.spd.x;
        robot_im_spd.target_inbase.vector.y = imu.spd.y;
        robot_im_spd.target_inbase.vector.z = imu.spd.z;
        robot_im_spd.target_inbase.extract(&(robot_im_spd.target_inbase));
        robot_im_spd.base2world(&robot_im_spd);

        // čŽĄçŽćşĺ¨äşşĺčçłťä¸­çć°´ĺšłčŻŻĺˇŽďźć¨Şĺä¸ĺĺďż??
        robot_im_pos.world2base(&robot_im_pos);


        // Pid controller refresh
        if (robot_controller.state[0] == 1) // x
            Tx = robot_controller.x.refresh(&robot_controller.x, robot_im_pos.target_inbase.vector.x);
        else
            Tx = openloop_thrust[0];

        if (robot_controller.state[1] == 1) // y
            Ty = robot_controller.y.refresh(&robot_controller.y, robot_im_pos.target_inbase.vector.y);
        else
            Ty = openloop_thrust[1];

        if (robot_controller.state[2] == 1) // z
            Tz = robot_controller.z.refresh(&robot_controller.z, robot_im_pos.target_inbase.vector.z);
        else
            Tz = openloop_thrust[2];

        // ĺ§żďż˝?ďż˝ć§ĺśĺ¨
        robot_pos.target_inworld.vector.ry = Ry;
        robot_pos.target_inworld.extract(&(robot_pos.target_inworld));
        robot_pos.world2base(&robot_pos);


        if (robot_controller.state[3] == 1) // rx
            //Mx = robot_controller.rx.refresh(&robot_controller.rx, AngleCorrect(robot_pos.target_inbase.vector.rx));
            Mx = 0;
        else
            //Mx = openloop_thrust[3];
            Mx = 0;
        if (robot_controller.state[4] == 1) // ry
            My = robot_controller.ry.refresh(&robot_controller.ry, AngleCorrect(robot_pos.target_inbase.vector.ry));
        else
            My = openloop_thrust[4];

        if (robot_controller.state[5] == 1) // rz
            Mz = robot_controller.rz.refresh(&robot_controller.rz, AngleCorrect(robot_pos.target_inbase.vector.rz));
        else
            Mz = openloop_thrust[5];


        // ć¨ĺć˘çŽ
        // Tx = NEGATIVE_BUOYANCY * tan(robot_pos.base.vector.ry * deg2rad);

        robot_im_thrust.target_inworld.vector.x = Tx;
        robot_im_thrust.target_inworld.vector.y = Ty;
        robot_im_thrust.target_inworld.vector.z = Tz;
        robot_im_thrust.target_inworld.extract(&(robot_im_thrust.target_inworld));
        robot_im_thrust.world2base(&robot_im_thrust);

        askedthrust[0] = robot_im_thrust.target_inbase.vector.x;
        askedthrust[1] = robot_im_thrust.target_inbase.vector.y;
        askedthrust[2] = robot_im_thrust.target_inbase.vector.z;

        robot_im_thrust.target_inworld.vector.x = Mx;
        robot_im_thrust.target_inworld.vector.y = My;
        robot_im_thrust.target_inworld.vector.z = Mz;
        robot_im_thrust.target_inworld.extract(&(robot_im_thrust.target_inworld));
        robot_im_thrust.world2base(&robot_im_thrust);

        askedthrust[3] = robot_im_thrust.target_inbase.vector.x;
        askedthrust[4] = robot_im_thrust.target_inbase.vector.y;
        askedthrust[5] = robot_im_thrust.target_inbase.vector.z;


        // allocate thrust
        ThrustAllocate(askedthrust, motorthrust);

        // thrust filter
        // for (int i = 0; i < 6; i++) { motorthrust_filted[i] = meanfilter[i].refresh(&(meanfilter[i]), motorthrust[i]); }
        for (int i = 0; i < 6; i++) { motorthrust_filted[i] = motorthrust[i]; }

        // Convert thrust signal to PWM signal
        MotorPwmRefresh(motorthrust_filted);

        // led
        if (led_motion)
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
        led_motion = !led_motion;
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
  MX_TIM5_Init();
  MX_UART4_Init();
  MX_UART7_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
    // uart it start
    CommInit();
    Ms5837Init(&hi2c1);

    // LED init
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

    // motor init
    MotorInit();

    // Data init
    CoordinateSystems_Init(&robot);  // Robot CS
    CoordinateSystems_Init(&target); // target CS

    // ćşĺ¨äşşćčż°çŠéľĺĺ§ĺ
    CoordinateSystems_Init(&robot_pos);
    CoordinateSystems_Init(&robot_im_pos);
    CoordinateSystems_Init(&robot_im_spd);
    CoordinateSystems_Init(&robot_im_thrust);


    robot_controller.state[0] = robot_controller.state[1] = robot_controller.state[2] = 0;
    robot_controller.state[3] = robot_controller.state[4] = robot_controller.state[5] = 0;
    PositionalPID_Init(&(robot_controller.x), 0, 0, 0, 0, 0);  // X pid controller
    PositionalPID_Init(&(robot_controller.y), 0, 0, 0, 0, 0);  // y pid controller
    PositionalPID_Init(&(robot_controller.z), 0, 0, 0, 0, 0);  // z pid controller
    PositionalPID_Init(&(robot_controller.rx), 0, 0, 0, 0, 0); // pitch pid controller
    PositionalPID_Init(&(robot_controller.ry), 0, 0, 0, 0, 0); // roll pid controller
    PositionalPID_Init(&(robot_controller.rz), 0, 0, 0, 0, 0); // yaw pid controller
    transbuf[0]   = 0xfa;
    transbuf[1]   = 0xaf;
    transbuf[2]   = 0x00;
    transbuf[155] = 0xfb;
    transbuf[156] = 0xbf;

    MeanFilter_Init(&(meanfilter[0]), thrust_mean_filter_length);
    MeanFilter_Init(&(meanfilter[1]), thrust_mean_filter_length);
    MeanFilter_Init(&(meanfilter[2]), thrust_mean_filter_length);
    MeanFilter_Init(&(meanfilter[3]), thrust_mean_filter_length);
    MeanFilter_Init(&(meanfilter[4]), thrust_mean_filter_length);
    MeanFilter_Init(&(meanfilter[5]), thrust_mean_filter_length);

    ThrustCurveInit(&(thrustcurve[0]));
    ThrustCurveInit(&(thrustcurve[1]));
    ThrustCurveInit(&(thrustcurve[2]));
    ThrustCurveInit(&(thrustcurve[3]));
    ThrustCurveInit(&(thrustcurve[4]));
    ThrustCurveInit(&(thrustcurve[5]));

    // start data process
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim3);

    // HAL_Delay(1000);
    // HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    Ms5837Read(&hi2c1);
    Ms5837Depth(&startdepth);
    checkeddepth = startdepth;

    while (1)
    {
        Ms5837Read(&hi2c1);
        Ms5837Depth(&measureddepth);
        realdepth = measureddepth - startdepth;
        KalmanFilter(realdepth, &checkeddepth);
        imu.pos.z = checkeddepth;


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        // led
        if (led_main)
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
        led_main = !led_main;

        //HAL_Delay(50);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

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
    while (1) {}
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
