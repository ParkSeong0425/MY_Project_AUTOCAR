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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "delay_us.h"
#include  <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRIG_CENTER_PORT   GPIOA
#define TRIG_CENTER_PIN   GPIO_PIN_5

#define TRIG_LEFT_PORT   GPIOB
#define TRIG_LEFT_PIN   GPIO_PIN_6

#define TRIG_RIGHT_PORT   GPIOA
#define TRIG_RIGHT_PIN   GPIO_PIN_4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int _write(int file, unsigned char* p, int len)
{
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, p, len, 100);
    return (status == HAL_OK ? len : 0);
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t IC_Left1 = 0, IC_Left2 = 0;
uint16_t IC_Center1 = 0, IC_Center2 = 0;
uint16_t IC_Right1 = 0, IC_Right2 = 0;

uint8_t captureLeftFlag = 0;
uint8_t captureCenterFlag = 0;
uint8_t captureRightFlag = 0;

uint16_t echoTimeLeft = 0, echoTimeCenter = 0, echoTimeRight = 0;
uint8_t distance_Left = 0, distance = 0, distance_Right = 0;
uint8_t rxData;


/* --- 타이밍 변수 --- */
uint32_t previous_trigger_time = 0;
uint32_t now = 0;
uint8_t sensor_state = 0;
uint32_t lastDecisionTime = 0;   // 마지막 주행 판단 시각

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HCSR04_TRIGGER(GPIO_TypeDef* port, uint16_t pin);
void Car_Forward(void);
void Car_Backward(void);
void Car_Left(void);
void Car_Right(void);
void Car_Stop(void);

// 초음파 트리거
void HCSR04_TRIGGER(GPIO_TypeDef* port, uint16_t pin)
{
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
  delay_us(2);
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
  delay_us(10);
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);

}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* --- 캡처 콜백 --- */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
   /* --- CENTER --- */
   if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
   {
       if(captureCenterFlag == 0)
       {
           IC_Center1 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
           captureCenterFlag = 1;
           __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
       }
       else
       {
           IC_Center2 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
           __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
           echoTimeCenter = (IC_Center2 > IC_Center1) ? (IC_Center2 - IC_Center1) : (0xFFFF - IC_Center1 + IC_Center2);
           distance = echoTimeCenter / 58;
           captureCenterFlag = 0;
       }
   }

   /* --- LEFT --- */
   if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
   {
       if(captureLeftFlag == 0)
       {
           IC_Left1 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
           captureLeftFlag = 1;
           __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
       }
       else
       {
           IC_Left2 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
           __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
           echoTimeLeft = (IC_Left2 > IC_Left1) ? (IC_Left2 - IC_Left1) : (0xFFFF - IC_Left1 + IC_Left2);
           distance_Left = echoTimeLeft / 58;
           captureLeftFlag = 0;
       }
   }

   /* --- RIGHT --- */
   if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
   {
       if(captureRightFlag == 0)
       {
           IC_Right1 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);
           captureRightFlag = 1;
           __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
       }
       else
       {
           IC_Right2 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);
           __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
           echoTimeRight = (IC_Right2 > IC_Right1) ? (IC_Right2 - IC_Right1) : (0xFFFF - IC_Right1 + IC_Right2);
           distance_Right = echoTimeRight / 58;
           captureRightFlag = 0;
       }
   }
}




/* --- 모터 제어 함수 --- */
void Car_Forward()
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
}

void Car_Backward()
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
}

void Car_Left()
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
}

void Car_Right()
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
}

void Car_Stop()
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
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
  MX_TIM3_Init();
  MX_TIM11_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim11);   // for delay_us() function
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  HAL_UART_Receive_IT(&huart2, &rxData, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    now = HAL_GetTick();

        /* --- 50ms마다 초음파 트리거 --- */
        if (now - previous_trigger_time >= 50)
        {
            previous_trigger_time = now;

            if (sensor_state == 0)
            {
                HCSR04_TRIGGER(TRIG_LEFT_PORT, TRIG_LEFT_PIN);
                sensor_state = 1;
            }
            else if (sensor_state == 1)
            {
                HCSR04_TRIGGER(TRIG_CENTER_PORT, TRIG_CENTER_PIN);
                sensor_state = 2;
            }
            else
            {
                HCSR04_TRIGGER(TRIG_RIGHT_PORT, TRIG_RIGHT_PIN);
                sensor_state = 0;
            }
        }

        /* --- 100ms마다 주행 판단 --- */
        if (now - lastDecisionTime >= 100)
        {
            lastDecisionTime = now;

            printf("L:%d C:%d R:%d\r\n", distance_Left, distance, distance_Right);

            // 기본 속도
            TIM4->CCR2 = 900;
            TIM4->CCR3 = 900;

            if (distance < 40 )
            {

                // 좌우 비교
                if (distance_Left > distance_Right+10)
                {
                   TIM4->CCR2 = 500;
                   TIM4->CCR3 = 900;
                    Car_Left();
                    HAL_Delay(100); // 회전 시간
                    Car_Stop();
                }
                else if (distance_Right > distance_Left +10)
                {
                   TIM4->CCR2 = 900;
                   TIM4->CCR3 = 500;
                    Car_Right();
                    HAL_Delay(100); // 회전 시간
                    Car_Stop();
                }

            }

            else
            {
                // 장애물 없음 → 전진

                Car_Forward();

                if (distance < 10 || (distance_Left < 10 || distance_Right < 10)) // 부딫히면 안돼!
                {
                  TIM4->CCR2 = 500;
                  TIM4->CCR3 = 500;
                  Car_Backward();

                  if (distance_Left > distance_Right +10)
                  {
                     TIM4->CCR2 = 500;
                     TIM4->CCR3 = 900;
                      Car_Left();
                      HAL_Delay(100); // 회전 시간
                      Car_Stop();
                  }
                  else if (distance_Right > distance_Left +10)
                  {
                     TIM4->CCR2 = 900;
                     TIM4->CCR3 = 500;
                      Car_Right();
                      HAL_Delay(100); // 회전 시간
                      Car_Stop();
                  }
                }
            }
        }



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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
