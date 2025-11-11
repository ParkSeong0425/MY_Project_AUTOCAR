/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "delay_us.h"
#include "ultrasonic.h"
#include "RC_car.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"
#include <stdio.h>
#include <stdlib.h>  // <-- abs() 사용하려면 추가

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for ultrasonic_TASK */
osThreadId_t ultrasonic_TASKHandle;
const osThreadAttr_t ultrasonic_TASK_attributes = {
  .name = "ultrasonic_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RC_car_TASK */
osThreadId_t RC_car_TASKHandle;
const osThreadAttr_t RC_car_TASK_attributes = {
  .name = "RC_car_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ultrasonic(void *argument);
void RC_Car(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* 외부 변수 선언 */
extern uint8_t distance_Left;
extern uint8_t distance;
extern uint8_t distance_Right;

/* 외부 함수 선언 */
extern void HCSR04_TRIGGER(GPIO_TypeDef* port, uint16_t pin);
extern void Car_Forward(void);
extern void Car_Backward(void);
extern void Car_Left(void);
extern void Car_Right(void);
extern void Car_Stop(void);

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ultrasonic_TASK */
  ultrasonic_TASKHandle = osThreadNew(ultrasonic, NULL, &ultrasonic_TASK_attributes);

  /* creation of RC_car_TASK */
  RC_car_TASKHandle = osThreadNew(RC_Car, NULL, &RC_car_TASK_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_ultrasonic */
/**
  * @brief  Function implementing the ultrasonic_TASK thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ultrasonic */
void ultrasonic(void *argument)
{
  /* USER CODE BEGIN ultrasonic */
  /* Infinite loop */
  for(;;)
  {
      static uint8_t sensor_state = 0;

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

      vTaskDelay(pdMS_TO_TICKS(50)); // 50ms 주기
    }
    /* USER CODE END ultrasonic */
  }

/* USER CODE BEGIN Header_RC_Car */
/**
* @brief Function implementing the RC_car_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RC_Car */
void RC_Car(void *argument)
{
  /* USER CODE BEGIN RC_Car */
  /* Infinite loop */
  for(;;)
  {
         printf("L:%d C:%d R:%d\r\n", distance_Left, distance, distance_Right);

         // 기본 전진
         Car_Forward();
         TIM4->CCR2 = 900; // 왼쪽 모터 속도
         TIM4->CCR3 = 900; // 오른쪽 모터 속도

         if(distance < 40) // 장애물이 어느 정도 가까워지면 회피 판단
         {
             // 한 주기 내에서 좌우 판단을 2~3번 반복
             for(int i = 0; i < 3; i++)
             {
                 // 좌우 센서 차이에 따른 회전
                 if(distance_Left > (distance_Right + 10))
                 {
                     TIM4->CCR2 = 300;
                     TIM4->CCR3 = 900;
                     Car_Left();
                 }
                 else if(distance_Right > (distance_Left + 10))
                 {
                     TIM4->CCR2 = 900;
                     TIM4->CCR3 = 300;
                     Car_Right();
                 }
                 // 가까이 있는 장애물 처리
                 else if(distance_Left <= 15)  // 왼쪽 장애물
                 {
                     TIM4->CCR2 = 900;
                     TIM4->CCR3 = 300;
                     Car_Right();
                 }
                 else if(distance_Right <= 15) // 오른쪽 장애물
                 {
                     TIM4->CCR2 = 300;
                     TIM4->CCR3 = 900;
                     Car_Left();
                 }
                 // 짧은 지연으로 센서 안정화
                 vTaskDelay(pdMS_TO_TICKS(20));
             }
         }

         // 주기 지연
         vTaskDelay(pdMS_TO_TICKS(80));
     }
 }


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

