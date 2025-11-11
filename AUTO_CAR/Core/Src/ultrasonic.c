/*
 * ultrasonic.c
 *
 *  Created on: Nov 10, 2025
 *      Author: user8
 */
#include "ultrasonic.h"
#include "delay_us.h"

uint16_t IC_Left1 = 0, IC_Left2 = 0;
uint16_t IC_Center1 = 0, IC_Center2 = 0;
uint16_t IC_Right1 = 0, IC_Right2 = 0;

uint8_t captureLeftFlag = 0;
uint8_t captureCenterFlag = 0;
uint8_t captureRightFlag = 0;

uint16_t echoTimeLeft = 0, echoTimeCenter = 0, echoTimeRight = 0;
uint8_t distance_Left = 0, distance = 0, distance_Right = 0;



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
