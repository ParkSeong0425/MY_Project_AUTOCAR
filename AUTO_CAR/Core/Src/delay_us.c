/*
 * delay_us.c
 *
 *  Created on: Oct 16, 2025
 *      Author: user8
 */

#include "delay_us.h"


void delay_us(uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim11, 0); // 0개부터 카운트
  while((__HAL_TIM_GET_COUNTER(&htim11)) < us);
}


