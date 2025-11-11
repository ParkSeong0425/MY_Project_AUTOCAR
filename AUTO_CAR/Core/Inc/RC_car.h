/*
 * RC_car.h
 *
 *  Created on: Nov 10, 2025
 *      Author: user8
 */
#ifndef INC_RC_CAR_H_
#define INC_RC_CAR_H_

#include "stm32f4xx_hal.h"
#include "tim.h"

/* --- 타이밍 변수 (extern 선언만) --- */
extern uint32_t previous_trigger_time;
extern uint32_t now;
extern uint8_t sensor_state;
extern uint32_t lastDecisionTime;

void Car_Forward(void);
void Car_Backward(void);
void Car_Left(void);
void Car_Right(void);
void Car_Stop(void);

#endif /* INC_RC_CAR_H_ */
