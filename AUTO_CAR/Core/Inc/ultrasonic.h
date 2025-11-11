/*
 * ultrasonic.h
 *
 *  Created on: Nov 10, 2025
 *      Author: user8
 */
// #include  "main.h"
#include "stm32f4xx_hal.h"

#include  "tim.h"

#ifndef INC_ULTRASONIC_H_
#define INC_ULTRASONIC_H_

#define TRIG_CENTER_PORT   GPIOA
#define TRIG_CENTER_PIN   GPIO_PIN_5

#define TRIG_LEFT_PORT   GPIOB
#define TRIG_LEFT_PIN   GPIO_PIN_6

#define TRIG_RIGHT_PORT   GPIOA
#define TRIG_RIGHT_PIN   GPIO_PIN_4

extern uint16_t IC_Left1, IC_Left2;
extern uint16_t IC_Center1, IC_Center2;
extern uint16_t IC_Right1, IC_Right2;

extern uint8_t captureLeftFlag, captureCenterFlag, captureRightFlag;

extern uint16_t echoTimeLeft, echoTimeCenter, echoTimeRight;
extern uint8_t distance_Left, distance, distance_Right;

void HCSR04_TRIGGER(GPIO_TypeDef* port, uint16_t pin);


#endif /* INC_ULTRASONIC_H_ */


