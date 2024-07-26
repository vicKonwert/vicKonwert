/*
 * timestamp.h
 *
 *  Created on: Jun 17, 2024
 *      Author: vichu
 */

#ifndef INC_TIMESTAMP_H_
#define INC_TIMESTAMP_H_

#include "stm32f4xx_hal.h"
#include "main.h"

//extern void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void Start_Stopwatch(void);

void Stop_Stopwatch(void);

void Reset_Stopwatch(void);

//void Display_Time(void);

#endif /* INC_TIMESTAMP_H_ */
