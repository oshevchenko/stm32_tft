/**
  ******************************************************************************
  * File Name          : speedometer.c
  * Description        : Calculate speed by measuring the time between encoder
  *                      signals pulses.
  ******************************************************************************

  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "speedometer.h"
#include "stm32f1xx_hal.h"
#include "event_queue.h"
#include "main_hal.h"

//static int16_t speed_left = 0;
//static int16_t speed_right = 0;

static speed_sm_s speed_meter_right = {
                                       .speed = 0,
                                       .old_encoder = 0
                                      };
static speed_sm_s speed_meter_left  = {
                                       .speed = 0,
                                       .old_encoder = 0
                                      };

void speed_calc(uint16_t encoder, speed_sm_s *meter)
{
	meter->speed  = (int16_t)encoder - (int16_t)meter->old_encoder;
	meter->old_encoder = encoder;
}


void SPEED_Task(void)
{
//	uint16_t encoder;
//	encoder = TIMER_HAL_GetEncoder_Y();
//	speed_calc(encoder, &speed_meter_right);
//	encoder = TIMER_HAL_GetEncoder_X();
//	speed_calc(encoder, &speed_meter_left);
}

void SPEED_GetSpeed(int16_t *p_speed_left, int16_t *p_speed_right)
{
	uint16_t encoder;
	encoder = TIMER_HAL_GetEncoder_Y();
	speed_calc(encoder, &speed_meter_right);
	encoder = TIMER_HAL_GetEncoder_X();
	speed_calc(encoder, &speed_meter_left);

	*p_speed_left = speed_meter_left.speed;
	*p_speed_right = speed_meter_right.speed;
}

/******END OF FILE****/

