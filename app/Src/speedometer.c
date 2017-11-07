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

#define SPEED_TIMEOUT_N 2

static int16_t speed_left = 0;
static int16_t speed_right = 0;
static int16_t speed_right_max=0;

static speed_sm_s speed_meter_right = {.state = SPEED_IDLE,
                                       .speed = 0,
                                       .old_encoder = 0,
                                       .timeout_counter = 0
                                      };
static speed_sm_s speed_meter_left  = {.state = SPEED_IDLE,
                                       .speed = 0,
                                       .old_encoder = 0,
                                       .timeout_counter = 0
                                      };
#define SPEED_TIME_MAX 2104

void speed_calc(uint16_t time, 	speed_sm_s *meter)
{
	meter->speed = time;
	meter->state = SPEED_TRIGGERED;
}
//Should be called from HW timer interrupt
void SPEED_RightTick(void)
{
	uint16_t time;
	time = TIMER_HAL_GetSpeedometerTime_R();
	speed_meter_right.speed = time;
	speed_meter_right.state = SPEED_TRIGGERED;
}

//Should be called from HW timer interrupt
void SPEED_LeftTick(void)
{
	uint16_t time;
	time = TIMER_HAL_GetSpeedometerTime_L();
	speed_meter_left.speed = time;
	speed_meter_left.state = SPEED_TRIGGERED;
}

#define FILTER_SIZE 4
typedef struct __SPEEDOMETER_FILTER_STRUCT {
	uint16_t fbuf[FILTER_SIZE];
} speed_filter_s;
static speed_filter_s filter_left;
static speed_filter_s filter_right;
uint16_t filter(uint16_t in, speed_filter_s *s_buf)
{
	uint8_t cnt = 0;
	uint32_t total = 0;
	s_buf->fbuf[0] = s_buf->fbuf[1];
	s_buf->fbuf[1] = s_buf->fbuf[2];
	s_buf->fbuf[2] = s_buf->fbuf[3];
	s_buf->fbuf[3] = in;

	for (cnt = 0; cnt < FILTER_SIZE; cnt++) {
		total = total + s_buf->fbuf[cnt];
	}

	total /= FILTER_SIZE;
	return (uint16_t) total;
}
void reset_filter(speed_filter_s *s_buf)
{
	uint8_t cnt = 0;

	for (cnt = 0; cnt < FILTER_SIZE; cnt++) {
		s_buf->fbuf[cnt] = 0;
	}

}


void SPEED_Timeout(void)
{
	if (speed_meter_left.timeout_counter < SPEED_TIMEOUT_N) speed_meter_left.timeout_counter++;
	if (SPEED_TIMEOUT_N == speed_meter_left.timeout_counter) {
		speed_left = filter(0, &filter_left);
	}

	if (speed_meter_right.timeout_counter < SPEED_TIMEOUT_N) speed_meter_right.timeout_counter++;
	if (SPEED_TIMEOUT_N == speed_meter_right.timeout_counter)  {
		speed_right = filter(0, &filter_right);
	}
}
void SPEED_Task(void)
{
	uint16_t speed;
	TIMER_HAL_DisableIrq_L();
	if (SPEED_TRIGGERED == speed_meter_left.state){
		if (SPEED_TIMEOUT_N != speed_meter_left.timeout_counter) {
			speed = speed_meter_left.speed;
		}
		speed_meter_left.timeout_counter = 0;
		speed_meter_left.state = SPEED_IDLE;
		TIMER_HAL_EnableIrq_L();
		if (speed > SPEED_TIME_MAX) speed = SPEED_TIME_MAX;
		speed = SPEED_TIME_MAX - speed;
		speed_left = filter(speed, &filter_left);
	} else {
		TIMER_HAL_EnableIrq_L();
	}

	TIMER_HAL_DisableIrq_R();
	if (SPEED_TRIGGERED == speed_meter_right.state){
		if (SPEED_TIMEOUT_N != speed_meter_right.timeout_counter) {
			speed = speed_meter_right.speed;
			if (speed > speed_right_max) speed_right_max = speed;
		}
		speed_meter_right.timeout_counter = 0;
		speed_meter_right.state = SPEED_IDLE;
		TIMER_HAL_EnableIrq_R();
		if (speed > SPEED_TIME_MAX) speed = SPEED_TIME_MAX;
		speed = SPEED_TIME_MAX - speed;
		speed_right = speed;
		speed_right = filter(speed, &filter_right);
	} else {
		TIMER_HAL_EnableIrq_R();
	}
}

void SPEED_GetSpeed(int16_t *p_speed_left, int16_t *p_speed_right)
{
	uint16_t encoder;
	encoder = TIMER_HAL_GetEncoder_Y();
	speed_meter_right.delta  = (int16_t)encoder - (int16_t)speed_meter_right.old_encoder;
	speed_meter_right.old_encoder = encoder;
	*p_speed_right = speed_meter_right.delta;

//	if (speed_meter_right.delta < 0) {
//		*p_speed_right = -((int16_t)speed_right);
//	} else {
//		*p_speed_right = (int16_t)speed_right;
//	}

	encoder = TIMER_HAL_GetEncoder_X();
	speed_meter_left.delta  = (int16_t)encoder - (int16_t)speed_meter_left.old_encoder;
	speed_meter_left.old_encoder = encoder;
	*p_speed_left = speed_meter_left.delta;
//	if (speed_meter_left.delta < 0) {
//		*p_speed_left = -((int16_t)speed_left);
//	} else {
//		*p_speed_left = (int16_t)speed_left;
//	}

}

#define SPEED_STAT_LEN 50
void SPEED_PrintStat()
{
	char cmd_buf[SPEED_STAT_LEN];
	snprintf (cmd_buf, SPEED_STAT_LEN, "speed_right_max=%05d delta=%d\n\r", speed_right_max, speed_meter_right.delta);
	print (cmd_buf);
}

/******END OF FILE****/

