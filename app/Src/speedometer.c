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
#include "logger.h"
#include "stm32_microrl_misc.h"

#define SPEED_TIMEOUT_N 2

static int16_t speed_left = 0;
static int16_t speed_right = 0;
static int16_t speed_right_f = 0;
static int16_t speed_right_max=0;
static uint16_t speed_max;
static uint16_t pulse_raw;
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
#define SPEED_TIME_MAX 10000

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
//	time_raw = time;
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

#define FILTER_SIZE 2
typedef struct __SPEEDOMETER_FILTER_STRUCT {
	uint16_t fbuf[FILTER_SIZE];
} speed_filter_s;
static speed_filter_s filter_left;
static speed_filter_s filter_right;
static uint16_t fbuf_x[FILTER_SIZE];
uint16_t filter(uint16_t in, speed_filter_s *p_filter)
{
	uint8_t cnt = 0;
	uint32_t total = 0;

	for (cnt = 0; cnt < FILTER_SIZE-1; cnt++) {
		p_filter->fbuf[cnt] = p_filter->fbuf[cnt+1];
	}
	p_filter->fbuf[FILTER_SIZE-1] = in;

	for (cnt = 0; cnt < FILTER_SIZE; cnt++) {
		total = total + p_filter->fbuf[cnt];
	}

	total = total / FILTER_SIZE;
	total = total / 50;
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
//		speed_left = filter(0, &filter_left);
	}

	if (speed_meter_right.timeout_counter < SPEED_TIMEOUT_N) speed_meter_right.timeout_counter++;
	if (SPEED_TIMEOUT_N == speed_meter_right.timeout_counter)  {
		speed_right = filter(0, &filter_right);
	}
}
void SPEED_Task(void)
{
	uint16_t speed;
	speed = SPEED_TIME_MAX;
	TIMER_HAL_DisableIrq_L();
	if (SPEED_TRIGGERED == speed_meter_left.state){
		if (SPEED_TIMEOUT_N != speed_meter_left.timeout_counter) {
			speed = speed_meter_left.speed;
		}
		speed_meter_left.timeout_counter = 0;
		speed_meter_left.state = SPEED_IDLE;
		TIMER_HAL_EnableIrq_L();
//		if (speed > SPEED_TIME_MAX) speed = SPEED_TIME_MAX;
//		speed = SPEED_TIME_MAX - speed;
//		speed_left = filter(speed, &filter_left);
	} else {
		TIMER_HAL_EnableIrq_L();
	}
	speed = SPEED_TIME_MAX;
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

	if (speed_right > speed_max) speed_max = speed_right;

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
typedef struct speed_logger_data
{
	int16_t speed_enc;
	int16_t speed_pulse;
} st_speed_logger_data;
st_speed_logger_data speed_data[LOGGER_ARRAY_SIZE];
st_logger_data speed_log_data[LOGGER_ARRAY_SIZE];
static st_logger_module speed_log_module;

//typedef struct logger_data
//{
//	void *p_data;
//    int (*sprintfDataCallback)(char*, void*, int);
//	struct list_head list;
//} st_logger_data;

int SpeedSnprintfDataCallback(char* str, void *p_dat, int str_max)
{
	st_speed_logger_data *p_data;
	p_data = (st_speed_logger_data *) p_dat;
	snprintf(str, str_max, "e:%d p:%d", p_data->speed_enc, p_data->speed_pulse);
	return 0;
}

st_logger_data* SpeedGetLogDataCallback()
{
	static int i = 0;

	i++;
	if (LOGGER_ARRAY_SIZE == i) i = 0;
	speed_data[i].speed_enc = speed_meter_right.delta;
	speed_data[i].speed_pulse = speed_right;
	return &speed_log_data[i];
}
void SPEED_Init()
{
	int i;
	for (i = 0; i < LOGGER_ARRAY_SIZE; i++) {
		speed_log_data[i].p_data = &speed_data[i];
		speed_log_data[i].sprintfDataCallback = SpeedSnprintfDataCallback;
	}
	speed_log_module.getLogDataCallback = SpeedGetLogDataCallback;
	LOGGER_RegisterModule(&speed_log_module);
}

#define SPEED_STAT_LEN 50
void SPEED_PrintStat()
{
	char cmd_buf[SPEED_STAT_LEN];
	snprintf (cmd_buf, SPEED_STAT_LEN, "speed_right_max=%05d d=%d speed_max=%d %d\n\r", speed_right_max, speed_meter_right.delta, speed_max, pulse_raw);
	speed_max = 0;
	print (cmd_buf);
}

/******END OF FILE****/

