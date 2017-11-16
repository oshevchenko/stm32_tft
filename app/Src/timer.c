/**
  ******************************************************************************
  * File Name          : timer.c
  * Description        : Simple timer.
  ******************************************************************************

  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "timer.h"
#include "stm32f1xx_hal.h"
//#include "stm32f4xx.h"
#include "event_queue.h"
#define TIMER_IRQn TIM6_IRQn
static tm_sm_s timer1;
static tm_sm_s timer2;
static uint32_t m_time = 0;

void TIMER_PrintStat(char* str, int max_len)
{
	uint32_t time;
	HAL_NVIC_DisableIRQ(TIMER_IRQn);
	time = m_time;
	HAL_NVIC_EnableIRQ(TIMER_IRQn);
	snprintf (str, max_len, "%05d", time);
}

tm_sm_s* getTimerByNumber(uint8_t num)
{
	tm_sm_s *p_timer;
	switch(num){
	case 1:
		p_timer = &timer1;
		break;
	case 2:
		p_timer = &timer2;
		break;
	default:
		p_timer = &timer1;
		break;
	}
	return p_timer;
}

//Should be called from HW timer interrupt
void TIMER_Tick(void)
{
	m_time++;
	if (RUNNING == timer1.state){
		if (timer1.counter) timer1.counter--;
		if (0 == timer1.counter) timer1.state = EXPIRED;
	} else	if (EXPIRED_AUTO == timer1.state || RUNNING_AUTO == timer1.state){
		if (timer1.counter) timer1.counter--;
		if (0 == timer1.counter){
			timer1.counter = timer1.delay;
			timer1.state = EXPIRED_AUTO;
		}
	}
	if (RUNNING == timer2.state){
		if (timer2.counter) timer2.counter--;
		if (0 == timer2.counter) timer2.state = EXPIRED;
	}
}

void TIMER_Task(void)
{
	tm_sm_e state;

	HAL_NVIC_DisableIRQ(TIMER_IRQn);
	state = timer1.state;
	if (EXPIRED == state) timer1.state = IDLE;
	if (EXPIRED_AUTO == state) timer1.state = RUNNING_AUTO;
	HAL_NVIC_EnableIRQ(TIMER_IRQn);
	if (EXPIRED_AUTO == state || EXPIRED == state)
	{
		EQ_PutEvent(TIMER1_EXPIRED);
	}

	HAL_NVIC_DisableIRQ(TIMER_IRQn);
	state = timer2.state;
	if (EXPIRED == state) timer2.state = IDLE;
	if (EXPIRED_AUTO == state) timer2.state = RUNNING_AUTO;
	HAL_NVIC_EnableIRQ(TIMER_IRQn);
	if (EXPIRED_AUTO == state || EXPIRED == state)
	{
		EQ_PutEvent(TIMER2_EXPIRED);
	}
}

void TIMER_Start(uint8_t num, uint32_t delay)
{
	tm_sm_s *p_timer;
	p_timer = getTimerByNumber(num);
	HAL_NVIC_DisableIRQ(TIMER_IRQn);
	p_timer->state = RUNNING;
	p_timer->counter = delay;
	HAL_NVIC_EnableIRQ(TIMER_IRQn);
}

void TIMER_StartAuto(uint8_t num, uint32_t delay)
{
	tm_sm_s *p_timer;
	//Only timer1 is allowed
//	num = num;
	p_timer = getTimerByNumber(1);
	HAL_NVIC_DisableIRQ(TIMER_IRQn);
	p_timer->state = RUNNING_AUTO;
	p_timer->counter = delay;
	p_timer->delay = delay;
	HAL_NVIC_EnableIRQ(TIMER_IRQn);
}

void TIMER_Stop(uint8_t num)
{
	tm_sm_s *p_timer;
	p_timer = getTimerByNumber(num);
	HAL_NVIC_DisableIRQ(TIMER_IRQn);
	p_timer->state = IDLE;
	HAL_NVIC_EnableIRQ(TIMER_IRQn);
}


/******END OF FILE****/

