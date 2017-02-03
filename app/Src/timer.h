/**
  ******************************************************************************
  * File Name          : timer.c
  * Description        : Simple timer.
  ******************************************************************************

  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#ifndef __TIMER_H__
#define __TIMER_H__
typedef enum TIMER_SM_STATE {
	IDLE = -1,
	RUNNING,
	EXPIRED,
	RUNNING_AUTO,
	EXPIRED_AUTO
} tm_sm_e;
typedef struct TIMER_SM_STRUCT {
	tm_sm_e state;
	uint32_t counter;
	uint32_t delay;
} tm_sm_s;

void TIMER_Tick(void);
void TIMER_Task(void);
void TIMER_Start(uint8_t num, uint32_t delay);
void TIMER_StartAuto(uint8_t num, uint32_t delay);
void TIMER_Stop(uint8_t num);

#endif //#define __TIMER_H__
/******END OF FILE****/

