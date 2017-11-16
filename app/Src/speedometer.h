/**
  ******************************************************************************
  * File Name          : speedometer.h
  * Description        : Calculate speed by measuring the time between encoder
  *                      signals pulses.
  ******************************************************************************

  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#ifndef __SPEEDOMETER_H__
#define __SPEEDOMETER_H__
typedef enum SPEEDOMETER_SM_STATE {
	SPEED_IDLE = -1,
	SPEED_TRIGGERED
} speed_sm_e;
typedef struct __SPEEDOMETER_SM_STRUCT {
	speed_sm_e state;
	int16_t delta;
	uint16_t old_encoder;
	uint16_t encoder;
	int16_t speed;
	uint8_t timeout_counter;
} speed_sm_s;



void SPEED_RightTick(void);
void SPEED_LeftTick(void);
void SPEED_Task(void);
void SPEED_Timeout(void);
void SPEED_GetSpeed(int16_t *p_speed_left, int16_t *p_speed_right);
void SPEED_PrintStat(char* str, int max_len);
void SPEED_Init();
#endif //#define __SPEEDOMETER_H__
/******END OF FILE****/

