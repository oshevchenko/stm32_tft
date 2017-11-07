#ifndef __MAIN_HAL_H
#define __MAIN_HAL_H
#include "stm32_dsp.h"
uint16_t TIMER_HAL_GetEncoder_X();
uint16_t TIMER_HAL_GetEncoder_Y();

void TIMER_HAL_SetMotor_X(PID_OUT* p);
void TIMER_HAL_SetMotor_Y(PID_OUT* p);
void TIMER_HAL_EnableIrq_L();
void TIMER_HAL_DisableIrq_L();
void TIMER_HAL_EnableIrq_R();
void TIMER_HAL_DisableIrq_R();
uint16_t TIMER_HAL_GetSpeedometerTime_L();
uint16_t TIMER_HAL_GetSpeedometerTime_R();

#endif /* __MAIN_HAL_H */

