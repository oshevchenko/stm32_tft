#ifndef __MAIN_HAL_H
#define __MAIN_HAL_H
uint16_t TIMER_HAL_GetEncoder_X();
uint16_t TIMER_HAL_GetEncoder_Y();

void TIMER_HAL_SetMotor_X(PID_OUT* p);
void TIMER_HAL_SetMotor_Y(PID_OUT* p);

#endif /* __MAIN_HAL_H */

