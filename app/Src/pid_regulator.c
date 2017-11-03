#include "stm32_dsp.h"
#include "main_hal.h"
#include "pid_regulator.h"

static uint16_t encoder_x_1 = 0;
static uint16_t encoder_y_1 = 0;
static int16_t speed_x_m;
static int16_t speed_y_m;

static PID_COEFS pid_coefs;


void GetPidCoefs_X(PID_COEFS *pCoefs)
{
	memcpy(pCoefs, &pid_coefs, sizeof(pid_coefs));
}

void SetPidCoefs_X(PID_COEFS *pCoefs)
{
	memcpy(&pid_coefs, pCoefs, sizeof(pid_coefs));
}

void GetPidCoefs_Y(PID_COEFS *pCoefs)
{
	memcpy(pCoefs, &pid_coefs, sizeof(pid_coefs));
}

void SetPidCoefs_Y(PID_COEFS *pCoefs)
{
	memcpy(&pid_coefs, pCoefs, sizeof(pid_coefs));
}

void GetPidStat(PID_REG_STAT *pStat)
{
	pStat->enc_x = encoder_x_1;
	pStat->enc_y = encoder_y_1;
	pStat->speed_x = speed_x_m;
	pStat->speed_y = speed_y_m;
}

void PidRun(int16_t speed_x_s, int16_t speed_y_s)
{
	PID_OUT pid_out;
	int16_t speed_x_e = 0;
	int16_t speed_y_e = 0;
	uint16_t encoder;

	encoder = TIMER_HAL_GetEncoder_X();
	speed_x_m  = (int16_t) encoder;
	speed_x_m -= (int16_t) encoder_x_1;
	encoder_x_1 = encoder;

	speed_x_e = speed_x_s - speed_x_m;

	DoPID(speed_x_e, &pid_coefs, &pid_out);
	//	TIMER_HAL_SetMotor_X(&pid_out);
	encoder = TIMER_HAL_GetEncoder_Y();
	speed_y_m  = (int16_t) encoder;
	speed_y_m -= (int16_t) encoder_y_1;
	encoder_y_1 = encoder;

	speed_y_e = speed_y_s - speed_y_m;

	DoPID(speed_y_e, &pid_coefs, &pid_out);
}
