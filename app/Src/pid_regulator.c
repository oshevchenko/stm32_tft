#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include "stm32_dsp.h"
#include "main_hal.h"
#include "pid_regulator.h"

static uint16_t encoder_x_1 = 0;
static uint16_t encoder_y_1 = 0;
static int16_t speed_x_m;
static int16_t speed_y_m;

static PID_COEFS pid_coefs_x = {.Int = 0, .PrevError = 0, .Kp = 300, .Ki = 50, .Kd = 100};
static PID_COEFS pid_coefs_y = {.Int = 0, .PrevError = 0, .Kp = 300, .Ki = 50, .Kd = 100};


void GetPidCoefs_X(PID_COEFS *pCoefs)
{
	memcpy(pCoefs, &pid_coefs_x, sizeof(pid_coefs_x));
}

void SetPidCoefs_X(PID_COEFS *pCoefs)
{
	memcpy(&pid_coefs_x, pCoefs, sizeof(pid_coefs_x));
}

void GetPidCoefs_Y(PID_COEFS *pCoefs)
{
	memcpy(pCoefs, &pid_coefs_y, sizeof(pid_coefs_y));
}

void SetPidCoefs_Y(PID_COEFS *pCoefs)
{
	memcpy(&pid_coefs_y, pCoefs, sizeof(pid_coefs_y));
}

void GetPidStat(PID_REG_STAT *pStat)
{
	pStat->enc_x = encoder_x_1;
	pStat->enc_y = encoder_y_1;
	pStat->speed_x = speed_x_m;
	pStat->speed_y = speed_y_m;
}
#define DEBUG_BUF_LEN 50
void PidReset()
{
	ResetPID(&pid_coefs_x);
	ResetPID(&pid_coefs_y);
}
void PidSensors()
{
	uint16_t encoder;
	encoder = TIMER_HAL_GetEncoder_X();
	speed_x_m  = (int16_t) encoder;
	speed_x_m -= (int16_t) encoder_x_1;
	encoder_x_1 = encoder;
	encoder = TIMER_HAL_GetEncoder_Y();
	speed_y_m  = (int16_t) encoder;
	speed_y_m -= (int16_t) encoder_y_1;
	encoder_y_1 = encoder;

}
void pid_output_correction(int16_t i_pid_out, PID_OUT *p_pid_out)
{
	if (i_pid_out < 0) {
		p_pid_out->Dir=1;
		p_pid_out->Out = (uint16_t) (-i_pid_out);
	} else {
		p_pid_out->Dir=0;
		p_pid_out->Out = (uint16_t) (i_pid_out);
	}
	p_pid_out->Out <<= 1;
	if (p_pid_out->Out > 60000) p_pid_out->Out = 60000;
	p_pid_out->Out += 5000;
}

void PidRun(int16_t speed_x_s, int16_t speed_y_s)
{
//	char cmd_buf[DEBUG_BUF_LEN] = {0,};
	PID_OUT pid_out;
	int16_t speed_x_e = 0;
	int16_t speed_y_e = 0;
	int16_t i_pid_out;


	speed_x_e = speed_x_s - speed_x_m;
	//	Kp=300 Ki=20 Kd=100
	DoPID(speed_x_e, &pid_coefs_x, &i_pid_out);


//	if (i_pid_out < 0) {
//		pid_out.Dir=1;
//		pid_out.Out = (uint16_t) (-i_pid_out);
//	} else {
//		pid_out.Dir=0;
//		pid_out.Out = (uint16_t) (i_pid_out);
//	}
//	pid_out.Out <<= 1;
//	if (pid_out.Out > 60000) pid_out.Out = 60000;
//	pid_out.Out += 5000;
	pid_output_correction(i_pid_out, &pid_out);
    TIMER_HAL_SetMotor_X(&pid_out);

	speed_y_e = speed_y_s - speed_y_m;
//		snprintf (cmd_buf, DEBUG_BUF_LEN, "m=%d s=%d p=%d\n\r",
//				speed_y_m, speed_y_s, i_pid_out);
//		print (cmd_buf);

	DoPID(speed_y_e, &pid_coefs_y, &i_pid_out);
	pid_output_correction(i_pid_out, &pid_out);

	TIMER_HAL_SetMotor_Y(&pid_out);
}
