#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include "stm32_dsp.h"
#include "main_hal.h"
#include "pid_regulator.h"
#include "speedometer.h"
#include "logger.h"

static uint16_t encoder_x_1 = 0;
static uint16_t encoder_y_1 = 0;
static int16_t speed_x_m;
static int16_t speed_y_m;

static PID_COEFS pid_coefs_x = {.Int = 0, .PrevError = 0, .Kp = 300, .Ki = 50, .Kd = 100};
static PID_COEFS pid_coefs_y = {.Int = 0, .PrevError = 0, .Kp = 300, .Ki = 50, .Kd = 100};
static int16_t i_pid_out_r;
static int16_t i_pid_out_l;
static int16_t speed_x_e;
static int16_t speed_y_e;


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


void PidPrintStat(char *str, int max_len)
{
	snprintf (str, max_len, "%d %d %d %d",
			i_pid_out_l, i_pid_out_r, speed_x_e, speed_y_e);
}

#define DEBUG_BUF_LEN 50
void PidReset()
{
	ResetPID(&pid_coefs_x);
	ResetPID(&pid_coefs_y);
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
void PidIdle()
{
	SPEED_GetSpeed(&speed_x_m, &speed_y_m);
}

void PidRun(int16_t speed_x_s, int16_t speed_y_s)
{
//	char cmd_buf[DEBUG_BUF_LEN] = {0,};
	PID_OUT pid_out;

	SPEED_GetSpeed(&speed_x_m, &speed_y_m);
	speed_x_e = speed_x_s - speed_x_m;
	//	Kp=300 Ki=20 Kd=100
	DoPID(speed_x_e, &pid_coefs_x, &i_pid_out_l);


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
	pid_output_correction(i_pid_out_l, &pid_out);
    TIMER_HAL_SetMotor_X(&pid_out);

	speed_y_e = speed_y_s - speed_y_m;
//		snprintf (cmd_buf, DEBUG_BUF_LEN, "m=%d s=%d p=%d\n\r",
//				speed_y_m, speed_y_s, i_pid_out);
//		print (cmd_buf);

	DoPID(speed_y_e, &pid_coefs_y, &i_pid_out_r);
	pid_output_correction(i_pid_out_r, &pid_out);

	TIMER_HAL_SetMotor_Y(&pid_out);
}

typedef struct pid_logger_data
{
	int16_t pid_out;
} st_pid_logger_data;
st_pid_logger_data pid_data[LOGGER_ARRAY_SIZE];
st_logger_data pid_log_data[LOGGER_ARRAY_SIZE];
static st_logger_module pid_log_module;


int PidSnprintfDataCallback(char* str, void *p_dat, int str_max)
{
	st_pid_logger_data *p_data;
	p_data = (st_pid_logger_data *) p_dat;
	snprintf(str, str_max, "pid:%d:", p_data->pid_out);
	return 0;
}

st_logger_data* PidGetLogDataCallback()
{
	static int i = 0;

	i++;
	if (LOGGER_ARRAY_SIZE == i) i = 0;
	pid_data[i].pid_out = i_pid_out_r;
	return &pid_log_data[i];
}
void PID_Init()
{
	int i;
	for (i = 0; i < LOGGER_ARRAY_SIZE; i++) {
		pid_log_data[i].p_data = &pid_data[i];
		pid_log_data[i].sprintfDataCallback = PidSnprintfDataCallback;
	}
	pid_log_module.getLogDataCallback = PidGetLogDataCallback;
	LOGGER_RegisterModule(&pid_log_module);
}
