/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx.h"
#include "stm32_dsp.h"


/** @addtogroup STM32F10x_DSP_Lib
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void ResetPID(PID_COEFS *c)
{
	c->PrevError = 0;
	c->Int = 0;
}

/**
  * @brief  PID in C, Error computed outside the routine
  * @param ror: difference between reference and measured value
  *   Coeff: pointer to the coefficient table
  * @retval : PID output (command)
  */
#define DEBUG_BUF_LEN 40

int DoPID(int16_t Error, PID_COEFS *c, int16_t *pOut)
{
//	char cmd_buf[DEBUG_BUF_LEN] = {0,};

//	int32_t sum_k;
//	int32_t sum_d;
  int32_t Output;
  int32_t Buf;
  int result = 0;
  //Int += Ki*Error;
  Buf = Error;
  Buf *= c->Ki;
  c->Int += Buf;
  if (c->Int > 32767) {
	  c->Int = 32767;
	  result |= 0x01;
  }
  else if (c->Int < -32767) {
	  c->Int = -32767;
	  result |= 0x01;
  }

  Output = Error;
  Output *= (int32_t)c->Kp;
//  sum_k = Output;
  Output += c->Int;

  //  Output += Kd * (Error - PrevError);
  Buf = (int32_t) Error;
  Buf -= (int32_t) c->PrevError;
  c->PrevError = Error;
  Buf *= (int32_t)c->Kd;
//  sum_d = Buf;
  Output += Buf;

//  snprintf (cmd_buf, DEBUG_BUF_LEN, "sum_k=%d sum_d=%d\n\r",
//			sum_k, sum_d);
//	print (cmd_buf);

  if (Output > 32767) {
	  Output = 32767;
	  result |= 0x02;
  }  else if (Output < -32767) {
	  Output = -32767;
	  result |= 0x02;
  }
  *pOut = (int16_t) Output;

  return (result);
}

/**
  * @}
  */ 




