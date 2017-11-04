/**
  ******************************************************************************
  * @file STM32F10x_DSP_Lib/src/PID_C_stm32.c 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  This source file contains code PID controller
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 


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
uint16_t PrevError = 0, IntTerm = 0;
static int16_t PrevError_C = 0;
static int32_t IntTerm_C = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void ResetPID()
{
	PrevError_C = 0;
	IntTerm_C = 0;
}

/**
  * @brief  PID in C, Error computed outside the routine
  * @param ror: difference between reference and measured value
  *   Coeff: pointer to the coefficient table
  * @retval : PID output (command)
  */
#define DEBUG_BUF_LEN 40

int DoPID(int16_t Error, PID_COEFS *c, int32_t *pOut)
{
	char cmd_buf[DEBUG_BUF_LEN] = {0,};

	int32_t sum_k;
	int32_t sum_d;
  int32_t Output;
  int32_t Buf;
  int result = 0;
  //IntTerm_C += Ki*Error;
  Buf = Error;
  Buf *= c->Ki;
  IntTerm_C += Buf;
  if (IntTerm_C > 65535) {
	  IntTerm_C = 65535;
	  result |= 0x01;
  }
  else if (IntTerm_C < -65535) {
	  IntTerm_C = -65535;
	  result |= 0x01;
  }

  Output = Error;
  Output *= (int32_t)c->Kp;
  sum_k = Output;
  Output += IntTerm_C;

  //  Output += Kd * (Error - PrevError_C);
  Buf = (int32_t) Error;
  Buf -= (int32_t) PrevError_C;
  PrevError_C = Error;
  Buf *= (int32_t)c->Kd;
  sum_d = Buf;
  Output -= Buf;

  snprintf (cmd_buf, DEBUG_BUF_LEN, "sum_k=%d sum_d=%d\n\r",
			sum_k, sum_d);
	print (cmd_buf);

  if (Output > 32767) {
	  Output = 32767;
	  result |= 0x02;
  }  else if (Output < -32767) {
	  Output = -32767;
	  result |= 0x02;
  }
  *pOut = Output;

  return (result);
}



/**
  * @brief  PID in C, Error computed inside the routine
  * @param : Input (measured value)
  *   Ref: reference (target value)
  *   Coeff: pointer to the coefficient table
  * @retval : PID output (command)
  */
uint16_t DoFullPID(uint16_t In, uint16_t Ref, uint16_t *Coeff)
{
  uint16_t Kp, Ki, Kd, Output, Error;

  Error = Ref - In;
  Kp = Coeff[0];
  Ki = Coeff[1];
  Kd = Coeff[2];

  IntTerm_C += Ki*Error;
  Output = Kp * Error;
  Output += IntTerm_C;
  Output += Kd * (Error - PrevError_C);

  PrevError_C = Error;

  return (Output);
}

/**
  * @}
  */ 




/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
