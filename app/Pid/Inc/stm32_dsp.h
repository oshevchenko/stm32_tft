/**
  ******************************************************************************
  * @file STM32F10x_DSP_Lib/inc/stm32_dsp.h 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  This source file contains prototypes of DSP functions
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_DSP_H
#define __STM32F10x_DSP_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx.h"


/* Exported types ------------------------------------------------------------*/
/* for block FIR module */

typedef struct {
  uint16_t Kp;
  uint16_t Ki;
  uint16_t Kd;
  int16_t PrevError;
  int32_t Int;
} PID_COEFS;

typedef struct {
  uint8_t Dir;
  uint16_t Out;
} PID_OUT;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


void ResetPID(PID_COEFS *c);
/* PID controller in C, error computed outside the function */
int DoPID(int16_t Error, PID_COEFS *c, int16_t *pOut);

#endif /* __STM32F10x_DSP_H */

