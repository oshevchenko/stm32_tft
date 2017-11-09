/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "stm32_microrl_misc.h"
#include "event_queue.h"
#include "timer.h"
#include "speedometer.h"
#include "pid_regulator.h"
#include "stm32_dsp.h"
#include "main_hal.h"
#include "logger.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
typedef enum STATE_MACHINE_TAG {
	MANUAL = 0,
	AUTO
} state_machine_e;
//arm-none-eabi-objcopy -O binary "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.bin" && arm-none-eabi-size "${BuildArtifactFileName}"
//arm-none-eabi-objcopy -O ihex "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.hex"
static st_stat_module PidStat = {.printStatFunc = PidPrintStat};
static st_stat_module SpeedStat = {.printStatFunc = SPEED_PrintStat};
static st_stat_module LoggerStat = {.printStatFunc = LOGGER_PrintData};

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	eq_queue_element_s ev;
	PID_OUT pid_out;
	state_machine_e sm = MANUAL;
	int16_t speed_x;
	int16_t speed_y;
	int8_t speed_x_s, speed_y_s;
	uint8_t cnt = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_USB_DEVICE_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();

  /* USER CODE BEGIN 2 */
  init();
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_GPIO_WritePin(GPIO_LED1_GPIO_Port, GPIO_LED1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIO_LED2_GPIO_Port, GPIO_LED2_Pin, GPIO_PIN_SET);
  TIMER_StartAuto(1, 75);
  HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);
//  HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_1 );
  TERM_RegisterPrintStatCallback(&PidStat);
  TERM_RegisterPrintStatCallback(&SpeedStat);
  TERM_RegisterPrintStatCallback(&LoggerStat);
  PID_Init();
  SPEED_Init();

  LOGGER_Trigger();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  TERM_Task();
	  TIMER_Task();
	  SPEED_Task();

//	  enc_x = TIM1->CNT;
//	  enc_y = TIM3->CNT;
//
//	  SetEnc(enc_x, enc_y);
	  do {
		  EQ_GetEvent(&ev);
		  switch(ev.event){
		  case NO_EVENT:
			  break;
		  case CDC_EMPTY:
			  LOGGER_PrintData();
			  break;

    	  case CMD_WIDTH:
		  	  TIMER_StartAuto(1, ev.param.uiParam);
		  	  break;
		  case CMD_SPEED:

			  speed_x_s = (int8_t)(ev.param.uiParam & 0xFF);
			  ev.param.uiParam >>= 8;
			  speed_y_s = (int8_t)(ev.param.uiParam & 0xFF);

			  if (sm == MANUAL) {
				  if (speed_x_s >= 0) pid_out.Dir = 0;
				  else {
					  pid_out.Dir = 1;
					  speed_x_s = -speed_x_s;
				  }
				  if (speed_x_s > 100) speed_x_s = 100;
				  pid_out.Out = (uint16_t) speed_x_s;
				  pid_out.Out *= 655;
				  TIMER_HAL_SetMotor_X(&pid_out);

				  if (speed_y_s >= 0) pid_out.Dir = 0;
				  else {
					  pid_out.Dir = 1;
					  speed_y_s = -speed_y_s;
				  }
				  if (speed_y_s > 100) speed_y_s = 100;
				  pid_out.Out = (uint16_t) speed_y_s;
				  pid_out.Out *= 655;
				  TIMER_HAL_SetMotor_Y(&pid_out);

			  } else {
				  speed_x = speed_x_s;
				  speed_y = speed_y_s;
			  }
			  LOGGER_Trigger();
			  break;
		  case CMD_STAB_ON:
			  sm = AUTO;
			  PidReset();
			  speed_x = 0;
			  speed_y = 0;
//			  HAL_GPIO_WritePin(GPIO_LED2_GPIO_Port, GPIO_LED2_Pin, GPIO_PIN_RESET);
//              TIM4->CCR3=65535;
//              TIM4->CCR4=65535;
			  break;
		  case CMD_STAB_OFF:
			  sm = MANUAL;
			  pid_out.Out = 0;
			  pid_out.Dir = 0;
			  TIMER_HAL_SetMotor_X(&pid_out);
			  TIMER_HAL_SetMotor_Y(&pid_out);

//			  HAL_GPIO_WritePin(GPIO_LED2_GPIO_Port, GPIO_LED2_Pin, GPIO_PIN_SET);
//              TIM4->CCR3=5;
//              TIM4->CCR4=5;
			  break;
		  case TIMER1_EXPIRED:
			  SPEED_Timeout();
			  if (cnt < 20) {
				  cnt++;
			  } else {
					HAL_GPIO_WritePin(GPIO_USB_DP_GPIO_Port, GPIO_USB_DP_Pin, GPIO_PIN_SET);
			  }
		  	  if (sm == AUTO) {
				  PidRun(speed_x, speed_y);
		  	  } else {
		  		  PidIdle();
		  	  }
		  	  LOGGER_GetData();
			  break;
		  case CMD_GET_STAT:
			  LOGGER_StartStat();
			  break;
		  default:
			  break;
		  }
	  } while (ev.event != NO_EVENT);
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//	  uint8_t Buf[100] = "";
//	  sprintf(Buf,"cnt 0x%08d    ", int_counter);
//
//	  CDC_Transmit_FS(Buf, 15);
//	  HAL_Delay(1000);
//	  if (1 == blink)
//	  {
//		  HAL_GPIO_WritePin(GPIO_LED1_GPIO_Port, GPIO_LED1_Pin, GPIO_PIN_SET);
//		  blink = 0;
//	  } else {
//		  HAL_GPIO_WritePin(GPIO_LED1_GPIO_Port, GPIO_LED1_Pin, GPIO_PIN_RESET);
//		  blink = 1;
//	  }


  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Enables the Clock Security System 
    */
  HAL_RCC_EnableCSS();

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* RTC init function */
static void MX_RTC_Init(void)
{

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 48000;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM8 init function */
static void MX_TIM8_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 511;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_LED1_Pin|GPIO_LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_OSC_SYNC_GPIO_Port, GPIO_OSC_SYNC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_DIR_X_Pin|GPIO_DIR_Y_Pin|GPIO_USB_DP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GPIO_LED1_Pin GPIO_LED2_Pin */
  GPIO_InitStruct.Pin = GPIO_LED1_Pin|GPIO_LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_OSC_SYNC_Pin */
  GPIO_InitStruct.Pin = GPIO_OSC_SYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIO_OSC_SYNC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_S3_Pin */
  GPIO_InitStruct.Pin = GPIO_S3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_S3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_DIR_X_Pin GPIO_DIR_Y_Pin */
  GPIO_InitStruct.Pin = GPIO_DIR_X_Pin|GPIO_DIR_Y_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_USB_DP_Pin */
  GPIO_InitStruct.Pin = GPIO_USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_USB_DP_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint16_t TIMER_HAL_GetSpeedometerTime_L()
{
	uint16_t time = 0;
	return time;
}

uint16_t TIMER_HAL_GetSpeedometerTime_R()
{
	uint16_t period = 0;
	uint16_t pulse = 0;
	uint16_t tmp;
	uint16_t cnt;
	static uint16_t cnt_old_1 = 0;
	static uint16_t cnt_old_2 = 0;
	static uint16_t time = 0;

	HAL_GPIO_TogglePin(GPIO_OSC_SYNC_GPIO_Port, GPIO_OSC_SYNC_Pin);
	if ((RESET != __HAL_TIM_GET_FLAG(&htim8, TIM_FLAG_CC1)) && (RESET == __HAL_TIM_GET_FLAG(&htim8, TIM_FLAG_CC1OF))){
		cnt = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_1);
		time = cnt - cnt_old_1;
		cnt_old_1 = cnt;
	}

	if ((RESET != __HAL_TIM_GET_FLAG(&htim8, TIM_FLAG_CC2)) && (RESET == __HAL_TIM_GET_FLAG(&htim8, TIM_FLAG_CC2OF))){
		cnt = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_2);
		time = cnt - cnt_old_2;
		cnt_old_2 = cnt;
		HAL_GPIO_TogglePin(GPIO_LED1_GPIO_Port, GPIO_LED1_Pin);

	}

//	pulse = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_2);
//    period = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_1);

//	if(__HAL_TIM_GET_FLAG(&htim8, TIM_FLAG_CC1) != RESET)
//	{
//		HAL_GPIO_WritePin(GPIO_LED1_GPIO_Port, GPIO_LED1_Pin, GPIO_PIN_SET);
//		pulse = (int16_t) TIM8->CCR1;
//		time = abs(pulse);
//	} else if (__HAL_TIM_GET_FLAG(&htim8, TIM_FLAG_CC2) != RESET) {
//		HAL_GPIO_WritePin(GPIO_LED1_GPIO_Port, GPIO_LED1_Pin, GPIO_PIN_RESET);
//		time = (int16_t) TIM8->CCR2;
//	}

	return time;
}

void TIMER_HAL_EnableIrq_L()
{
}

void TIMER_HAL_DisableIrq_L()
{
}

uint16_t TIMER_HAL_GetEncoder_X()
{
	uint16_t enc_x;
	enc_x = TIM3->CNT;
	return enc_x;
}

void TIMER_HAL_EnableIrq_R()
{
	HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);
}

void TIMER_HAL_DisableIrq_R()
{
	HAL_NVIC_DisableIRQ(TIM8_CC_IRQn);
}

uint16_t TIMER_HAL_GetEncoder_Y()
{
	uint16_t enc_y;
	enc_y = TIM1->CNT;
	return enc_y;
}

void TIMER_HAL_SetMotor_X(PID_OUT* p)
{
	if (p->Dir) {
		HAL_GPIO_WritePin(GPIO_DIR_X_GPIO_Port, GPIO_DIR_X_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIO_DIR_X_GPIO_Port, GPIO_DIR_X_Pin, GPIO_PIN_RESET);
	}

	TIM4->CCR3 = p->Out;

}

void TIMER_HAL_SetMotor_Y(PID_OUT* p)
{
	if (p->Dir) {
		HAL_GPIO_WritePin(GPIO_DIR_Y_GPIO_Port, GPIO_DIR_Y_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIO_DIR_Y_GPIO_Port, GPIO_DIR_Y_Pin, GPIO_PIN_RESET);
	}
	TIM4->CCR4 = p->Out;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
