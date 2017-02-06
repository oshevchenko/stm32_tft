#include "stm32f1xx_hal.h"
static uint8_t TxData[3] = {0, 0, 0};
static uint8_t RxData[3]= {0, 0, 0};
static uint8_t completed1 = 1;
static SPI_HandleTypeDef *hspi_g = NULL;
static uint8_t mode = 0;
static uint8_t pen_irq = 0;
static uint16_t pos_x;
static uint16_t pos_y;

#define BIT_S       7
#define BIT_A2      6
#define BIT_A1      5
#define BIT_A0      4
#define BIT_MODE    3
#define BIT_SER_DFR 2
#define BIT_PD1     1
#define BIT_PD0     0

#define X_PLUS ((1 << BIT_S) | (1 << BIT_A0) | (1 << BIT_MODE))
#define Y_PLUS ((1 << BIT_S) | (1 << BIT_A2) | (1 << BIT_A0) | (1 << BIT_MODE))
//#define X_PLUS ((1 << BIT_S) | (1 << BIT_A0) | (1 << BIT_MODE)| (1 << BIT_PD1)| (1 << BIT_PD0))
//#define Y_PLUS ((1 << BIT_S) | (1 << BIT_A2) | (1 << BIT_A0) | (1 << BIT_MODE)| (1 << BIT_PD1)| (1 << BIT_PD0))


void ADS7843_Init(SPI_HandleTypeDef *hspi)
{
	hspi_g = hspi;
	mode = 0;
	HAL_GPIO_WritePin(TOUCH_IRQ_GPIO_Port, TOUCH_IRQ_Pin, GPIO_PIN_SET);
}
void ADS7843_ReadCallback()
{
	completed1 = 1;
}
void ADS7843_Task()
{
	GPIO_PinState pin_irq;
	uint16_t pos;
	switch(mode) {
	case 0:
//		if(pen_irq == 0) break;
		pin_irq = HAL_GPIO_ReadPin(TOUCH_IRQ_GPIO_Port, TOUCH_IRQ_Pin);
		if (pin_irq == GPIO_PIN_SET) break;
		HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(TOUCH_IRQ_GPIO_Port, TOUCH_IRQ_Pin, GPIO_PIN_RESET);
		completed1 = 0;
		mode = 2;
		TxData[0] = (uint8_t) X_PLUS;
		HAL_SPI_TransmitReceive_DMA(hspi_g, TxData, RxData, 3);
		break;
	case 2:
		if (completed1 == 0) break;
		pos = (uint16_t) RxData[2];
		pos >>= 3;
		pos_x = pos;
		pos = (uint16_t) RxData[1];
		pos <<= 5;
		pos_x |= pos;
		completed1 = 0;
		mode = 3;
		TxData[0] = (uint8_t) Y_PLUS;
		HAL_SPI_TransmitReceive_DMA(hspi_g, TxData, RxData, 3);
		break;
	case 3:
		if (completed1 == 0) break;
		HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(TOUCH_IRQ_GPIO_Port, TOUCH_IRQ_Pin, GPIO_PIN_SET);
		pos = (uint16_t) RxData[2];
		pos >>= 3;
		pos_y = pos;
		pos = (uint16_t) RxData[1];
		pos <<= 5;
		pos_y |= pos;
//		pen_irq = 0;
		mode = 0;
		break;
	}
}

void ADS7843_Start()
{
	pen_irq = 1;
}
