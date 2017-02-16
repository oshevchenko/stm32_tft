#include "stm32f1xx_hal.h"
#include "ads7843.h"
static uint8_t TxData[3] = {0, 0, 0};
static uint8_t RxData[3]= {0, 0, 0};
static uint8_t completed1 = 1;
static SPI_HandleTypeDef *hspi_g = NULL;
static uint8_t mode = 0;
static uint8_t pen_irq = 0;

static uint16_t position_x = 0;
static uint16_t position_y = 0;
static ADS7843_PositionState pos_state = ST_UNTOUCH;
#define POS_BUF_SIZE 15
uint16_t buf_x[POS_BUF_SIZE];
uint16_t buf_y[POS_BUF_SIZE];


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
int cmpfunc (const void * a, const void * b)
{
	int result;
	result = ( *(uint16_t*)a == *(uint16_t*)b ) ? 0 : (( *(uint16_t*)a > *(uint16_t*)b ) ? 1 : -1);
	return result;
}

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
static uint16_t median_filter(uint16_t pos, uint16_t *pBuf)
{
	static uint16_t buf[POS_BUF_SIZE];
	uint8_t cnt;
	for (cnt = 1; cnt < POS_BUF_SIZE; cnt++) {
		pBuf[cnt - 1] = pBuf[cnt];
	}
	pBuf[POS_BUF_SIZE - 1] = pos;
	memcpy(buf, pBuf, sizeof(buf));
	qsort(buf, POS_BUF_SIZE, sizeof(uint16_t), cmpfunc);
	return buf[POS_BUF_SIZE / 2];
}
static void touched(uint16_t pos_x, uint16_t pos_y)
{
	static uint8_t cnt=0;
	position_x = median_filter(pos_x, buf_x);
	position_y = median_filter(pos_y, buf_y);
	switch(pos_state){
	case ST_UNTOUCH:
		cnt = 0;
		pos_state = ST_DELAY;
		break;
	case ST_IDLE:
		cnt = 0;
		pos_state = ST_DELAY;
		break;
	case ST_DELAY:
		if (cnt++ > POS_BUF_SIZE / 2){
			pos_state = ST_TOUCH;
		}
		break;
	default:
		break;
	}
}


ADS7843_PositionState ADS7843_GetPosition(uint16_t* p_pos_x, uint16_t* p_pos_y)
{
	static uint8_t cnt=0;
	ADS7843_PositionState local_pos_state =  pos_state;
	if (pos_state == ST_UNTOUCH) {
		if (++cnt == 200)	pos_state = ST_IDLE;
	}
	else {
		cnt = 0;
	}
	*p_pos_x = position_x;
	*p_pos_y = 4095 - position_y;
	return local_pos_state;
}

#define TOUCH_DELAY 10
void ADS7843_Task()
{
	GPIO_PinState pin_irq;
	uint16_t pos;
	static uint16_t pos_x;
	static uint16_t pos_y;
	static uint16_t counter = 0;

	switch(mode) {
	case 0:
//		if(pen_irq == 0) break;
		pin_irq = HAL_GPIO_ReadPin(TOUCH_IRQ_GPIO_Port, TOUCH_IRQ_Pin);
		if (pin_irq == GPIO_PIN_SET) {
			if (pos_state == ST_TOUCH) pos_state = ST_UNTOUCH;
			counter = 0;
			break;
		}

		if (counter < TOUCH_DELAY) {
			counter++;
			break;
		}

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
		pos_x &= 0xFFF;
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
		pos_y &= 0xFFF;
		touched(pos_x, pos_y);
		mode = 0;
		break;
	}
}

void ADS7843_Start()
{
	pen_irq = 1;
}
