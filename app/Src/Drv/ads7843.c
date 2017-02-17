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
#define POS_BUF_SIZE 32
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

#define MODE_POWER_DOWN ((1 << BIT_S))

#define X_PLUS ((1 << BIT_S) | (1<< BIT_SER_DFR) | (1 << BIT_A0))
#define Y_PLUS ((1 << BIT_S) | (1<< BIT_SER_DFR) | (1 << BIT_A2) | (1 << BIT_A0))
#define X_PLUS_1 ((1 << BIT_S) | (1<< BIT_SER_DFR) | (1 << BIT_A0) | (1 << BIT_PD1) | (1 << BIT_PD0))
#define X_PLUS_2 ((1 << BIT_S)                     | (1 << BIT_A0) | (1 << BIT_PD1) | (1 << BIT_PD0))
#define Y_PLUS_1 ((1 << BIT_S) | (1<< BIT_SER_DFR) | (1 << BIT_A2) | (1 << BIT_A0) | (1 << BIT_PD1) | (1 << BIT_PD0))
#define Y_PLUS_2 ((1 << BIT_S)                     | (1 << BIT_A2) | (1 << BIT_A0) | (1 << BIT_PD1) | (1 << BIT_PD0))
//#define Y_PLUS ((1 << BIT_S) | (1 << BIT_A2) | (1 << BIT_A0) | (1 << BIT_MODE)| (1 << BIT_PD1)| (1 << BIT_PD0))
int cmpfunc (const void * a, const void * b)
{
	int result;
	uint16_t cmp_a = *(uint16_t*)a;
	uint16_t cmp_b = *(uint16_t*)b;
	if (cmp_a > cmp_b) result = 1;
	else if (cmp_a < cmp_b) result = -1;
	else result = 0;
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

void ADS7843_Debug(uint8_t start, uint8_t inc)
{
	static uint16_t cnt_touched;
	static uint16_t cnt_usb;
	if (start){
		cnt_touched=0;
		cnt_usb = 0;
		return;
	}
	if (inc == 1) cnt_touched++;
	else if (inc == 2) cnt_usb++;
	if (cnt_touched == 2000)
	{
		cnt_touched=0;
		cnt_usb = 0;
	}
}
static uint16_t buf[POS_BUF_SIZE];
static uint16_t median_filter(uint16_t pos, uint16_t *pBuf)
{
	uint8_t cnt;
	for (cnt = 1; cnt < POS_BUF_SIZE; cnt++) {
		pBuf[cnt - 1] = pBuf[cnt];
	}
	pBuf[POS_BUF_SIZE - 1] = pos;
	memcpy(buf, pBuf, sizeof(buf));
	qsort(buf, POS_BUF_SIZE, sizeof(uint16_t), cmpfunc);
	return buf[POS_BUF_SIZE >> 1];
}
static void touched(uint8_t reset, uint16_t pos_x, uint16_t pos_y)
{
	static uint8_t cnt=0;
	if (reset) {
		if (pos_state != ST_UNTOUCH) {
			cnt = 0;
			memset(buf_x, 0, sizeof(buf_x));
			memset(buf_y, 0, sizeof(buf_y));
			pos_state = ST_UNTOUCH;
		}
		return;
	}
	if (pos_state == ST_UNTOUCH) pos_state  = ST_DELAY;

	if ((pos_x > 4000) || (pos_y > 4000)) return;
	position_x = median_filter(pos_x, buf_x);
	position_y = median_filter(pos_y, buf_y);

	if (cnt < POS_BUF_SIZE) {
		cnt++;
		return;
	}

	pos_state = ST_TOUCH;
}


ADS7843_PositionState ADS7843_GetPosition(uint16_t* p_pos_x, uint16_t* p_pos_y)
{
	static int16_t old_ret;
	static uint8_t cnt=0;
	ADS7843_PositionState local_pos_state =  pos_state;
	if (local_pos_state != ST_TOUCH) {
		goto exit;
	}
	*p_pos_x = position_x;
	*p_pos_y = 4095 - position_y;

	old_ret = position_y;

exit:
	return local_pos_state;
}

#define TOUCH_DELAY 5
void ADS7843_Task()
{
	GPIO_PinState pin_irq;
	uint16_t pos;
	static uint16_t pos_x;
	static uint16_t pos_x_1;
	static uint16_t pos_x_2;
	static uint16_t pos_y_1;
	static uint16_t pos_y_2;
	static uint16_t pos_y;
	static uint16_t counter = 0;

	switch(mode) {
	case 0:
		pin_irq = HAL_GPIO_ReadPin(TOUCH_IRQ_GPIO_Port, TOUCH_IRQ_Pin);
		if (pin_irq == GPIO_PIN_SET) {
			touched(1, 0, 0);
			counter = 0;
			break;
		}

		if (counter < TOUCH_DELAY) {
			counter++;
			ADS7843_Debug(1, 0); //reset debug counters
			break;
		}
		HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(TOUCH_IRQ_GPIO_Port, TOUCH_IRQ_Pin, GPIO_PIN_RESET);
		completed1 = 0;
		mode = 10;
		TxData[0] = (uint8_t) X_PLUS_2;
		HAL_SPI_TransmitReceive_DMA(hspi_g, TxData, RxData, 3);
		break;
//	case 1:
//		if (completed1 == 0) break;
//		pos = (uint16_t) RxData[2];
//		pos >>= 3;
//		pos_x_1 = pos;
//		pos = (uint16_t) RxData[1];
//		pos <<= 5;
//		pos_x_1 |= pos;
//		pos_x_1 &= 0xFFF;
//		completed1 = 0;
//		mode = 10;
//		TxData[0] = (uint8_t) X_PLUS_2;
//		HAL_SPI_TransmitReceive_DMA(hspi_g, TxData, RxData, 3);
//		break;
	case 10:
		if (completed1 == 0) break;
		pos = (uint16_t) RxData[2];
		pos >>= 3;
		pos_x_2 = pos;
		pos = (uint16_t) RxData[1];
		pos <<= 5;
		pos_x_2 |= pos;
		pos_x_2 &= 0xFFF;
		completed1 = 0;
		mode = 12;
		TxData[0] = (uint8_t) Y_PLUS_2;
		HAL_SPI_TransmitReceive_DMA(hspi_g, TxData, RxData, 3);
		break;
//	case 11:
//		if (completed1 == 0) break;
//		pos = (uint16_t) RxData[2];
//		pos >>= 3;
//		pos_y_1 = pos;
//		pos = (uint16_t) RxData[1];
//		pos <<= 5;
//		pos_y_1 |= pos;
//		pos_y_1 &= 0xFFF;
//		completed1 = 0;
//		mode = 12;
//		TxData[0] = (uint8_t) Y_PLUS_2;
//		HAL_SPI_TransmitReceive_DMA(hspi_g, TxData, RxData, 3);
//		break;
	case 12:
		if (completed1 == 0) break;
		pos = (uint16_t) RxData[2];
		pos >>= 3;
		pos_y_2 = pos;
		pos = (uint16_t) RxData[1];
		pos <<= 5;
		pos_y_2 |= pos;
		pos_y_2 &= 0xFFF;
		completed1 = 0;
		mode = 15;
		touched(0, pos_x_2, pos_y_2);
		TxData[0] = (uint8_t) MODE_POWER_DOWN;
		HAL_SPI_TransmitReceive_DMA(hspi_g, TxData, RxData, 3);
		break;
	case 15:
		if (completed1 == 0) break;
		HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(TOUCH_IRQ_GPIO_Port, TOUCH_IRQ_Pin, GPIO_PIN_SET);
		mode = 0;
		break;
	}
}

void ADS7843_Start()
{
	pen_irq = 1;
}
