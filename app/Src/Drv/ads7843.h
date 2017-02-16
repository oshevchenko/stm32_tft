#ifndef __ADS7843_H_
#define __ADS7843_H_
void ADS7843_Init(SPI_HandleTypeDef *hspi);
void ADS7843_ReadCallback();
void ADS7843_Task();
void ADS7843_Start();
typedef enum PositionState_TAG {
	ST_IDLE,
	ST_DELAY,
	ST_TOUCH,
	ST_UNTOUCH
} ADS7843_PositionState;

ADS7843_PositionState ADS7843_GetPosition(uint16_t* p_pos_x, uint16_t* p_pos_y);
#endif
