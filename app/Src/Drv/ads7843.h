#ifndef __ADS7843_H_
#define __ADS7843_H_
void ADS7843_Init(SPI_HandleTypeDef *hspi);
void ADS7843_ReadCallback();
void ADS7843_Task();
void ADS7843_Start();
#endif
