#ifndef __LCD_H_
#define __LCD_H_
#include "main.h"
#define CTR_OSC_START                   0x0000
#define CTR_DRV_OUTPUT1                 0x0001
#define CTR_DRV_WAVE                    0x0002
#define CTR_ENTRY_MODE                  0x0003
#define CTR_RESIZE                      0x0004
#define CTR_DISPLAY1                    0x0007
#define CTR_DISPLAY2                    0x0008
#define CTR_DISPLAY3                    0x0009
#define CTR_DISPLAY4                    0x000A
#define CTR_RGB_INTERFACE1              0x000C
#define CTR_FRM_MARKER                  0x000D
#define CTR_RGB_INTERFACE2              0x000F
#define CTR_POWER1                      0x0010
#define CTR_POWER2                      0x0011
#define CTR_POWER3                      0x0012
#define CTR_POWER4                      0x0013
#define CTR_HORZ_ADDRESS                0x0020
#define CTR_VERT_ADDRESS                0x0021
#define CTR_WRITE_DATA                  0x0022
#define CTR_READ_DATA                   0x0022
#define CTR_POWER7                      0x0029
#define CTR_FRM_COLOR                   0x002B
#define CTR_GAMMA1                      0x0030
#define CTR_GAMMA2                      0x0031
#define CTR_GAMMA3                      0x0032
#define CTR_GAMMA4                      0x0035
#define CTR_GAMMA5                      0x0036
#define CTR_GAMMA6                      0x0037
#define CTR_GAMMA7                      0x0038
#define CTR_GAMMA8                      0x0039
#define CTR_GAMMA9                      0x003C
#define CTR_GAMMA10                     0x003D
#define CTR_HORZ_START                  0x0050
#define CTR_HORZ_END                    0x0051
#define CTR_VERT_START                  0x0052
#define CTR_VERT_END                    0x0053
#define CTR_DRV_OUTPUT2                 0x0060
#define CTR_BASE_IMAGE                  0x0061
#define CTR_VERT_SCROLL                 0x006A
#define CTR_PIMG1_POS                   0x0080
#define CTR_PIMG1_START                 0x0081
#define CTR_PIMG1_END                   0x0082
#define CTR_PIMG2_POS                   0x0083
#define CTR_PIMG2_START                 0x0084
#define CTR_PIMG2_END                   0x0085
#define CTR_PANEL_INTERFACE1    	0x0090
#define CTR_PANEL_INTERFACE2    	0x0092
#define CTR_PANEL_INTERFACE4   		0x0095
#define CTR_OTP_VCMPROGRAM              0x00A1
#define CTR_OTP_VCMSTATUS               0x00A2
#define CTR_OTP_IDKEY                   0x00A5

#define LCDRegister 	(*((volatile uint16_t*) 0x60000000))
#define LCDMemory		(*((volatile uint16_t*) 0x60020000))

#define LCD_WRITE_REGISTER(REG, DATA)	do {                 \
											LCDRegister=REG; \
											LCDMemory=DATA;  \
										} while (0)
#define LCD_BEGIN_RAM_WRITE			LCDRegister = (uint16_t) CTR_WRITE_DATA
#define LCD_WRITE_RAM(DATA)			LCDMemory = (uint16_t) DATA

void LCDHardwareReset();
void LCDSetBounds(uint16_t left, uint16_t top, uint16_t right, uint16_t bottom);
void LCDOn();
void LCDOff();
void LCDClear(uint16_t color);
void LCDInit();
void LCDUaFlag();
#endif
