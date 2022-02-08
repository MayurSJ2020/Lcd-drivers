/*
 * LCD_driver.h
 *
 *  Created on: Jan 9, 2022
 *      Author: 007ma
 */

#ifndef INC_LCD_DRIVER_H_
#define INC_LCD_DRIVER_H_

#include "stm32f407xx.h"
#include <stdint.h>

/*
 * Macros which can be used by users
 */
#define Commandmode   0
#define Datamode	  1

#define Inputmode8bit 1
#define Inputmode4bit 0

#define Nooflines2	  0
#define Nooflines1	  1

#define BlinkingOn	  1
#define BlinkingOff	  0

#define CursorON	  1
#define CursorOFF	  0

#define D_5x10		  1
#define D_5x8		  0

#define Right 		  R
#define Left 		  L

#define Clear_display  0x01
#define EntyMode_Set1  0x07 // here increment is set and shifting is also set
#define EntyMode_Set2  0x06// here increment is set and shifting is disabled
#define Fun_8b_2line   0x38 // 0 0 1 DL `N F — —
#define Fun_4b_2line   0x28 // 0 0 1 DL N F — —
#define BCD_function   0x0C // 0 0 0 0 1 D C B
#define Retrun_home    0x02
/*
 * Structure which can be used to initialize which mode you want
 */
typedef struct
{
	uint8_t Inputmode;
	uint8_t Nooflines;
	uint8_t Blinking;
	uint8_t CursorIndication;
	uint8_t Pixel;
	GPIO_RegDef_t *pdatalsb1;
	GPIO_RegDef_t *pdatamsb1;
	GPIO_RegDef_t *pCommandport;
	uint8_t RS_Pin;
	uint8_t RW_Pin;
	uint8_t EN_Pin;

}LCD_typedef;




//User helper functions
void Delayinms(uint32_t delay);
void Writestring(char *string);
void TimerConfig(void);
void LCD_Init(LCD_typedef *pLCD);
void Clear_screen(void);
void Return_home(void);
void Second_Line(void);
void Shift_Cursor(const char direction,uint8_t steps);
void Shift_Display(const char direction,uint8_t steps);
void Sendcommand(uint8_t comm);



#endif /* INC_LCD_DRIVER_H_ */
