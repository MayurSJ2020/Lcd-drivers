/*
 * LCD_driver.c
 *
 *  Created on: Jan 9, 2022
 *      Author: 007ma
 */



#include "LCD_driver.h"
#include <string.h>

//static functions(helper functions)
static void Sendcomm8bit(uint8_t command);
static void Sendcomm4bit(uint8_t command);
static void EnablePulse();
static void commandpin(uint8_t state);
static void Senddata8(uint8_t data);
static void Senddata4(uint8_t data);

/*
 * Pointers to Ports which are used for data and command which will be act as helper pointers
 */
static GPIO_RegDef_t *pdatalsb ;
static GPIO_RegDef_t *pdatamsb;
static GPIO_RegDef_t *pcomm ;
static uint8_t RS;
static uint8_t EN ;
static uint8_t WR;
static uint8_t Modes;

/*
 * Function : TimerConfig()
 * Parameters : None
 * This function must be called in you main code before using Delayinms funtion to initiate delay
 */

void TimerConfig(void)
{

	//Enabling clock
	RCC->APB1ENR |= 1UL;
	TIM2->CR1 &= ~(0X3FF);
	//Enabling one pulse mode
	TIM2->CR1 |= (1<<3);
	//Setting direction
	TIM2->CR1 |= (1<<4);
	//setting prescaler to 16000 our clock is 16MHZ and it will divide it by 16000
	TIM2->PSC = (uint16_t)16000;
	//Setting arr register


}

/*
 * Function : Delayinms(uint32_t delay)
 * Parameters : Takes one parameter which will assign delay in ms for ex 1 will be for 1ms
 * This function must be called in you main code before using Delayinms funtion to initiate delay
 */

void Delayinms(uint32_t delay)//This is delay in ms only
{
	//clearing before updating timer count
	TIM2->ARR = (uint32_t)0;
	TIM2->SR &= ~(0x1);
	//updating timer count
	TIM2->ARR = (uint32_t)delay-1;
	//Starting the timer
	TIM2->CR1 |= 1UL;
	while((TIM2->SR & 0x1) != 1);

}

/*
 * Function : LCD_Init(LCD_typedef *pLCD)
 * Parameters : Takes one parameter address of configurable structure which user defines
 * Used for initializing all the lcd parameters requested by the user
 */

void LCD_Init(LCD_typedef *pLCD)
{
	pdatalsb = pLCD->pdatalsb1;
	pdatamsb = pLCD->pdatamsb1;
	pcomm = pLCD->pCommandport;
	RS = pLCD->RS_Pin;
	EN = pLCD->EN_Pin;
	WR = pLCD->RW_Pin;
	Modes = pLCD->Inputmode;



	// Refer manual for commands of lcds
	uint8_t DisplayControl = 0x0C;
	uint8_t Functionset = 0x20;

	if(pLCD->Inputmode == Inputmode8bit)
	{

		Delayinms(15);
		Sendcomm8bit(0x30);
		Delayinms(5);
		Sendcomm8bit(0x30);
		Delayinms(1);
		Sendcomm8bit(0x30);
		Delayinms(1);
		Functionset |= (1<<4);
		if(pLCD->Nooflines == Nooflines2 ){Functionset |= (1<<3);}
		if(pLCD->Pixel == D_5x10){Functionset |= (1<<2);}
		Sendcomm8bit(Functionset);
		Sendcomm8bit(0x80);
		Sendcomm8bit(EntyMode_Set2);
		if(pLCD->CursorIndication == CursorON){DisplayControl |= (0x02);}
		if(pLCD->Blinking == BlinkingOn){DisplayControl |= 0x01;}
		Sendcomm8bit(DisplayControl);
		Sendcomm8bit(Clear_display);
		Delayinms(2);


	}
	else
	{
		Delayinms(15);
		GPIO_WriteToOutputPort(pdatamsb, (uint16_t)0X3);
		EnablePulse();
		Delayinms(5);
		GPIO_WriteToOutputPort(pdatamsb, (uint16_t)0X3);
		EnablePulse();
		Delayinms(5);
		GPIO_WriteToOutputPort(pdatamsb, (uint16_t)0X3);
		EnablePulse();
		GPIO_WriteToOutputPort(pdatamsb, (uint16_t)0X2);
		EnablePulse();
		if(pLCD->Nooflines == Nooflines2 ){Functionset |= (1<<3);}
		if(pLCD->Pixel == D_5x10){Functionset |= (1<<2);}
		Sendcomm4bit(Functionset);
	//	Sendcomm4bit(0x08);
		if(pLCD->CursorIndication == CursorON){DisplayControl |= (0x02);}
		if(pLCD->Blinking == BlinkingOn){DisplayControl |= 0x01;}
		Sendcomm4bit(DisplayControl);
		// Can use entrymode 1 or 2 I only used entry mode 1 refer header file and data sheet for more info
		Sendcomm4bit(EntyMode_Set2);
		Sendcomm4bit(Clear_display);
		Delayinms(2);


	}
}

/*
 * Function : Sendcomm8bit(uint8_t command)
 * Parameters : It takes one parameter which is command which you want to send to the lcd
 * This is used for sending coommands to LCD in command mode in 8bits
 */

static void Sendcomm8bit(uint8_t command)
{
	commandpin(Commandmode);
	pdatalsb->ODR = 0x00;
	pdatamsb->ODR = 0x00;
	GPIO_WriteToOutputPort(pdatalsb, ((uint16_t)command));
	GPIO_WriteToOutputPort(pdatamsb, ((uint16_t)((command>>4))));
	EnablePulse();
}

/*
 * Function : Sendcomm4bit(uint8_t command)
 * Parameters : It takes one parameter which is command which you want to send to the lcd
 * This is used for sending coommands to LCD in command mode in 4bits
 */

static void Sendcomm4bit(uint8_t command)
{
	commandpin(Commandmode);
	pdatamsb->ODR = 0x00;
	GPIO_WriteToOutputPort(pdatamsb, ((uint16_t)((command>>4)&0x0F)));
	EnablePulse();

	pdatamsb->ODR = 0x00;
	GPIO_WriteToOutputPort(pdatamsb, (uint16_t)command&0x0F);
	EnablePulse();

}

/*
 * Function : Sendcommand(uint8_t comm)
 * Parameters : It takes one parameter which is command which you want to send to the lcd
 * This is the generic function to send command which can be used in main file
 */

 void Sendcommand(uint8_t comm)
 {
	 if(Modes == Inputmode8bit)
	 {
		 Sendcomm8bit(comm);
	 }
	 if(Modes == Inputmode4bit)
	 {
		 Sendcomm4bit(comm);
	 }

 }

 /*
  * Function : Senddata8(uint8_t data)
  * Parameters : It takes one parameter which is data which you want to send to the lcd
  * This is used for sending data to LCD in command mode in 8bits
  */

 static void Senddata8(uint8_t data)
{
	commandpin(Datamode);
	pdatalsb->ODR = 0x00;
	pdatamsb->ODR = 0x00;
	GPIO_WriteToOutputPort(pdatalsb, ((uint16_t)data&0x0F));
	GPIO_WriteToOutputPort(pdatamsb, ((uint16_t)((data>>4)&0x0F)));
	EnablePulse();
}

 /*
  * Function : Senddata4(uint8_t data)
  * Parameters : It takes one parameter which is data which you want to send to the lcd
  * This is used for sending data to LCD in command mode in 4bits
  */

 static void Senddata4(uint8_t data)
{
	 commandpin(Datamode);
	 pdatamsb->ODR = 0x00;
	GPIO_WriteToOutputPort(pdatamsb, ((uint16_t)((data>>4)&0x0F)));
	EnablePulse();

	pdatamsb->ODR = 0x00;
	GPIO_WriteToOutputPort(pdatamsb, (uint16_t)data&0x0F);
	EnablePulse();

}

 /*
  * Function : EnablePulse()
  * Parameters : NA
  * This is used to toggle enable pin
  */

 static void EnablePulse()
{
	GPIO_WriteToOutputPin(pcomm, EN, GPIO_PIN_SET);
	Delayinms(2);
	GPIO_WriteToOutputPin(pcomm, EN, GPIO_PIN_RESET);
}

 /*
  * Function : Writestring(char *string)
  * Parameters : It takes one parameter which is array which consists of a string user want to print in lcd
  * User helper function
  */


void Writestring(char *string)
{

	if(Modes == Inputmode8bit)
	{
	for(uint8_t i=0;i<32 && string[i] != '\0';i++)
	{
		Senddata8(string[i]);
		if(i == 16 ){Sendcomm8bit(0xc0);}

	}
	}
	if(Modes == Inputmode4bit)
	{
		for(uint8_t i=0;i<32 && string[i] != '\0';i++)
		{
			Senddata4(string[i]);
		if(i == 16 ){Sendcomm8bit(0xc0);}

		}
	}


}

/*
* Function : commandpin(uint8_t state)
* Parameters : It take one parameter
* Helps to put lcd in either data or command mode
*/

static void commandpin(uint8_t state)
{
	if(state == Commandmode)
	{
		GPIO_WriteToOutputPin(pcomm, RS, GPIO_PIN_RESET);
	}
	else
	{
		GPIO_WriteToOutputPin(pcomm, RS, GPIO_PIN_SET);
	}
}

/*
 * User helper functions
 */
void Clear_screen(void)
{
	Sendcommand(Clear_display);
}

void Return_home(void)
{
	Sendcommand(Retrun_home);
}

void Second_Line(void)
{
	Sendcommand(0xC0);
}

void Shift_Cursor(const char direction,uint8_t steps)
{
	uint8_t dummy = 0x10;
	if(direction == 'R')
	{
		dummy |= (1<<2);
		while(steps>0)
		{
			Sendcommand(dummy);
			steps--;
		}
	}
	if(direction == 'L')
	{
		while(steps>0)
		{
			Sendcommand(dummy);
			steps--;
		}
	}
}

void Shift_Display(const char direction,uint8_t steps)
{
	uint8_t dummy = 0x18;
	if(direction == 'R')
	{
		dummy |= (1<<2);
		while(steps>0)
		{
			Sendcommand(dummy);
			steps--;
		}
	}
	if(direction == 'L')
	{
		while(steps>0)
		{
			Sendcommand(dummy);
			steps--;
		}
	}

}




