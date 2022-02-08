/* * lcd.c
 *
 *  Created on: 14-Jan-2022
 *      Author: 007ma
 */
#include "LCD_driver.h"
#include <stdio.h>


LCD_typedef mylcd;
void Gpio_Initialization();
void Lcd_Initializatiion();


int main()
{
	char stringss[] = "First line test1";

	Gpio_Initialization();
	TimerConfig();
	Lcd_Initializatiion();

	Delayinms(1000);

	Writestring(stringss);
	while(1)
	{


	Second_Line();
	Writestring("MAYUR SJ");
	Delayinms(5000);
	Second_Line();
	Writestring("Second line2.0");
	Delayinms(5000);
	}







}

void Lcd_Initializatiion()
{
	mylcd.Blinking = BlinkingOff;
	mylcd.CursorIndication = CursorON;
	mylcd.EN_Pin = GPIO_PIN_N0_0;
	mylcd.Inputmode = Inputmode4bit;
	mylcd.Nooflines = Nooflines2;
	mylcd.Pixel = D_5x10;
	mylcd.RS_Pin = GPIO_PIN_N0_1;
	mylcd.RW_Pin = GPIO_PIN_N0_2;
	mylcd.pCommandport = GPIOB;
	mylcd.pdatalsb1 = GPIOA;
	mylcd.pdatamsb1 = GPIOD;

	LCD_Init(&mylcd);
}

void Gpio_Initialization()
{
	GPIO_Handle_t gpiodata;
	gpiodata.pGPIOx = GPIOD;
	gpiodata.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpiodata.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpiodata.GPIO_PinConfig.GPIO_Pinnumber = GPIO_PIN_N0_0;

	GPIO_Init(&gpiodata);

	gpiodata.GPIO_PinConfig.GPIO_Pinnumber = GPIO_PIN_N0_1;
	GPIO_Init(&gpiodata);

	gpiodata.GPIO_PinConfig.GPIO_Pinnumber = GPIO_PIN_N0_2;
	GPIO_Init(&gpiodata);

	gpiodata.GPIO_PinConfig.GPIO_Pinnumber = GPIO_PIN_N0_3;
	GPIO_Init(&gpiodata);

	GPIO_Handle_t gpiodata1;
	gpiodata1.pGPIOx = GPIOA;
	gpiodata1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpiodata1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpiodata1.GPIO_PinConfig.GPIO_Pinnumber = GPIO_PIN_N0_0;

	GPIO_Init(&gpiodata1);

	gpiodata1.GPIO_PinConfig.GPIO_Pinnumber = GPIO_PIN_N0_1;
	GPIO_Init(&gpiodata1);

	gpiodata1.GPIO_PinConfig.GPIO_Pinnumber = GPIO_PIN_N0_2;
	GPIO_Init(&gpiodata1);

	gpiodata1.GPIO_PinConfig.GPIO_Pinnumber = GPIO_PIN_N0_3;
	GPIO_Init(&gpiodata1);

	GPIO_Handle_t gpiocomm;
	gpiocomm.pGPIOx = GPIOB;
	gpiocomm.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpiocomm.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpiocomm.GPIO_PinConfig.GPIO_Pinnumber = GPIO_PIN_N0_0;
	GPIO_Init(&gpiocomm);

	gpiocomm.GPIO_PinConfig.GPIO_Pinnumber = GPIO_PIN_N0_1;
	GPIO_Init(&gpiocomm);

	gpiocomm.GPIO_PinConfig.GPIO_Pinnumber = GPIO_PIN_N0_2;
	GPIO_Init(&gpiocomm);
}



