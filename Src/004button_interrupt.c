/*
 * 004button_interrupt.c
 *
 *  Created on: Nov 12, 2025
 *      Author: doubleup
 */


/*
 * 003led_button_ext.c
 *
 *  Created on: Nov 4, 2025
 *      Author: doubleup
 */


/*
 * 002led_button.c
 *
 *  Created on: Nov 4, 2025
 *      Author: doubleup
 */


#include "stm32f407xx.h"
#include <string.h>

#define HIGH         1
#define LOW          0
#define BTN_PRESSED  LOW


void delay(void)
{
	for(uint32_t i = 0; i < (500000/2); i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed, GpioBtn;

	memset(&GpioLed,0, sizeof(GpioLed));
	memset(&GpioBtn,0, sizeof(GpioBtn));

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);


	GpioBtn.pGPIOx = GPIOD;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;


	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioBtn);

	//IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);


	while(1);
}

void EXTI9_5_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
