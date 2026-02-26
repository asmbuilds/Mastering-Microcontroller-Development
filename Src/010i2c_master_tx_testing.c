/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: Dec 19, 2025
 *      Author: doubleup
 */

#include  "stm32f407xx.h"
#include <string.h>
#include <stdio.h>

#define MY_ADDRESS 0x61
#define SLAVE_ADDR 0x68

void delay(void)
{
	for(uint32_t i = 0; i < (500000/2); i++);
}

I2C_Handle_t I2C1Handle;

//data to send
uint8_t some_data[] = "We are testing I2C master Tx\n";

//PB6-> SCL
//PB7-> SDA

// this function codes GPIO pins PB6 and PB9 to behave as I2C pins
void I2C1_GPIOInit(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);



}

void I2C1_Init(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDRESS;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GpioBtn;

	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_Init(&GpioBtn);
}


int main(void)
{
	GPIO_ButtonInit();

	// I2C pin initialization
	I2C1_GPIOInit();

	// I2C peripheral initialization
	I2C1_Init();

	// Enable I2C peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	while(1)
	{
		//wait for button press
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		//de-bounce with 200ms delay
		delay();

		// Send data to slave
		I2C_MasterSendData(&I2C1Handle, some_data, strlen((char*)some_data), SLAVE_ADDR);
	}



}
