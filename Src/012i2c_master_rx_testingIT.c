/*
 * 012i2c_master_rx_testing.c
 *
 *  Created on: Dec 19, 2025
 *      Author: doubleup
 */

#include  "stm32f407xx.h"
#include <string.h>
#include <stdio.h>

extern void initialise_monitor_handles();

//flag variable
uint8_t rxCmplt = RESET;


#define MY_ADDRESS 0x61
#define SLAVE_ADDR 0x68

void delay(void)
{
	for(uint32_t i = 0; i < (500000/2); i++);
}

I2C_Handle_t I2C1Handle;

//receive data buffer
uint8_t rcv_buf[32];

//PB8-> SCL
//PB9-> SDA

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
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
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
	uint8_t commandcode;
	uint8_t len;

	initialise_monitor_handles();

	printf("Application is running\n");

	GPIO_ButtonInit();

	// I2C pin initialization
	I2C1_GPIOInit();

	// I2C peripheral initialization
	I2C1_Init();

	//I2C IRQ configs
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	// Enable I2C peripheral (PE = 1)
	I2C_PeripheralControl(I2C1, ENABLE);

	// Enable ACK bit
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while(1)
	{
		//wait for button press
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		//de-bounce with 200ms delay
		delay();

		commandcode = 0x51;

		while (I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SLAVE_ADDR,I2C_ENABLE_SR) != I2C_READY); //application hangs until ready

		while (I2C_MasterReceiveDataIT(&I2C1Handle, &len, 1, SLAVE_ADDR,I2C_ENABLE_SR) != I2C_READY); //application hangs until ready

		commandcode = 0x52;

		while (I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SLAVE_ADDR,I2C_ENABLE_SR) != I2C_READY); //application hangs until ready

		while (I2C_MasterReceiveDataIT(&I2C1Handle, rcv_buf, len, SLAVE_ADDR,I2C_ENABLE_SR) != I2C_READY); //application hangs until ready

		rxCmplt = RESET;

		// wait for Rx to complete
		while(rxCmplt != SET)
		{

		}

		rcv_buf[len+1] = '\0';

		printf("Data : %s", rcv_buf);

		rxCmplt = RESET;
	}



}


void I2C1_EV_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
	if(AppEv == I2C_EV_TX_CMPLT)
	{
		printf("Tx complete\n");
	}

	else if(AppEv == I2C_EV_RX_CMPLT)
	{
		printf("Rx complete\n");
		rxCmplt = SET;
	}

	else if(AppEv == I2C_ERROR_AF)
	{
		//ACK failure results when slave fails to send ACK for byte sent from master.

		printf("ACK failure\n");
		I2C_CloseSendData(pI2CHandle);

		// generate STOP condition to release bus
		I2C_GenerateStopCondition(I2C1);

		//hang in infinite loop
		while(1);
	}
}
