/*
 * 01i2c_slave_tx_string.c
 *
 *  Created on: Dec 19, 2025
 *      Author: doubleup
 */

#include  "stm32f407xx.h"
#include <string.h>
#include <stdio.h>




#define SLAVE_ADDR 0x68
#define MY_ADDR    SLAVE_ADDR

void delay(void)
{
	for(uint32_t i = 0; i < (500000/2); i++);
}

I2C_Handle_t I2C1Handle;

//receive data buffer
uint8_t Tx_buf[32] = "STM32 slave mode test...";

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
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
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

	//I2C IRQ configs
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_SlaveEnableDisableCallbackEvents(I2C1,ENABLE);

	// Enable I2C peripheral (PE = 1)
	I2C_PeripheralControl(I2C1, ENABLE);

	// Enable ACK bit
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while(1);

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
	static uint8_t commandCode = 0;
	static uint8_t Cnt = 0;

	if(AppEv == I2C_EV_DATA_REQ)
	{
		//master requests slave to send data...
		if (commandCode == 0x51)
		{
			//send length info to master
			I2C_SlaveSendData(pI2CHandle->pI2Cx,strlen((char*)Tx_buf));
		}
		else if (commandCode == 0x52)
		{
			//send data stream from Tx_buf
			I2C_SlaveSendData(pI2CHandle->pI2Cx,Tx_buf[Cnt++]);
		}
	}

	else if(AppEv == I2C_EV_DATA_RCV)
	{
		//data in slave read queue...waiting for slave read
		commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
	}

	else if(AppEv == I2C_ERROR_AF)
	{
		//event only happens during slave Tx
		//slave Tx...master sent NACK...data stream ends
		commandCode = 0xff;
		Cnt = 0;
	}

	else if(AppEv == I2C_EV_STOP)
	{
		//event only happens during slave Rx
		//Master ended I2C data stream with slave
	}
}
