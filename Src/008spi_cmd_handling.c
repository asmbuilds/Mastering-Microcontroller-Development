/*
 * 008spi_cmd_handling.c
 *
 *  Created on: Nov 28, 2025
 *      Author: doubleup
 */
#include  "stm32f407xx.h"
#include <string.h>
#include <stdio.h>

//command codes
#define COMMAND_LED_CTRL             0x50
#define COMMAND_SENSOR_READ          0x51
#define COMMAND_LED_READ             0x52
#define COMMAND_PRINT                0x53
#define COMMAND_ID_READ              0x54

#define LED_ON      1
#define LED_OFF     0

//arduino 'ANALOG IN' pins
#define ANALOG_PIN0       0
#define ANALOG_PIN1       1
#define ANALOG_PIN2       2
#define ANALOG_PIN3       3
#define ANALOG_PIN4       4
#define ANALOG_PIN5       5

//arduino LED
#define LED_PIN       9

//PB12 --> SPI2_NSS
//PB13 --> SPI2_SCK
//PB14 --> SPI2_MISO
//PB15 --> SPI2_MOSI
// ALT function mode: 5



void delay(void)
{
	for(uint32_t i = 0; i < (500000/2); i++);
}

void SPI2_GPIOInit(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;

	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPIPins);
}

void SPI2_Init(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;

	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_Sclkspeed = SPI_SCLK_SPEED_DIV8; // generates 2MHz on SCLK
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // software slave management disabled on NSS pin

	SPI_Init(&SPI2Handle);

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

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
	{
		return 1;
	}

	return 0;
}

int main(void)
{
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	GPIO_ButtonInit();

	// this function initializes GPIO pins to be used for SPI protocol
	SPI2_GPIOInit();

	// this function initializes SPI2 peripheral parameters
	SPI2_Init();



	//Pulls NSS signal high to bypass MODF error
	SPI_SSIConfig(SPI2, ENABLE);

	//Slave select output enable
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
        //wait for button press
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		//de-bounce with 200ms delay
		delay();

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//1. COMMAND_LED_CTRL   <pin no(1)>      <value(1)>
		uint8_t cmndcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t params[2];

		//send command
		SPI_SendData(SPI2, &cmndcode, 1);

		//read dummy to clear RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//send dummy byte to fetch slave response
		SPI_SendData(SPI2, &dummy_write, 1);

		//read ackbyte if received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if( SPI_VerifyResponse(ackbyte) )
		{

			params[0] = LED_PIN;
			params[1] = LED_ON;

			//send parameters (2 bytes of data)
			SPI_SendData(SPI2, params, 2);
		}// end CMD_LED_CTRL


		//2. COMMAND_SENSOR_READ     <analog pin number(1) >

		//wait for button press
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		//de-bounce with 200ms delay
		delay();

		cmndcode = COMMAND_SENSOR_READ;

		//send command
		SPI_SendData(SPI2, &cmndcode, 1);

		//read dummy to clear RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//send dummy byte to fetch slave response
		SPI_SendData(SPI2, &dummy_write, 1);

		//read ackbyte if received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if( SPI_VerifyResponse(ackbyte) )
		{

			params[0] = ANALOG_PIN0;

			//send parameter (1 byte of data)
			SPI_SendData(SPI2, params, 1);

			//read dummy to clear RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//insert delay for ADC processing time
			delay();

			//send dummy byte to fetch slave response
			SPI_SendData(SPI2, &dummy_write, 1);


			uint8_t analog_read;
			SPI_ReceiveData(SPI2,&analog_read,1);
		}// end COMMAND_SENSOR_READ


		//3.  CMD_LED_READ 	 <pin no(1) >

		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		cmndcode = COMMAND_LED_READ;

		//send command
		SPI_SendData(SPI2,&cmndcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		if( SPI_VerifyResponse(ackbyte))
		{
			params[0] = LED_PIN;

			//send arguments
			SPI_SendData(SPI2,params,1); //sending one byte

			//clear RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI2,&dummy_write,1);

			uint8_t led_status;
			SPI_ReceiveData(SPI2,&led_status,1);
			printf("COMMAND_READ_LED %d\n",led_status);

		}

		//4. CMD_PRINT 		<len(2)>  <message(len) >

		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		cmndcode = COMMAND_PRINT;

		//send command
		SPI_SendData(SPI2,&cmndcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		uint8_t message[] = "Hello ! How are you ??";
		if( SPI_VerifyResponse(ackbyte))
		{
			params[0] = strlen((char*)message);

			//send arguments
			SPI_SendData(SPI2,params,1); //sending length

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			delay();

			//send message
			for(int i = 0 ; i < params[0] ; i++){
				SPI_SendData(SPI2,&message[i],1);
				SPI_ReceiveData(SPI2,&dummy_read,1);
			}

			printf("COMMAND_PRINT Executed \n");

		}

		//5. CMD_ID_READ
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		cmndcode = COMMAND_ID_READ;

		//send command
		SPI_SendData(SPI2,&cmndcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		uint8_t id[11];
		uint32_t i=0;

		if( SPI_VerifyResponse(ackbyte))
		{
			//read 10 bytes id from the slave
			for(  i = 0 ; i < 10 ; i++)
			{
				//send dummy byte to fetch data from slave
				SPI_SendData(SPI2,&dummy_write,1);
				SPI_ReceiveData(SPI2,&id[i],1);
			}

			id[10] = '\0';

			printf("COMMAND_ID : %s \n",id);

		}


		//confirm SPI Busy Flag not enabled before disabling
		while( SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG) );

		//disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

	}

	return 0;
}


