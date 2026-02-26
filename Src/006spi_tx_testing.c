/*
 * 006spi_tx_testing.c
 *
 *  Created on: Nov 19, 2025
 *      Author: doubleup
 */

#include  "stm32f407xx.h"
#include <string.h>

//PB12 --> SPI2_NSS
//PB13 --> SPI2_SCK
//PB14 --> SPI2_MISO
//PB15 --> SPI2_MOSI
// ALT function mode: 5


void SPI2_GPIOInit(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;

	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

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
	SPI2Handle.SPIConfig.SPI_Sclkspeed = SPI_SCLK_SPEED_DIV2; // generates 8MHz on SCLK
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN; // software slave management enabled on NSS pin

	SPI_Init(&SPI2Handle);

}






int main(void)
{
	//user data buffer
	char user_data[] = "Hello world";

	// this function initializes GPIO pins to be used for SPI protocol
	SPI2_GPIOInit();

	// this function initializes SPI2 peripheral parameters
	SPI2_Init();

	//Pulls NSS signal high to bypass MODF error
	SPI_SSIConfig(SPI2, ENABLE);

	//Slave select output enable
	//SPI_SSOEConfig(SPI2, ENABLE);

	//enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));



	//confirm SPI Busy Flag not enabled before disabling
	while( SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG ) );

	//disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

	return 0;
}

