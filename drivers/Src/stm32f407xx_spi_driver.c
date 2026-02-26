/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Nov 15, 2025
 *      Author: doubleup
 */
#include "stm32f407xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/* @fn                         -GPIO_WriteToOutputPin
 *
 * @breif                      -This function writes data to a specified GPIO pin
 *
 * @param[in]                  -base address of GPIO peripheral
 *
 * @return                     -none
 *
 * @Note                       -none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	//enable peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);


	// configure SPI_CR1 register
	uint32_t tempreg = 0;

	//1. configure SPI_DeviceMode
	tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	//2. configure SPI_BusConfig
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//clear BIDI mode
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}

	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//set BIDI mode
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}

	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//clear BIDI mode and set RXONLY mode
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= (1 << SPI_CR1_RXONLY);
	}
	//3. configure SPI_Sclkspeed
	tempreg |= (pSPIHandle->SPIConfig.SPI_Sclkspeed << SPI_CR1_BR);

	//4. configure SPI_DFF
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	//5. configure SPI_CPOL
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	//6. configure SPI_CPHA
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	//7. configure SPI_SSM
	tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

    // clear lower 15 bits of SPI_CR1 register
	pSPIHandle->pSPIx->CR1 &= ~0x7FFF;

	// set lower 15 bits of SPI_CR1 register
	pSPIHandle->pSPIx->CR1 |= tempreg;

}

/* @fn                         -SPI_DeInit(SPI_RegDef_t *pSPIx)
 *
 * @breif                      -This function triggers a reset on RCC APB2 peripheral reset register (RCC_APB2RSTR) for SPI1
 *                              and RCC APB1 peripheral reset register (RCC_APB1RSTR) for SPI2 and SPI3
 *
 * @param[in]                  -base address of SPIx peripheral
 *
 * @return                     -none
 *
 * @Note                       -none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}

	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}

	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

// Peripheral clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}

			else if(pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}

			else if(pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}

		}

	else if(EnorDi == DISABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}

		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}

		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}

	}
}

// Data send and receive

/* @fn                         -SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
 *
 * @breif                      -This function sends data of specified length in 16 or 8 bit format to Tx buffer.
 *
 *
 * @param[in]                  -base address of SPIx peripheral
 * @param[in]                  -base address of TxBuffer
 * @param[in]                  -length/number of bits to send
 *
 * @return                     -none
 *
 * @Note                       -this API is a blocking call. Function call does not return until all bytes transmit.
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait for TXE set. When TXE is set the Tx buffer is empty and ready to receive data to transmit.
		// program is polling for the TXE flag to set.
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2. check the DFF bit in CR1 for 8 or 16 bit format
		if( (pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 bit DFF
			// load 16 bit data into DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer ++;
		}

		else
		{
			//8 bit DFF
			// load 8 bit data into DR
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer ++;
		}

	}

}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait for RXNE set. When RXNE is set the Rx buffer is  not empty and ready to receive transmitted data.
		// program is polling for the RXNE flag to set.
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//2. check the DFF bit in CR1 for 8 or 16 bit format
		if( (pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 bit DFF
			// load 16 bit data from DR into Rx buffer
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer ++;
		}

		else
		{
			//8 bit DFF
			// load 8 bit data from DR into Rx buffer
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer ++;
		}

	}

}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{

	//1. save Tx buffer address and length in global variables
	pSPIHandle->pTxBuffer = pTxBuffer;
	pSPIHandle->TxLen = Len;

	//2. set SPI state to busy for transmission to finish before executing next code block
	pSPIHandle->TxState = SPI_BUSY_IN_TX;

	//3. enable TXEIE control bit to get interrupt whenever TXE flag is set in SR
	pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}

	//4. ISR code will handle data transmission

	return state;
}


uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{

	//1. save Tx buffer address and length in global variables
	pSPIHandle->pRxBuffer = pRxBuffer;
	pSPIHandle->RxLen = Len;

	//2. set SPI state to busy for transmission to finish before executing next code block
	pSPIHandle->RxState = SPI_BUSY_IN_RX;

	//3. enable RXNEIE control bit to get interrupt whenever RXNE flag is set in SR
	pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}

	//4. ISR code will handle data transmission

	return state;
}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;

	//1. check for TXE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	//2. check for RXNE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	//3. check for OVR flag
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		//handle OVR error
		spi_ovr_err_interrupt_handle(pHandle);
	}

}


// IRQ configuration and ISR handling

/* @fn                         -SPI_IRQInterruptConfig
 *
 * @breif                      -This function configures SPI peripheral for interrupts
 *
 * @param[in]                  -IRQ number
 * @param[in]                  -enable or disable
 *
 * @return                     -none
 *
 * @Note                       -none
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );
		}

		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( (1 << IRQNumber) % 32 );
		}

		else if(IRQNumber > 64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= ( (1 << IRQNumber) % 64 );
		}
	}

	else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}

		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER0 |= ( (1 << IRQNumber % 32) );
		}

		else if(IRQNumber > 64 && IRQNumber < 96)
		{
			//program ICER2 register
			*NVIC_ICER0 |= ( (1 << IRQNumber % 64) );
		}
	}

}



/* @fn                         -GPIO_IRQPriorityConfig
 *
 * @breif                      -This function
 *
 * @param[in]                  -IRQNumber
 * @param[in]                  -IRQPriority
 *
 * @return                     -none
 *
 * @Note                       -none
 */

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// IPR register based on IRQ number

 	uint8_t iprx = IRQNumber / 4;

	// section of IPR register based on IRQ number

	uint8_t iprx_section = IRQNumber % 4;

	// set bit field for corresponding IRQ number
	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_BITS_IMPLEMENTED) ;
	*(NVIC_PR_BASE_ADDR + iprx) |= ( IRQPriority << shift_amount );

}


void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}

	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}

	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}

	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

// helper functions
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check the DFF bit in CR1 for 8 or 16 bit format
	if( (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
	{
		//16 bit DFF
		// load 16 bit data into DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer ++;
	}

	else
	{
		//8 bit DFF
		// load 8 bit data into DR
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer ++;
	}

	if(! pSPIHandle->TxLen)
	{
		// TxLen = 0 so SPI transmission should be closed and application informed Tx complete.
		// stop interrupts from setting TXE flag

		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}

}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check the DFF bit in CR1 for 8 or 16 bit format
	if( (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
	{
		//16 bit DFF
		// load 16 bit data into DR
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t*)pSPIHandle->pRxBuffer ++;
	}

	else
	{
		//8 bit DFF
		// load 8 bit data into DR
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer ++;
	}

	if(! pSPIHandle->RxLen)
	{
		// TxLen = 0 so SPI transmission should be closed and application informed Tx complete.
		// stop interrupts from setting TXE flag

		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	uint8_t temp;
	//1. clear OVR flag by a read access to SPI_DR register followed by read access to
	// SPI_SR register.

	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}

    (void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

	//This is a weak implementation . the user application may override this function.
}
