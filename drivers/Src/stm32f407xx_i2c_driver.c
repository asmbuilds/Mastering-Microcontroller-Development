/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Dec 9, 2025
 *      Author: doubleup
 */
#include <stdint.h>
#include <stdio.h>
#include "stm32f407xx_i2c_driver.h"



static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);



static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1; // makes room for r/w bit

	// 7 bit slave address with 1 r/w bit (8 bits total)
	SlaveAddr &= ~(1);          // bit position zero set to 0 for WRITE

	pI2Cx->DR = SlaveAddr;      // pass slave address to DR
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1; // makes room for r/w bit

	// 7 bit slave address with 1 r/w bit (8 bits total)
	SlaveAddr |= 1;          // bit position zero set to 1 for READ

	pI2Cx->DR = SlaveAddr;      // pass slave address to DR
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		//device in MASTER mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				//first disable ACK
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//clear ADDR flag (READ SR1, READ SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
		}

		else
		{
			//clear ADDR flag (READ SR1, READ SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}
	}

	else
	{
		//device in SLAVE mode
		//clear ADDR flag (READ SR1, READ SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}



}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP);
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	else
	{
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
	}
}


// Peripheral clock setup
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}

		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}

		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}

	}

	else if(EnorDi == DISABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}

		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}

		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}

	}
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}

	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}






void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	// enable peripheral clock for I2Cx
	I2C_PeriClockControl(pI2CHandle->pI2Cx , ENABLE);

	//config ack control bit in CR1 register
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 |= tempreg;

	//configure FREQ bits/field in CR2 register
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 |= (tempreg & 0x3F);

	//program device own address in OAR1 register
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 |= tempreg;

	//configure CCR register (calculations)
	uint16_t ccr_value = 0;
	tempreg = 0;

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//standard mode
		ccr_value = (RCC_GetPCLK1Value() / (2*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	}

	else
	{
		//fast mode
		tempreg |= (1 << I2C_CCR_FS);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / (3*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}

		else
		{
			ccr_value = (RCC_GetPCLK1Value() / (25*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);

	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
		{
			//standard mode

			tempreg = (RCC_GetPCLK1Value() /1000000U) + 1;

		}

		else
		{
			//mode is fast mode
			tempreg = (RCC_GetPCLK1Value()*300 /1000000000U) + 1;
		}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

}

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}

	else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}

	else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}



void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. confirm completion of START condition by checking SB flag in SR1
	// SCL will be stretched (held low) until SB is cleared
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)); // performs READ on SR1

	//3. Send the address of the slave with r/w bit set to w(0) 8 bits total
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm address phase completion by checking ADDR flag in SR1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)); // performs READ on SR1

	//5.clear ADDR flag according to software sequence
	// SCL will be stretched until ADDR flag is cleared
	I2C_ClearADDRFlag(pI2CHandle);

	//6. send data until length = 0
	while(Len > 0)
	{
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); // hang until TXE set
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	//7. when Len = 0 wait for TXE = 1 and BTF = 1 before generating STOP condition
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); // hang until TXE set
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)); // hang until BTF set

	//8. generate STOP condition. BTF automatically cleared.
	if(Sr == I2C_DISABLE_SR)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	//1. Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm completion of START condition by checking SB flag in SR1
	//   SCL will be stretched (held low) until SB is cleared
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)); // performs READ on SR1

	//3. Send the address of the slave with r/w bit set to R(1) 8 bits total
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm address phase completion by checking ADDR flag in SR1S
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)); // performs READ on SR1


	//procedure to read only 1 byte from slave
	if(Len == 1)
	{
		//disable ACK
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);


		//clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until RXNE = 1
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		//generate STOP condition
		if(Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);


		//read data into buffer from data register
		*pRxbuffer = pI2CHandle->pI2Cx->DR;


	}

	//procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read data until Len = 0
		for(uint32_t i = Len; i > 0; i--)
		{
			//wait until RXNE = 1
			while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if(i == 2) // if last 2 bytes are remaining
			{
				//disable ACK
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//generate STOP condition
				if(Sr == I2C_DISABLE_SR)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			//read data from data register into data buffer
			*pRxbuffer = pI2CHandle->pI2Cx->DR;

			//increment data buffer address
			pRxbuffer++;
		}
	}


	//re-enable ACK
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}


}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == I2C_ACK_ENABLE)
	{
		//enable ACK
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}

	else
	{
		//disable ACK
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}


/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Complete the below code . Also include the function prototype in header file

 */
uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);


		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);


	}

	return busystate;

}

/*********************************************************************
 * @fn      		  - I2C_MasterReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the below code . Also include the fn prototype in header file

 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data)
{
	pI2C->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	return (uint8_t)pI2C->DR;
}

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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


void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// IPR register based on IRQ number

 	uint8_t iprx = IRQNumber / 4;

	// section of IPR register based on IRQ number

	uint8_t iprx_section = IRQNumber % 4;

	// set bit field for corresponding IRQ number
	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_BITS_IMPLEMENTED) ;
	*(NVIC_PR_BASE_ADDR + iprx) |= ( IRQPriority << shift_amount );

}

/*********************************************************************
 * @fn      		  - I2C_EV_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Interrupt handling for different I2C events (refer SR1)

 */

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	//disable ITEVTEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//disable ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		//disable ITEVTEN control bit
		pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

		pI2CHandle->TxRxState = I2C_READY;
		pI2CHandle->pTxBuffer = NULL;
		pI2CHandle->TxLen = 0;
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen > 0)
	{
		//1. load data into DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
		//2. decrement TxLen
		pI2CHandle->TxLen--;

		//3. increment buffer address
		pI2CHandle->pTxBuffer++;
	}
}


static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	//data reception
	if(pI2CHandle->RxSize == 1)
	{
		//read DR (pass data from DR to RxBuffer)
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			//clear ACK
			I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
		}

		//read DR (pass data from DR to RxBuffer)
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0)
	{
		//close I2C data reception and notify application

		//1. generate STOP condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//2. Close I2C Rx
		I2C_CloseReceiveData(pI2CHandle);

		//3.Notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);


	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

	if(temp1 && temp3)
	{
		//this interrupt is generated due to SB event (not executed in slave mode because SB = 0)
		//execute address phase in this block
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}

		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}

	}



	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);

	if(temp1 && temp3)
		{
			//interrupt generated by ADDR event
			I2C_ClearADDRFlag(pI2CHandle);
		}



	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);

	if(temp1 && temp3)
	{
		//set BTF flag
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//verify TXE also set
			if(pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE))
			{
				//BTF, TXE = 1
				if(pI2CHandle->TxLen == 0)
				{
					//1. generate STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					//2. reset all member elements of structure
					I2C_CloseSendData(pI2CHandle);

					//3. notify application transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}

		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			;
		}

	}


	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF); // READ operation on SR1

	if(temp1 && temp3)
		{
			// STOPF flag set, need to clear by READ on SR1 followed by WRITE on CR1
			// READ operation on SR1 is already completed above
			pI2CHandle->pI2Cx->CR1 |= 0x0000; // WRITE to CR1 without altering state

			// notify the application of STOP event
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
		}


	//5. Handle For interrupt generated by TXE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);

	if(temp1 && temp2 && temp3)
	{
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))// Check device mode (Master or Slave)
		{
			//master mode
			//TXE flag set, need to transmit data
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}

		else
		{
			//slave mode
			//verify slave in transmit mode
			if(pI2CHandle->pI2Cx->SR2 &(1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}


	//6. Handle For interrupt generated by RXNE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);

	if(temp1 && temp2 && temp3)
	{

		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))// Check device mode (Master or Slave)
		{
			//Device in MASTER mode
			//RXNE flag set, need to receive data
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);

			}
		}

		else
		{
			//slave mode
			//verify slave in receive mode
			if(pI2CHandle->pI2Cx->SR2 &(1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}


	}

}


/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the code also define these macros in the driver
						header file
						#define I2C_ERROR_BERR  3
						#define I2C_ERROR_ARLO  4
						#define I2C_ERROR_AF    5
						#define I2C_ERROR_OVR   6
						#define I2C_ERROR_TIMEOUT 7

 */

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}

