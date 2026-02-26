/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Oct 27, 2025
 *      Author: doubleup
 */


#include "stm32f407xx_gpio_driver.h"



// Init and De-Init
/***********************************************************************
 * @fn                         -GPIO_Init
 *
 * @breif                      -This function initializes a specified GPIO pin
 *
 * @param[in]                  -base address of GPIOHandle struct
 *
 * @return                     -none
 *
 * @Note                       -none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	//enable peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. config GPIO pin mode

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clear specified register bits
		pGPIOHandle->pGPIOx->MODER |= temp; // set specified register bits


	}

	else
	{
		// code for interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// configure the FTSR bit
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		    // clear the corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// configure the RTSR bit
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// clear the corresponding FTSR bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// configure both the FTSR and the RTSR

			// configure the RTSR bit
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// configure FTSR bit
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		// enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	//2. config speed
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	        pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clear specified register bits
			pGPIOHandle->pGPIOx->OSPEEDR |= temp;  //set specified register bits

	temp = 0;

	//3. config pu/pd settings
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	        pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clear specified register bits
			pGPIOHandle->pGPIOx->PUPDR |= temp;  //set specified register bits

	temp = 0;


	//4. config output type
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	        pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clear specified register bits
			pGPIOHandle->pGPIOx->OTYPER |= temp;  //set specified register bits

	temp = 0;

	//5. config alternate functionality (if needed)
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// config alt function registers
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8; // result is 0 (low register) or 1 (high register).
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8; // result is 0,1,2,3,4,5,6 or 7. Used to determine port in high or low register.
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); // clear specified register bits
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2) ); //set specified register bits

	}
}
/***********************************************************************
 * @fn                         -GPIO_DeInit
 *
 * @breif                      -This function resets all registers of specified GPIO port
 *
 * @param[in]                  -base address of GPIO peripheral
 *
 * @return                     -none
 *
 * @Note                       -none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)

{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}

	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}

	else if(pGPIOx == GPIOC)
	{
		GPIOD_REG_RESET();
	}

	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}

	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}

	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}

	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}

	else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}

}

// Peripheral clock setup
/***********************************************************************
 * @fn                         -GPIO_PeriClockControl
 *
 * @breif                      -This function enables or disables peripheral clock for specified GPIO port
 *
 * @param[in]                  -base address of GPIO peripheral
 * @param[in]                  -ENABLE or DISABLE macros
 *
 * @return                     -none
 *
 * @Note                       -none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}

		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}

		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}

		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}

		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}

		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}

		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}

		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}

		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}

	else if(EnorDi == DISABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}

		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}

		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}

		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}

		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}

		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}

		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}

		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}

		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}


// Data read and write
/* @fn                         -GPIO_ReadFromInputPin
 *
 * @breif                      -This function reads input from a specified GPIO pin
 *
 * @param[in]                  -base address of GPIO peripheral
 * @param[in]                  -GPIO pin number to read
 *
 * @return                     -0 or 1
 *
 * @Note                       -none
 */
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001 );
	return value;
}

/* @fn                         -GPIO_ReadFromInputPort
 *
 * @breif                      -This function reads input from a specified GPIO port
 *
 * @param[in]                  -base address of GPIO peripheral
 * @param[in]                  -port to be read
 *
 * @return                     -port data
 *
 * @Note                       -none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
		value = (uint16_t)pGPIOx->IDR;
		return value;
}


/* @fn                         -GPIO_WriteToOutputPin
 *
 * @breif                      -This function writes data to a specified GPIO pin
 *
 * @param[in]                  -base address of GPIO peripheral
 * @param[in]                  -GPIO pin number
 * @param[in]                  -set or reset
 *
 * @return                     -none
 *
 * @Note                       -none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= ( 1 << PinNumber);
	}

	else
	{
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}
}


/* @fn                         -GPIO_WriteToOutputPort
 *
 * @breif                      -This function writes data to a specified GPIO port
 *
 * @param[in]                  -base address of GPIO peripheral
 * @param[in]                  -set or reset
 *
 * @return                     -none
 *
 * @Note                       -none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx ,uint8_t Value)
{
	pGPIOx->ODR = Value;
}


/* @fn                         -GPIO_ToggleOutputPin
 *
 * @breif                      -This function toggles a specified GPIO pin
 *
 * @param[in]                  -base address of GPIO peripheral
 * @param[in]                  -GPIO pin number to toggle
 *
 * @return                     -none
 *
 * @Note                       -none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx-> ODR  ^= (1 << PinNumber);
}

// IRQ configuration and ISR handling

/* @fn                         -GPIO_IRQInterruptConfig
 *
 * @breif                      -This function configures GPIO peripheral for interrupts
 *
 * @param[in]                  -IRQ number
 * @param[in]                  -enable or disable
 *
 * @return                     -none
 *
 * @Note                       -none
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
			*NVIC_ICER0 = ( 1 << IRQNumber );
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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// IPR register based on IRQ number

 	uint8_t iprx = IRQNumber / 4;

	// section of IPR register based on IRQ number

	uint8_t iprx_section = IRQNumber % 4;

	// set bit field for corresponding IRQ number
	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_BITS_IMPLEMENTED) ;
	*(NVIC_PR_BASE_ADDR + iprx) |= ( IRQPriority << shift_amount );

}

/* @fn                         -GPIO_IRQHandling
 *
 * @breif                      -This function applies IRQ handling to a specified GPIO pin
 *
 * @param[in]                  -GPIO pin number
 *
 * @return                     -none
 *
 * @Note                       -none
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// clear the EXTI pr register corresponding to pin number
	if(EXTI->PR & ( 1 << PinNumber))
	{
		//clear
		EXTI->PR |= ( 1 << PinNumber);
	}
}


