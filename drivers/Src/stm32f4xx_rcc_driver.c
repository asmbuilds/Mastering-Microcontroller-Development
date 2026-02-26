/*
 * stm32f4xx_rcc_driver.c
 *
 *  Created on: Jan 20, 2026
 *      Author: doubleup
 */

#include "stm32f4xx_rcc_driver.h"


uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};
uint8_t APB2_PreScaler[4] = {2,4,8,16};

uint32_t RCC_GetPLLOutputClock()
{
	return 0;
}


uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = ((RCC->RCC_CFGR >>2) & 0x3);

	if(clksrc == 0)
	{
		// HSI
		SystemClk = 16000000;
	}

	else if(clksrc == 1)
	{
		// HSE
		SystemClk = 8000000;
	}

	else if(clksrc == 2)
	{
		// PLL
		SystemClk = RCC_GetPLLOutputClock();
	}


	// for AHB prescaler

	temp = ((RCC->RCC_CFGR >> 4) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}

	else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	// for APB1 (low speed) prescaler

	temp = ((RCC->RCC_CFGR >> 10) & 0x7);

	if(temp < 4)
	{
		apb1p = 1;
	}

	else
	{
		apb1p = APB1_PreScaler[temp-4];
	}

	pclk1 = (SystemClk/ahbp) /apb1p;

	return pclk1;
}


uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2, SystemClk;

	uint8_t clksrc, temp, ahbp, apb2p;

	clksrc = ((RCC->RCC_CFGR >>2) & 0x3);

	if(clksrc == 0)
	{
		// HSI
		SystemClk = 16000000;
	}

	else if(clksrc == 1)
	{
		// HSE
		SystemClk = 8000000;
	}

	else if(clksrc == 2)
	{
		// PLL
		SystemClk = RCC_GetPLLOutputClock();
	}


	// for AHB prescaler

	temp = ((RCC->RCC_CFGR >> 4) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}

	else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	// for APB2 (high speed) prescaler

	temp = ((RCC->RCC_CFGR >> 13) & 0x7);

	if(temp < 4)
	{
		apb2p = 1;
	}

	else
	{
		apb2p = APB2_PreScaler[temp-4];
	}

	pclk2 = (SystemClk/ahbp) /apb2p;

	return pclk2;
}


