/*
 * stm32f407xx_gpio.c
 *
 *  Created on: Sep 10, 2025
 *      Author: Anis
 */

#include "stm32f407xx.h"

#define ENABLE		1
#define DISABLE		0

void GPIO_PeriClockControl(GPIO_TypeDef *pGPIOx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {
		if(pGPIOx == GPIOA)
		{
			RCC->AHB1ENR |= (1 << 0);
		}
		else if(pGPIOx == GPIOB)
		{
			RCC->AHB1ENR |= (1 << 1);
		}
		else if(pGPIOx == GPIOC)
		{
			RCC->AHB1ENR |= (1 << 2);
		}
		else if(pGPIOx == GPIOD)
		{
			RCC->AHB1ENR |= (1 << 3);
		}
		else if(pGPIOx == GPIOE)
		{
			RCC->AHB1ENR |= (1 << 4);
		}
		else if(pGPIOx == GPIOF)
		{
			RCC->AHB1ENR |= (1 << 5);
		}
		else if(pGPIOx == GPIOG)
		{
			RCC->AHB1ENR |= (1 << 6);
		}
		else if(pGPIOx == GPIOH)
		{
			RCC->AHB1ENR |= (1 << 7);
		}
		else if(pGPIOx == GPIOI)
		{
			RCC->AHB1ENR |= (1 << 8);
		}
	}
	else {
		if(pGPIOx == GPIOA)
			RCC->AHB1ENR &= ~(1 << 0 );
		else if(pGPIOx == GPIOB)
			RCC->AHB1ENR &= ~(1 << 1);
		else if(pGPIOx == GPIOC)
			RCC->AHB1ENR &= ~(1 << 2);
		else if(pGPIOx == GPIOD)
			RCC->AHB1ENR &= ~(1 << 3);
		else if(pGPIOx == GPIOE)
			RCC->AHB1ENR &= ~(1 << 4);
		else if(pGPIOx == GPIOF)
			RCC->AHB1ENR &= ~(1 << 5);
		else if(pGPIOx == GPIOG)
			RCC->AHB1ENR &= ~(1 << 6);
		else if(pGPIOx == GPIOH)
			RCC->AHB1ENR &= ~(1 << 7);
		else if(pGPIOx == GPIOI)
			RCC->AHB1ENR &= ~(1 << 8);
	}
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp;
//	1. Configure the mode a. analog b. interrupt
	if(pGPIOHandle->PinConfig.PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->PinConfig.PinMode << (2 * pGPIOHandle->PinConfig.PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->PinConfig.PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{	//interrupt mode
		if(pGPIOHandle->PinConfig.PinMode == GPIO_MODE_IT_FT)
		{
			EXTI->FTSR |= (1 << pGPIOHandle->PinConfig.PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->PinConfig.PinNumber);
		}
		else if(pGPIOHandle->PinConfig.PinMode == GPIO_MODE_IT_RT)
		{
			EXTI->FTSR &= ~(1 << pGPIOHandle->PinConfig.PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->PinConfig.PinNumber);
		}
		else if(pGPIOHandle->PinConfig.PinMode == GPIO_MODE_IT_FT)
		{
			EXTI->FTSR |= (1 << pGPIOHandle->PinConfig.PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->PinConfig.PinNumber);
		}

		//configure SYSCFG register
		uint8_t temp1 =	pGPIOHandle->PinConfig.PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->PinConfig.PinNumber % 4;

		uint8_t portcode = ((pGPIOHandle->pGPIOx == GPIOA) ? 0 :
							(pGPIOHandle->pGPIOx == GPIOB) ? 1 :
							(pGPIOHandle->pGPIOx == GPIOC) ? 2 :
							(pGPIOHandle->pGPIOx == GPIOD) ? 3 :
							(pGPIOHandle->pGPIOx == GPIOE) ? 4 :
							(pGPIOHandle->pGPIOx == GPIOF) ? 5 :
							(pGPIOHandle->pGPIOx == GPIOG) ? 6 :
							(pGPIOHandle->pGPIOx == GPIOH) ? 7 : 8 );

		RCC->APB2ENR |= (1 << 14);		//enabling syscfg register from rcc_apb2enr register
		SYSCFG->EXTICR[temp1] &= ~(0xF << (temp2 * 4));
		SYSCFG->EXTICR[temp1] |= (portcode << (temp2 * 4));

		//Unmasking the EXTI line
		EXTI->IMR |= (1 << pGPIOHandle->PinConfig.PinNumber);
	}
	//2. Configure the Speed
	temp = pGPIOHandle->PinConfig.PinSpeed << (2 * pGPIOHandle->PinConfig.PinNumber);
	pGPIOHandle->PinConfig.PinSpeed &= ~(0x3 << pGPIOHandle->PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	//3. Configure Pull-up Pull down
	temp = pGPIOHandle->PinConfig.PinPuPdControl << (2 * pGPIOHandle->PinConfig.PinNumber);
	pGPIOHandle->PinConfig.PinPuPdControl &= ~(0x3 << pGPIOHandle->PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	//4. Configure the Output Type Register
	temp = pGPIOHandle->PinConfig.PinOptType << pGPIOHandle->PinConfig.PinNumber;
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//5. Configure Alternate Function Register
	if(pGPIOHandle->PinConfig.PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t afr_index = pGPIOHandle->PinConfig.PinNumber / 8;
		uint8_t afr_pos	  = pGPIOHandle->PinConfig.PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[afr_index] &= ~(0xF << (4 * afr_pos));
		pGPIOHandle->pGPIOx->AFR[afr_index] |= (pGPIOHandle->PinConfig.PinAltFunMode << (4 * afr_pos));
	}
}

void GPIO_DeInit(GPIO_TypeDef *pGPIOx)
{
    if(pGPIOx == GPIOA) {
    	RCC->AHB1RSTR |= (1 << 0);
    	RCC->AHB1RSTR &= ~(1 << 0);
    }
    else if (pGPIOx == GPIOB)
    {
    	RCC->AHB1RSTR |= (1 << 1);
    	RCC->AHB1RSTR &= ~(1 << 1);
    }
    else if(pGPIOx == GPIOC)
    {
    	RCC->AHB1RSTR |= (1 << 2);
    	RCC->AHB1RSTR &= ~(1 << 2);
    }
    else if(pGPIOx == GPIOD)
    {
    	RCC->AHB1RSTR |= (1 << 3);
    	RCC->AHB1RSTR &= ~(1 << 3);
    }
    else if(pGPIOx == GPIOE)
    {
    	RCC->AHB1RSTR |= (1 << 4);
    	RCC->AHB1RSTR &= ~(1 << 4);
    }
    else if(pGPIOx == GPIOF) {
    	RCC->AHB1RSTR |= (1 << 5);
    	RCC->AHB1RSTR &= ~(1 << 5);
    }
    else if(pGPIOx == GPIOG) {
    	RCC->AHB1RSTR |= (1 << 6);
    	RCC->AHB1RSTR &= ~(1 << 6);
    }
    else if(pGPIOx == GPIOH) {
    	RCC->AHB1RSTR |= (1 << 7);
    	RCC->AHB1RSTR &= ~(1 << 7);
    }
    else if(pGPIOx == GPIOI) {
    	RCC->AHB1RSTR |= (1 << 8);
    	RCC->AHB1RSTR &= ~(1 << 8);
    }
}


uint8_t GPIO_ReadFromInputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber)
{
	return (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x1);
}


uint16_t GPIO_ReadFromInputPort(GPIO_TypeDef *pGPIOx)
{
	return (uint16_t) (pGPIOx->IDR);
}


void GPIO_WriteToOutputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if(Value == 1)
        pGPIOx->ODR |= (1 << PinNumber);
    else
        pGPIOx->ODR &= ~(1 << PinNumber);
}

void GPIO_WriteToOutputPort(GPIO_TypeDef *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
}

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        if(IRQNumber <= 31)
            *NVIC_ISER0 |= (1 << IRQNumber);
        else if(IRQNumber > 31 && IRQNumber < 64)
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        else if(IRQNumber >= 64 && IRQNumber < 96)
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
    }
    else
    {
        if(IRQNumber <= 31)
            *NVIC_ICER0 |= (1 << IRQNumber);
        else if(IRQNumber > 31 && IRQNumber < 64)
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        else if(IRQNumber >= 64 && IRQNumber < 96)
            *NVIC_ICER2 |= (1 << (IRQNumber % 64));
    }
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx      = IRQNumber / 4;
    uint8_t iprx_sec  = IRQNumber % 4;
    uint8_t shift_amt = (8 * iprx_sec) + (8 - 4); // only upper 4 bits used

    NVIC_PR_BASE_ADDR[ iprx ] &= ~(0xF << shift_amt);
    NVIC_PR_BASE_ADDR[ iprx ] |= (IRQPriority << shift_amt);
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
    if(EXTI->PR & (1 << PinNumber))
    {
        // clear
        EXTI->PR |= (1 << PinNumber);
    }
}
