/*
 * stm32f407xx_i2c.c
 *
 *  Created on: Sep 18, 2025
 *      Author: Anis
 */
#include "stm32f407xx.h"
#include "stm32f407xx_i2c.h"
#include "stm32f407xx_gpio.h"
#include <stddef.h>
#include <stdint.h>

/* Peripheral Clock control */
void I2C_PeriClockControl(I2C_TypeDef *pI2Cx, uint8_t EnOrDi)
{
	 if(EnOrDi == ENABLE)
	    {
	        if(pI2Cx == I2C1)
	            RCC->APB1ENR |= (1 << 21);
	        else if(pI2Cx == I2C2)
	            RCC->APB1ENR |= (1 << 22);
	        else if(pI2Cx == I2C3)
	            RCC->APB1ENR |= (1 << 23);
	    }
	    else
	    {
	        if(pI2Cx == I2C1)
	            RCC->APB1ENR &= ~(1 << 21);
	        else if(pI2Cx == I2C2)
	            RCC->APB1ENR &= ~(1 << 22);
	        else if(pI2Cx == I2C3)
	            RCC->APB1ENR &= ~(1 << 23);
	    }
}

/* RCC runtime APB1 helper */
uint32_t RCC_GetPCLK1Value(void)
{
    uint32_t sysclk_mhz = 16U * 1000000U; 					/* default assume HSI=16MHz */
    uint32_t sws = (RCC->CFGR >> 2) & 0x3;

    if(sws == 0)
    {
        /* HSI used as system clock */
        sysclk_mhz = 16000000U;
    } else if(sws == 1)
    {
        /* HSE used - many boards use 8 MHz HSE, but this may vary */
        sysclk_mhz = 8000000U;
    } else if(sws == 2)
    {
        /* PLL used -> fallback to 168 MHz common for F407 when PLL used.*/
        sysclk_mhz = 168000000U; 							/* common default; change if needed */
    }

    /* AHB prescaler */
    uint32_t hpre = (RCC->CFGR >> 4) & 0xF;
    uint32_t ahb_div = 1;
    if(hpre & 0x8)
    {
        /* AHB prescaler when bits 7:4 >= 8 -> real divide value table */
        uint8_t ahb_table[] = {2,4,8,16,64,128,256,512};
        ahb_div = ahb_table[hpre & 0x7];
    }

    uint32_t hclk = sysclk_mhz / ahb_div;

    /* APB1 prescaler */
    uint32_t ppre1 = (RCC->CFGR >> 10) & 0x7;
    uint32_t apb1_div = 1;
    if(ppre1 & 0x4)
    {
        uint8_t apb1_table[] = {2,4,8,16};
        apb1_div = apb1_table[ppre1 & 0x3];
    }

    uint32_t pclk1 = hclk / apb1_div;
    return pclk1;
}



/* Init */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
    uint32_t tempreg = 0;
    uint32_t pclk1 = 0;

    /* Enable clock */
    I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

    /* Configure ACK control in CR1 (bit10) */
    tempreg = (pI2CHandle->pI2C_Config->I2C_AckControl << 10);
    pI2CHandle->pI2Cx->CR1 &= ~(1 << 10);
    pI2CHandle->pI2Cx->CR1 = tempreg;

    /* Configure frequency (FREQ field in CR2) using runtime APB1 clock (in MHz) */
    pclk1 = RCC_GetPCLK1Value();
    uint32_t freq = (pclk1 / 1000000U) & 0x3F;
    tempreg = (pclk1 / 1000000U) & 0x3F;
    pI2CHandle->pI2Cx->CR2 &= ~0x3F;
    pI2CHandle->pI2Cx->CR2 |= freq;

    /* Own address */
    uint32_t oar1 = (pI2CHandle->pI2C_Config->I2C_DeviceAddress << 1);
    oar1 |= (1 << 14);
    pI2CHandle->pI2Cx->OAR1 = oar1;

    /* CCR calculation */
    uint16_t ccr_value = 0;
    if(pI2CHandle->pI2C_Config->I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        /* standard mode */
        ccr_value = (uint16_t)(pclk1 / (2 * pI2CHandle->pI2C_Config->I2C_SCLSpeed));
        pI2CHandle->pI2Cx->CCR &= ~0xFFF;
        pI2CHandle->pI2Cx->CCR |= (ccr_value & 0xFFF);
    }
    else
    {
        // fast mode
        pI2CHandle->pI2Cx->CCR |= (1 << 15);
        pI2CHandle->pI2Cx->CCR &= ~(1 << 14);
        pI2CHandle->pI2Cx->CCR |= (pI2CHandle->pI2C_Config->I2C_FMDutyCycle << 14);

        if(pI2CHandle->pI2C_Config->I2C_FMDutyCycle == I2C_FM_DUTY_2)
        	ccr_value = (uint16_t)(pclk1 / (3 * pI2CHandle->pI2C_Config->I2C_SCLSpeed));
        else
        	ccr_value = (uint16_t)(pclk1 / (25 * pI2CHandle->pI2C_Config->I2C_SCLSpeed));

        pI2CHandle->pI2Cx->CCR &= ~0xFFF;
        pI2CHandle->pI2Cx->CCR |= (ccr_value & 0xFFF);
    }

    /* TRISE configuration */
    /* TRISE */
    if(pI2CHandle->pI2C_Config->I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
        pI2CHandle->pI2Cx->TRISE = (uint32_t)((pclk1/1000000U) + 1);
    else
        pI2CHandle->pI2Cx->TRISE = (uint32_t)(((pclk1 * 300U)/1000000000U) + 1);
}

/* DeInit */
void I2C_DeInit(I2C_TypeDef *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		RCC->APB1RSTR |= (1 << 21);
		RCC->APB1RSTR &= ~(1 << 21);
	}
	else if(pI2Cx == I2C2)
	{
		RCC->APB1RSTR |= (1 << 22);
		RCC->APB1RSTR &= ~(1 << 22);
	}
	else if(pI2Cx == I2C3)
	{
		RCC->APB1RSTR |= (1 << 23);
		RCC->APB1RSTR &= ~(1 << 23);
	}
}

/* Master send data (blocking) */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	//1. Generate START
	pI2CHandle->pI2Cx->CR1 |= (1 << 8);

	//2. Wait for Start Bit (SB) in master mode
	while(!(pI2CHandle->pI2Cx->SR1 & I2C_FLAG_SB));

	//3. Send Address
	pI2CHandle->pI2Cx->DR = (SlaveAddr << 1);

	//4. Wait for ADDR
	while(!(pI2CHandle->pI2Cx->SR1 & I2C_FLAG_ADDR));
	(void)pI2CHandle->pI2Cx->SR1;
	(void)pI2CHandle->pI2Cx->SR2;

	//5. Send Data
	while(Len > 0)
	{
		while(!(pI2CHandle->pI2Cx->SR1 & I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//6. Wait for Byte Transfer Finished (BTF)
	while(!(pI2CHandle->pI2Cx->SR1 & I2C_FLAG_BTF));

	//7. Generate STOP
	if(Sr == 0)
	{
		pI2CHandle->pI2Cx->CR1 |= (1 << 9);
	}
}

/* Master receive data (blocking) */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	//1. Generate START
	pI2CHandle->pI2Cx->CR1 |= (1 << 8);

	//2. Wait for Start Bit (SB) in master mode
	while(!(pI2CHandle->pI2Cx->SR1 & I2C_FLAG_SB));

	//3. Send Address
	pI2CHandle->pI2Cx->DR = (SlaveAddr << 1) | 1;

	//4. Wait for ADDR
	while(!(pI2CHandle->pI2Cx->SR1 & I2C_FLAG_ADDR));

	if(Len == 1)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		(void)pI2CHandle->pI2Cx->SR1;
		(void)pI2CHandle->pI2Cx->SR2;

		pI2CHandle->pI2Cx->CR1 |= (1 << 9); //STOP

		while(!(pI2CHandle->pI2Cx->SR1 & I2C_FLAG_RXNE));
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}
	else
	{
		(void)pI2CHandle->pI2Cx->SR1;
		(void)pI2CHandle->pI2Cx->SR2;

		while(Len > 0)
		{
			while(!(pI2CHandle->pI2Cx->SR1 & I2C_FLAG_RXNE));
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			pRxBuffer++;
			Len--;

			if(Len == 1)
			{
                I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
                pI2CHandle->pI2Cx->CR1 |= (1 << 9); // STOP
			}
		}
	}

	if(pI2CHandle->pI2C_Config->I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

/* Peripheral enable/disable */
void I2C_PeripheralControl(I2C_TypeDef *pI2Cx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
        pI2Cx->CR1 |= (1 << 0);
    else
        pI2Cx->CR1 &= ~(1 << 0);
}

void I2C_ManageAcking(I2C_TypeDef *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << 10);

	}
	else
	{
		pI2Cx->CR1 &= ~(1 << 10);
	}
}

/* Flag status */
uint8_t I2C_GetFlagStatus(I2C_TypeDef *pI2Cx, uint32_t FlagName)
{
    if(pI2Cx->SR1 & FlagName)
        return SET;
    return RESET;
}


/* ------------------ NVIC helpers ------------------ */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        if(IRQNumber <= 31)
            *NVIC_ISER0 |= (1 << IRQNumber);
        else if(IRQNumber <= 63)
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        else if(IRQNumber <= 95)
            *NVIC_ISER2 |= (1 << (IRQNumber % 32));
    }
    else
    {
        if(IRQNumber <= 31)
            *NVIC_ICER0 |= (1 << IRQNumber);
        else if(IRQNumber <= 63)
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        else if(IRQNumber <= 95)
            *NVIC_ICER2 |= (1 << (IRQNumber % 32));
    }
}

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;
    uint32_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/* ------------------ Non-blocking (IT) APIs ------------------ */

/* Start I2C transmission in interrupt mode
 * returns 0 if accepted, 1 if busy */

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TXRXState;
	if(busystate != I2C_READY)
		return 1;					/* it means busy */

	pI2CHandle->pTxBuffer = pTxBuffer;
	pI2CHandle->TXLen = Len;
	pI2CHandle->DevAddr = SlaveAddr;
	pI2CHandle->Sr = Sr;
	pI2CHandle->TXRXState = I2C_BUSY_IN_TX;

	/* Enable ITERREN (error), ITEVTEN (event) and ITBUFEN (buffer) interrupts */
	pI2CHandle->pI2Cx->CR2 |= I2C_CR2_ITERREN | I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN;

    /* Generate START */
    pI2CHandle->pI2Cx->CR1 |= (1 << 8);

    return 0;
}

/* Start master reception in interrupt mode */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TXRXState;
    if(busystate != I2C_READY)
        return 1; /* busy */

    pI2CHandle->pRxBuffer = pRxBuffer;
    pI2CHandle->RXLen = Len;
    pI2CHandle->DevAddr = SlaveAddr;
    pI2CHandle->Sr = Sr;
    pI2CHandle->RXSize = Len;
    pI2CHandle->TXRXState = I2C_BUSY_IN_RX;

    /* Enable ITERREN (error), ITEVTEN (event) and ITBUFEN (buffer) interrupts */
    pI2CHandle->pI2Cx->CR2 |= I2C_CR2_ITERREN | I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN;

    /* Generate START */
    pI2CHandle->pI2Cx->CR1 |= (1 << 8);

    return 0;
}

/* Close/abort APIs used by IRQ handlers to stop transfers */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
    /* disable buffer and event interrupts */
    pI2CHandle->pI2Cx->CR2 &= ~I2C_CR2_ITBUFEN;
    pI2CHandle->pI2Cx->CR2 &= ~I2C_CR2_ITEVTEN;

    pI2CHandle->pRxBuffer = NULL;
    pI2CHandle->RXLen = 0;
    pI2CHandle->RXSize = 0;
    pI2CHandle->TXRXState = I2C_READY;
    /* Application callback - RX complete */
    I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
    /* disable buffer and event interrupts */
    pI2CHandle->pI2Cx->CR2 &= ~I2C_CR2_ITBUFEN;
    pI2CHandle->pI2Cx->CR2 &= ~I2C_CR2_ITEVTEN;

    pI2CHandle->pTxBuffer = NULL;
    pI2CHandle->TXLen = 0;
    pI2CHandle->TXRXState = I2C_READY;
    /* Application callback - TX complete */
    I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
}

/* Event IRQ handler (I2C_EV) */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    uint32_t sr1 = pI2CHandle->pI2Cx->SR1;
    uint32_t sr2 = pI2CHandle->pI2Cx->SR2; /* some events need reading SR2 too */

    /* 1. Handle SB (start bit) - master mode */
    if(sr1 & I2C_FLAG_SB)
    {
        /* SB cleared by reading SR1 followed by writing DR (address) */
        if(pI2CHandle->TXRXState == I2C_BUSY_IN_TX)
        {
            /* send address + write */
            pI2CHandle->pI2Cx->DR = (pI2CHandle->DevAddr << 1);
        }
        else if(pI2CHandle->TXRXState == I2C_BUSY_IN_RX)
        {
            /* send address + read */
            pI2CHandle->pI2Cx->DR = (pI2CHandle->DevAddr << 1) | 1;
        }
    }

    /* 2. Handle ADDR */
    if(sr1 & I2C_FLAG_ADDR)
    {
        /* clear ADDR by reading SR1 then SR2 */
        (void)pI2CHandle->pI2Cx->SR1;
        (void)pI2CHandle->pI2Cx->SR2;

        /* Special: if receiving and RxLen == 1, we need to clear ACK BEFORE clearing ADDR (handled in master blocking).
         * In IT mode, we'll manage logic on RXNE events. */
    }

    /* 3. Handle TXE - data register empty (ready to transmit) */
    if(sr1 & I2C_FLAG_TXE)
    {
        if(pI2CHandle->TXRXState == I2C_BUSY_IN_TX)
        {
            if(pI2CHandle->TXLen > 0)
            {
                /* send next data byte */
                pI2CHandle->pI2Cx->DR = *pI2CHandle->pTxBuffer;
                pI2CHandle->pTxBuffer++;
                pI2CHandle->TXLen--;

                /* if TxLen becomes zero, BTF will signal end (or TXE + BTF) */
            }

            if(pI2CHandle->TXLen == 0)
            {
                /* Wait for BTF then generate STOP in BTF handling */
            }
        }
    }

    /* 4. Handle BTF (byte transfer finished) */
    if(sr1 & I2C_FLAG_BTF)
    {
        if(pI2CHandle->TXRXState == I2C_BUSY_IN_TX)
        {
            if(pI2CHandle->TXLen == 0)
            {
                /* Master transmission complete */
                /* Generate STOP if no repeated start requested */
                if(pI2CHandle->Sr == 0)
                    pI2CHandle->pI2Cx->CR1 |= (1 << 9);

                /* Close and notify app */
                I2C_CloseSendData(pI2CHandle);
            }
        }
        else if(pI2CHandle->TXRXState == I2C_BUSY_IN_RX)
        {
            /* In master Rx, BTF indicates two bytes remain - handle in RXNE below */
        }
    }

    /* 5. Handle RXNE - data register not empty (data available) */
    if(sr1 & I2C_FLAG_RXNE)
    {
        if(pI2CHandle->TXRXState == I2C_BUSY_IN_RX)
        {
            if(pI2CHandle->RXLen == 1)
            {
                /* last byte - read and finish */
                *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
                pI2CHandle->pRxBuffer++;
                pI2CHandle->RXLen--;

                /* generate STOP if no repeated start */
                if(pI2CHandle->Sr == 0)
                    pI2CHandle->pI2Cx->CR1 |= (1 << 9);

                /* close and callback */
                I2C_CloseReceiveData(pI2CHandle);
            }
            else if(pI2CHandle->RXLen > 1)
            {
                /* read byte */
                *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
                pI2CHandle->pRxBuffer++;
                pI2CHandle->RXLen--;

                /* if next to last byte (RxLen==1) then disable ACK so that after next read a NACK is sent */
                if(pI2CHandle->RXLen == 1)
                {
                    I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
                }
            }
        }
    }

    /* 6. Handle STOPF (only valid in slave mode for master reading slave) - clear by reading SR1 then writing to CR1 */
    if(sr1 & I2C_FLAG_STOPF)
    {
        /* clear STOPF */
        (void)pI2CHandle->pI2Cx->SR1;
        pI2CHandle->pI2Cx->CR1 |= 0;
        /* Application can be notified */
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
    }


    /* ---------------- SLAVE MODE HANDLING ---------------- */

    /* Address matched (slave) */
    if(sr1 & I2C_FLAG_ADDR)
    {
        /* reading SR1 and SR2 already done above; notify */
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_SLAVE_ADDR);
    }

    /* Data request from master (Master reading -> slave TXE set) */
    /* Detect transmitter mode via TRA bit in SR2 (bit 2) */
    if((sr1 & I2C_FLAG_TXE) && (sr2 & (1 << 2)))
    {
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
    }

    /* Data received from master (Master writing -> slave RXNE set) */
    if((sr1 & I2C_FLAG_RXNE) && !(sr2 & (1 << 2)))
    {
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
    }
}



/* Error IRQ handler (I2C_ER) */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    uint32_t sr1 = pI2CHandle->pI2Cx->SR1;

    if(sr1 & I2C_FLAG_BERR)
    {
        /* Clear BERR by writing 0 to the bit */
        pI2CHandle->pI2Cx->SR1 &= ~(I2C_FLAG_BERR);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
    }

    if(sr1 & I2C_FLAG_ARLO)
    {
        pI2CHandle->pI2Cx->SR1 &= ~(I2C_FLAG_ARLO);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
    }

    if(sr1 & I2C_FLAG_AF)
    {
        /* Acknowledge failed (NACK received) */
        pI2CHandle->pI2Cx->SR1 &= ~(I2C_FLAG_AF);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
    }

    if(sr1 & I2C_FLAG_OVR)
    {
        pI2CHandle->pI2Cx->SR1 &= ~(I2C_FLAG_OVR);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
    }

    if(sr1 & I2C_FLAG_TIMEOUT)
    {
        pI2CHandle->pI2Cx->SR1 &= ~(I2C_FLAG_TIMEOUT);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
    }
}

/* Slave blocking send */
void I2C_SlaveSendData(I2C_TypeDef *pI2Cx, uint8_t data)
{
    pI2Cx->DR = data;
}

/* Slave blocking receive */
uint8_t I2C_SlaveReceiveData(I2C_TypeDef *pI2Cx)
{
    return (uint8_t)pI2Cx->DR;
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_TypeDef *pI2Cx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        pI2Cx->CR2 |= (I2C_CR2_ITERREN | I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);
    }
    else
    {
        pI2Cx->CR2 &= ~(I2C_CR2_ITERREN | I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);
    }
}

/* -----------------------------------------------------------
 * I2C Bus Recovery
 *  - Handles stuck SDA/SCL lines by manually toggling GPIO SCL
 *  - Generates a STOP condition
 *  - Restores pins to AF mode and reinitializes I2C peripheral
 * ----------------------------------------------------------- */
void I2C_BusRecovery(I2C_Handle_t *pI2CHandle)
{
    I2C_TypeDef *pI2Cx = pI2CHandle->pI2Cx;

    /* 1. Disable I2C peripheral */
    pI2Cx->CR1 &= ~(1 << 0);

    /* For I2C1 assume PB6=SCL, PB7=SDA (adjust per your board/pins) */
    /* --- Configure as GPIO open-drain outputs --- */
    GPIOB->MODER &= ~((3 << (6*2)) | (3 << (7*2))); // clear mode
    GPIOB->MODER |= ((1 << (6*2)) | (1 << (7*2)));  // set as output
    GPIOB->OTYPER |= (1 << 6) | (1 << 7);           // open-drain

    /* 2. Toggle SCL up to 9 times until SDA released */
    for(int i=0; i<9; i++)
    {
        GPIOB->BSRRL = (1 << (6 + 16)); // SCL low
        for(volatile int d=0; d<100; d++); // short delay
        GPIOB->BSRRH = (1 << 6);        // SCL high
        for(volatile int d=0; d<100; d++); // short delay

        if(GPIOB->IDR & (1 << 7)) // SDA released?
            break;
    }

    /* 3. Generate STOP condition manually */
    GPIOB->BSRRL = (1 << (7 + 16)); // SDA low
    for(volatile int d=0; d<100; d++);
    GPIOB->BSRRH = (1 << 6);        // SCL high
    for(volatile int d=0; d<100; d++);
    GPIOB->BSRRH = (1 << 7);        // SDA high

    /* 4. Reconfigure pins back to AF4 (I2C) */
    GPIOB->MODER &= ~((3 << (6*2)) | (3 << (7*2)));
    GPIOB->MODER |= ((2 << (6*2)) | (2 << (7*2))); // Alternate Function
    GPIOB->AFR[0] |= (4 << (6*4)) | (4 << (7*4));  // AF4 = I2C1

    /* 5. Reset and re-enable I2C peripheral */
    if(pI2Cx == I2C1)
    {
        RCC->APB1RSTR |= (1 << 21);
        RCC->APB1RSTR &= ~(1 << 21);
    }
    else if(pI2Cx == I2C2)
    {
        RCC->APB1RSTR |= (1 << 22);
        RCC->APB1RSTR &= ~(1 << 22);
    }
    else if(pI2Cx == I2C3)
    {
        RCC->APB1RSTR |= (1 << 23);
        RCC->APB1RSTR &= ~(1 << 23);
    }

    /* Re-init with user config */
    I2C_Init(pI2CHandle);

    /* Enable peripheral again */
    I2C_PeripheralControl(pI2Cx, ENABLE);
}

/* Weak application callback */
__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
    // user can override
    (void)pI2CHandle;
    (void)AppEv;
}









