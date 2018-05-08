/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2011, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/**
 * \file
 *
 * Implementation of UART (Universal Asynchronous Receiver Transmitter)
 * controller.
 *
 */
/*------------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/
#include "chip.h"

#include "board.h"

#include <assert.h>
#include <string.h>

//------------------------------------------------------------------------------
//         TypeDefs and Defines
//------------------------------------------------------------------------------
#define UART_PHYCH0        0
#define UART_PHYCH1        1
#define UART_PHYCH2        2
#define UART_PHYCH3        3
#define UART_PHYCH4        4

typedef struct
{
    IRQn_Type   UartIRQn[UART_MAX_CH];
    uint32_t    UartPeriphIds[UART_MAX_CH];
    Pin         UartPinId[UART_MAX_CH];
    Uart        *UartBaseAddress[UART_MAX_CH];
}UartPeripheralConfig_Type;

/*------------------------------------------------------------------------------
 *         Global Valiables
 *----------------------------------------------------------------------------*/
Uart *UartPtrg;
IRQn_Type Uart_IRQIdg;

uint8_t pTxBuffer[] = {"This is UART Tx Buffer.........\n\r"};

uint8_t TxBuffRdy = 0;

LINFuncPtr LinFcnPtr;

const UartPeripheralConfig_Type UartCfg =
{
    {UART0_IRQn, UART1_IRQn, UART2_IRQn, UART3_IRQn, UART4_IRQn},
    {ID_UART0, ID_UART1, ID_UART2, ID_UART3, ID_UART4},
    {PINS_UART0, PINS_UART1, PINS_UART2, PINS_UART3, PINS_UART4},
    {UART0, UART1, UART2, UART3, UART4}
};

/*------------------------------------------------------------------------------
 *         Local functions
 *----------------------------------------------------------------------------*/

/*
 * Brief: Generic UART interrupt service routine
 */
void Uart_Isr(uint8_t PhyChannel)
{
    UartMasks status = 0;

    /*Get Pointer to the UART peripheral to configure.*/
    UartPtrg = UartCfg.UartBaseAddress[PhyChannel];

    status = UartPtrg->UART_SR;

    if(status & UART_MASK_RXRDY)
    {
        //printf("%c", (char)UartPtrg->UART_RHR);
        //Lin_GetSlaveResponse(PhyChannel, );

        NVIC_EnableIRQ(Uart_IRQIdg);
    }
    else if((status & UART_MASK_TXRDY) && (!TxBuffRdy))
    {
        TxBuffRdy = 1;
        
        NVIC_DisableIRQ(Uart_IRQIdg);

        if (LinFcnPtr){
            LinFcnPtr(PhyChannel);
        }
        else{
            /*Do Nothing*/
        }
    }
    else {
        /*Do Nothing*/
    }
}

/*------------------------------------------------------------------------------
 *         Exported functions
 *----------------------------------------------------------------------------*/

/*
 * Brief: Initializes the UART module
 */
void Uart_Init(uint8_t PhyChannel, uint32_t Baudrate,  void (*linfunc_ptr)(uint8_t))
{
    uint8_t *pBuffer = &pTxBuffer[0];
    uint32_t UartPeriphId;
    Pin Uart_Pins;

    LinFcnPtr = linfunc_ptr;

    /*Get Pointer to the UART peripheral to configure.*/
    UartPtrg = UartCfg.UartBaseAddress[PhyChannel];

    /*Get UART Interrupt Number.*/
    Uart_IRQIdg = UartCfg.UartIRQn[PhyChannel];

    /*Get UART Peripherial Id*/
    UartPeriphId = UartCfg.UartPeriphIds[PhyChannel];

    /*Get UART pins to configure*/
    Uart_Pins = UartCfg.UartPinId[PhyChannel];

    PIO_Configure(&Uart_Pins, PIO_LISTSIZE(Uart_Pins));
    PMC_EnablePeripheral(UartPeriphId);
    UART_Configure(UartPtrg, (UART_MR_CHMODE_NORMAL | UART_MR_BRSRCCK_PERIPH_CLK | UART_MR_PAR_NO), Baudrate, BOARD_MCK);

    NVIC_ClearPendingIRQ(Uart_IRQIdg);
    NVIC_SetPriority(Uart_IRQIdg ,1);

    /* Enables the UART to transfer and receive data. */
    UART_SetTransmitterEnabled (UartPtrg , ENABLE);
    UART_SetReceiverEnabled (UartPtrg , ENABLE);

    UART_EnableIt(UartPtrg, (UART_IER_RXRDY | UART_IER_TXRDY));
    /* Enable interrupt  */
    NVIC_EnableIRQ(Uart_IRQIdg);
}

/*
 *  Function to update baudrate
 */
void UART_UpdateBaudRate(uint8_t PhyChannel, uint32_t Baudrate)
{
    /*Get Pointer to the UART peripheral to configure.*/
    UartPtrg = UartCfg.UartBaseAddress[PhyChannel];

    /* Reset and disable receiver & transmitter*/
    UartPtrg->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX
            | UART_CR_RXDIS | UART_CR_TXDIS | UART_CR_RSTSTA;

    /* Configure baudrate*/
    UartPtrg->UART_BRGR = (BOARD_MCK / Baudrate) / 16;

    UartPtrg->UART_CR = UART_CR_TXEN | UART_CR_RXEN;
}

/**
 * \brief Configures an UART peripheral with the specified parameters.
 *
 *
 *  \param uart  Pointer to the UART peripheral to configure.
 *  \param mode  Desired value for the UART mode register (see the datasheet).
 *  \param baudrate  Baudrate at which the UART should operate (in Hz).
 *  \param masterClock  Frequency of the system master clock (in Hz).
 */
void UART_Configure(Uart *uart,
		uint32_t mode,
		uint32_t baudrate,
		uint32_t masterClock)
{
	/* Reset and disable receiver & transmitter*/
	uart->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX
		| UART_CR_RXDIS | UART_CR_TXDIS | UART_CR_RSTSTA;

	uart->UART_IDR = 0xFFFFFFFF;

	/* Configure mode*/
	uart->UART_MR = mode;

	/* Configure baudrate*/
	uart->UART_BRGR = (masterClock / baudrate) / 16;

	uart->UART_CR = UART_CR_TXEN | UART_CR_RXEN;

}
/**
 * \brief Enables or disables the transmitter of an UART peripheral.
 *
 *
 * \param uart  Pointer to an UART peripheral
 * \param enabled  If true, the transmitter is enabled; otherwise it is
 *  disabled.
 */
void UART_SetTransmitterEnabled(Uart *uart, uint8_t enabled)
{
	if (enabled) {
		uart->UART_CR = UART_CR_TXEN;
	} else {
		uart->UART_CR = UART_CR_TXDIS;
	}
}

/**
 * \brief Enables or disables the receiver of an UART peripheral
 *
 *
 * \param uart  Pointer to an UART peripheral
 * \param enabled  If true, the receiver is enabled; otherwise it is disabled.
 */
void UART_SetReceiverEnabled(Uart *uart, uint8_t enabled)
{
	if (enabled) {
		uart->UART_CR = UART_CR_RXEN;
	} else {
		uart->UART_CR = UART_CR_RXDIS;
	}
}

/**
 * \brief   Return 1 if a character can be read in UART
 * \param uart  Pointer to an UART peripheral.
 */
uint32_t UART_IsRxReady(Uart *uart)
{
	return (uart->UART_SR & UART_SR_RXRDY);
}

/**
 * \brief  Reads and returns a character from the UART.
 *
 * \note This function is synchronous (i.e. uses polling).
 * \param uart  Pointer to an UART peripheral.
 * \return Character received.
 */
uint8_t UART_GetChar(Uart *uart)
{
	while (!UART_IsRxReady(uart));
	return uart->UART_RHR;
}

/**
 * \brief   Return 1 if a character can be send to UART
 * \param uart  Pointer to an UART peripheral.
 */
uint32_t UART_IsTxReady(Uart *uart)
{
	return (uart->UART_SR & UART_SR_TXRDY);
}

/**
 * \brief   Return 1 if a character can be send to UART
 * \param uart  Pointer to an UART peripheral.
 */
static uint32_t UART_IsTxSent(Uart *uart)
{
	return (uart->UART_SR & UART_SR_TXEMPTY);
}

/**
 * \brief  Sends one packet of data through the specified UART peripheral. This
 * function operates synchronously, so it only returns when the data has been
 * actually sent.
 *
 * \param uart  Pointer to an UART peripheral.
 * \param c  Character to send
 */
void UART_PutChar(uint8_t PhyChannel, uint8_t c)
{
    /*Get Pointer to the UART peripheral to configure.*/
    UartPtrg = UartCfg.UartBaseAddress[PhyChannel];

	/* Wait for the transmitter to be ready*/
	//while (!UART_IsRxReady(uart) && !UART_IsTxSent(uart));

    if(1 == TxBuffRdy)
    {
        TxBuffRdy = 0;
        /* Send character*/
        UartPtrg->UART_THR = c;
        NVIC_EnableIRQ(Uart_IRQIdg);
    }
    else{
    	/*Do Nothing*/
    }



	/* Wait for the transfer to complete*/
	//while (!UART_IsTxSent(uart));
}

/**
 * \brief   Get present status
 * \param uart  Pointer to an UART peripheral.
 */
uint32_t UART_GetStatus(Uart *uart)
{
	return uart->UART_SR;
}

/**
 * \brief   Enable interrupt
 * \param uart  Pointer to an UART peripheral.
 * \param mode  Interrupt mode.
 */
void UART_EnableIt(Uart *uart,uint32_t mode)
{
	uart->UART_IER = mode;
}

/**
 * \brief   Disable interrupt
 * \param uart  Pointer to an UART peripheral.
 * \param mode  Interrupt mode.
 */
void UART_DisableIt(Uart *uart,uint32_t mode)
{
	uart->UART_IDR = mode;
}

/**
 * \brief   Return interrupt mask
 * \param uart  Pointer to an UART peripheral.
 */
uint32_t UART_GetItMask(Uart *uart)
{
	return uart->UART_IMR;
}

void UART_SendBuffer(uint8_t PhyChannel, uint8_t *pBuffer, uint32_t BuffLen)
{
	uint8_t *pData = pBuffer;
	uint32_t Len =0;

	for(Len =0; Len<BuffLen; Len++ ) {
		UART_PutChar(PhyChannel, *pData);
		pData++;
	}
}

void UART_ReceiveBuffer(Uart *uart, uint8_t *pBuffer, uint32_t BuffLen)
{
	uint32_t Len =0;

	for(Len =0; Len<BuffLen; Len++ ) {
		*pBuffer = UART_GetChar(uart);
		pBuffer++;
	}
}

void UART_CompareConfig(Uart *uart, uint8_t Val1, uint8_t Val2)
{
	uart->UART_CMPR = (UART_CMPR_VAL1(Val1) | UART_CMPR_VAL2(Val2));
}

/**
 *  \brief Handler for UART0.
 *
 *  Process UART0 interrupts
 */
void UART0_Handler(void)
{
    Uart_Isr(UART_PHYCH0);
}
/**
 *  \brief Handler for UART1.
 *
 *  Process UART1 interrupts
 */
void UART1_Handler(void)
{
    Uart_Isr(UART_PHYCH1);
}
/**
 *  \brief Handler for UART2.
 *
 *  Process UART2 interrupts
 */
void UART2_Handler(void)
{
    Uart_Isr(UART_PHYCH2);
}
/**
 *  \brief Handler for UART3.
 *
 *  Process UART3 interrupts
 */
void UART3_Handler(void)
{
    Uart_Isr(UART_PHYCH3);
}

/**
 *  \brief Handler for UART4.
 *
 *  Process UART4 interrupts
 */
void UART4_Handler(void)
{
    Uart_Isr(UART_PHYCH4);
}
