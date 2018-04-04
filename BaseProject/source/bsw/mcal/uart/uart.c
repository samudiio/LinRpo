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

uint8_t pTxBuffer[] = {"This is UART Tx Buffer.........\n\r"};
const Pin Uart_Default_Pins[] = {PINS_UART4};

uint8_t TxBuffRdy = 0;

LINFuncPtr LinFcnPtr;


/*------------------------------------------------------------------------------
 *         Exported functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief Handler for UART4.
 *
 *  Process UART4 interrupts
 */
void UART4_Handler(void)
{
    UartMasks status = 0;

    status = UART_DEFAULT->UART_SR;

    if(status & UART_MASK_RXRDY)
    {
        printf("%c", (char)UART_DEFAULT->UART_RHR);
    }
    else if((status & UART_MASK_TXRDY) && (!TxBuffRdy))
    {
        TxBuffRdy = 1;
        
        NVIC_DisableIRQ(UART_IRQ_DEFAULT);

        if (LinFcnPtr){
            LinFcnPtr();
        }
    }
    else {
        /*Do Nothing*/
    }
}


void Uart_Init(uint32_t Baudrate,  void (*linfunc_ptr)(void))
{
    uint8_t *pBuffer = &pTxBuffer[0];

    LinFcnPtr = linfunc_ptr;

    PIO_Configure(Uart_Default_Pins, PIO_LISTSIZE(Uart_Default_Pins));
    PMC_EnablePeripheral(UART_ID_DEFAULT);
    UART_Configure(UART_DEFAULT, (UART_MR_CHMODE_NORMAL | UART_MR_BRSRCCK_PERIPH_CLK | UART_MR_PAR_NO), Baudrate, BOARD_MCK);

    NVIC_ClearPendingIRQ(UART_IRQ_DEFAULT);
    NVIC_SetPriority(UART_IRQ_DEFAULT ,1);

    /* Enables the UART to transfer and receive data. */
    UART_SetTransmitterEnabled (UART_DEFAULT , ENABLE);
    UART_SetReceiverEnabled (UART_DEFAULT , ENABLE);

    UART_EnableIt(UART_DEFAULT, (UART_IER_RXRDY | UART_IER_TXRDY));
    /* Enable interrupt  */
    NVIC_EnableIRQ(UART_IRQ_DEFAULT);
}

void UART_UpdateBaudRate(uint32_t Baudrate)
{
    /* Reset and disable receiver & transmitter*/
    UART_DEFAULT->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX
            | UART_CR_RXDIS | UART_CR_TXDIS | UART_CR_RSTSTA;

    /* Configure baudrate*/
    UART_DEFAULT->UART_BRGR = (BOARD_MCK / Baudrate) / 16;

    UART_DEFAULT->UART_CR = UART_CR_TXEN | UART_CR_RXEN;
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
void UART_PutChar( Uart *uart, uint8_t c)
{
	/* Wait for the transmitter to be ready*/
	//while (!UART_IsRxReady(uart) && !UART_IsTxSent(uart));

    if(TxBuffRdy == 1)
    {
        TxBuffRdy = 0;

        /* Send character*/
        uart->UART_THR = c;
        NVIC_EnableIRQ(UART_IRQ_DEFAULT);
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

void UART_SendBuffer(Uart *uart, uint8_t *pBuffer, uint32_t BuffLen)
{
	uint8_t *pData = pBuffer;
	uint32_t Len =0;

	for(Len =0; Len<BuffLen; Len++ ) {
		UART_PutChar(uart, *pData);
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
