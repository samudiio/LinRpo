
/**
 * \file: lin.c
 * \Desription: Comunication File to control LIN Headder
 * \Date: 03/04/2018
 */

/*-----------------------------------------------------------------------------
 *         Includes
 *----------------------------------------------------------------------------*/

#include "lin.h"
#include "uart.h"

/*-----------------------------------------------------------------------------
 *         Defines
 *----------------------------------------------------------------------------*/

#define BAUDRATE_UPD     115200

/*------------------------------------------------------------------------------
 *          Global Variables
 *----------------------------------------------------------------------------*/

 static uint8_t gs_Lin_stateMachine;
 static uint8_t gs_Lin_ByteCounter;
 static uint8_t gs_Lin_LinPid;

 uint8_t TxBuffRdy;

/*------------------------------------------------------------------------------
 *         Global Functions
 *----------------------------------------------------------------------------*/

/* Lin_Init: UART driver Init Function*/
void Lin_Init (const LinConfigType* Config)
{
    uint8_t LinBaudrate;
    LinBaudrate = Config->ChTyp.ChBr;
    Uart_Init(LinBaudrate, Lin_Isr);
}

/* Lin_SendFrame: LIN function that send the frame every 10ms*/
 Std_ReturnType Lin_SendFrame (uint8_t Channel, LinPduType* PduInfoPtr)
 {
     gs_Lin_LinPid = 0;//LinPid;

     if(IDLE == gs_Lin_stateMachine)
     {
         if((gs_Lin_ByteCounter == FIRST_BREAK_BYTE) && (TxBuffRdy)){
             gs_Lin_stateMachine = SEND_BREAK;
             UART_PutChar(UART4, 0x00);
             UART_UpdateBaudRate(BAUDRATE_UPD);
             gs_Lin_ByteCounter = SECOND_BREAK_BYTE;
         }
         else{
            /*Do Nothing*/
         }
     }
     else{
         /*Do Nothing*/
     }
    return 0;
 }

 Std_ReturnType Lin_GetSlaveResponse ( uint8_t Channel, uint8_t** LinSduPtr )
 {
     return 0;
 }

/* Lin_Isr: LIN callback function for UART Interrupt also implement the StateMachine*/
void Lin_Isr(void)
{
    switch(gs_Lin_stateMachine)
    {
        case(IDLE):
            /*Do Nothing*/
            break;

        case(SEND_BREAK):
            if ((gs_Lin_ByteCounter == SECOND_BREAK_BYTE) && (TxBuffRdy)){
                UART_PutChar(UART4, 0x00);
                gs_Lin_ByteCounter = FIRST_BREAK_BYTE;
                /* Change State to SYNC Bytes*/
                gs_Lin_stateMachine = SEND_SYNC;
            }
            else{
                /* Do Nothing */
                }
                break;
            case(SEND_SYNC):
                if (TxBuffRdy){
                    UART_PutChar(UART4, 0x55);
                    gs_Lin_stateMachine = SEND_PID;
                }
                else{
                    /*Do Notihing*/
                }
                break;
            case(SEND_PID):
                if (TxBuffRdy){
                    UART_PutChar(UART4,gs_Lin_LinPid);
                    gs_Lin_stateMachine = IDLE;
                }
                else{
                    /*Do Nothing*/
                }
                break;
            case(SEND_RESPONSE):
                break;
            default:
                break;
    }
}

