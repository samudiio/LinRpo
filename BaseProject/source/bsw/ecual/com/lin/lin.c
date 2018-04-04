
/**
 * \file
 */

/*-----------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include "board.h"
#include "lin.h"

/*------------------------------------------------------------------------------
 *          Variables
 *----------------------------------------------------------------------------*/

 static uint8_t ls_Lin_stateMachine;
 static uint8_t ls_Lin_ByteCounter;
 static uint8_t Lin_LinPid;

 uint8_t TxBuffRdy;

/*------------------------------------------------------------------------------
 *         Global Functions
 *----------------------------------------------------------------------------*/



void Lin_Init (uint32_t LinBaudrate)
{
    Uart_Init(LinBaudrate, Lin_Isr);
}

 void Lin_SendFrame (uint8_t LinPid)
 {
     Lin_LinPid = LinPid;

     if(IDLE == ls_Lin_stateMachine)
     {
         if((ls_Lin_ByteCounter == FIRST_BREAK_BYTE) && (TxBuffRdy)){
             UART_PutChar(UART4, 0x41);
             ls_Lin_ByteCounter = SECOND_BREAK_BYTE;
         }
         ls_Lin_stateMachine = SEND_BREAK;
     }
     else{
         /*Do Nothing*/
     }
 }

void Lin_Isr(void)
{
    switch(ls_Lin_stateMachine)
    {
        case(IDLE):
            /*Do Nothing*/
            break;

        case(SEND_BREAK):
            if ((ls_Lin_ByteCounter == SECOND_BREAK_BYTE) && (TxBuffRdy)){
                UART_PutChar(UART4, 0x42);
                ls_Lin_ByteCounter = FIRST_BREAK_BYTE;
                /* Change State to SYNC Bytes*/
                ls_Lin_stateMachine = SEND_SYNC;
            }
            else{
                /* Do Nothing */
                }
                break;
            case(SEND_SYNC):
                if (TxBuffRdy){
                    UART_PutChar(UART4, 0x55);
                    ls_Lin_stateMachine = SEND_PID;
                }
                else{
                    /*Do Notihing*/
                }
                break;
            case(SEND_PID):
                if (TxBuffRdy){
                    UART_PutChar(UART4,Lin_LinPid);
                    ls_Lin_stateMachine = IDLE;
                }
                break;
            case(SEND_RESPONSE):
                break;
            default:
                break;
    }
}
