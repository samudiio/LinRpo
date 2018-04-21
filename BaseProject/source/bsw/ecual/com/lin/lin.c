
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

#define BAUDRATE_UPD    115200

/*------------------------------------------------------------------------------
 *         Types Definitions
 *----------------------------------------------------------------------------*/

typedef enum
{
    Lin_NotBussy = 0,
    Lin_Bussy,
}LinDvrStateType;

/*
 * Channel-specific status parameters
 */
typedef struct
{
    uint8_t Channel;
    LinDvrStateType TxBussy;
    LinDvrStateType RxBussy;
}LinChStatusType;

/*
 * Driver-specific status parameters
 */
typedef struct
{
    uint8_t ChannelNumber;
    const LinChStatusType *PtrChStatus;
}LinStatusType;

/*------------------------------------------------------------------------------
 *          Global Variables
 *----------------------------------------------------------------------------*/
const LinConfig_T *gLinConfigPtr;    /*Gobal Pointer to Lin configuration*/
const LinStatusType *gLinStatusPtr;

uint8_t LinLogicaltoPhysicalCh[UART_MAX_CH];

 static uint8_t gs_Lin_stateMachine;
 static uint8_t gs_Lin_ByteCounter;
 static uint8_t gs_Lin_LinPid;
 static uint8_t gs_Lin_LinFrameResponse;
 static uint8_t gs_Lin_LinFrameDlc;
 static uint8_t gs_Lin_LinFrameMessageCounter;

 static uint8_t *gs_Lin_LinFrameData;

 uint8_t TxBuffRdy;

/*------------------------------------------------------------------------------
 *         Global Functions
 *----------------------------------------------------------------------------*/

/* Lin_Init: UART driver Init Function*/
void Lin_Init (const LinConfig_T *Config)
{
    uint8_t Lin_Idx;
    uint8_t PhyChannel;
    uint32_t LinBaudrate;
    Linchannel_T Lin_LogChannel;

    /*Copy Lin Configuration to a global variable*/
    gLinConfigPtr = Config;

    /*Reserve necesary memory to store internal status structure according to the total numer of channels defined in configuration*/
    gLinStatusPtr = (LinStatusType*) MemAlloc( (sizeof(LinStatusType)) * (gLinConfigPtr->LinNumberOfChannels));

    /*Do for all channels confugured*/
    for(Lin_Idx = 0; Lin_Idx < gLinConfigPtr->LinNumberOfChannels; Lin_Idx++)
    {
        /*Get Logical Channel*/
        Lin_LogChannel = gLinConfigPtr->PtrLinChannelCfg[Lin_Idx];

        /*Get Physical Channel*/
        PhyChannel = Lin_LogChannel.LinChannelId;

        /*Get Baudrate to configure*/
        LinBaudrate = Lin_LogChannel.LinChannelBaudrate;

        /*Map corresponding channels*/
        LinLogicaltoPhysicalCh[Lin_Idx] = PhyChannel;

        /*Initialitate UART module*/
        Uart_Init(PhyChannel, LinBaudrate, Lin_Isr);
    }
}

/* Lin_SendFrame: LIN function that send the frame every 10ms*/
 Std_ReturnType Lin_SendFrame (uint8_t Channel, LinPduType* PduInfoPtr)
 {
     gs_Lin_LinPid              = PduInfoPtr->Pid;
     gs_Lin_LinFrameResponse    = PduInfoPtr->Drc;
     gs_Lin_LinFrameDlc         = PduInfoPtr->Dl;
     gs_Lin_LinFrameData        = PduInfoPtr->SduPtr;

     if(IDLE == gs_Lin_stateMachine)
     {
         if((gs_Lin_ByteCounter == FIRST_BREAK_BYTE) && (TxBuffRdy)){
             gs_Lin_stateMachine = SEND_BREAK;
             UART_PutChar(LinLogicaltoPhysicalCh[Channel], 0x00);
             UART_UpdateBaudRate(LinLogicaltoPhysicalCh[Channel], BAUDRATE_UPD);
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
void Lin_Isr(uint8_t Channel)
{
    switch(gs_Lin_stateMachine)
    {
        case(IDLE):
            /*Do Nothing*/
            break;

        case(SEND_BREAK):
            if ((gs_Lin_ByteCounter == SECOND_BREAK_BYTE) && (TxBuffRdy)){
                UART_PutChar(Channel, 0x00);
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
                    UART_PutChar(Channel, 0x55);
                    gs_Lin_stateMachine = SEND_PID;
                }
                else{
                    /*Do Notihing*/
                }
                break;
            case(SEND_PID):
                if (TxBuffRdy){
                    UART_PutChar(Channel, gs_Lin_LinPid);
                    if(LIN_MASTER_REQUEST_FRAME == gs_Lin_LinFrameResponse){
                        gs_Lin_stateMachine = MASTER_RESPONSE;
                    }
                    else{
                        gs_Lin_stateMachine = SLAVE_RESPONSE;
                    }
                    
                }
                else{
                    /*Do Nothing*/
                }
                break;
            case(MASTER_RESPONSE):
                if(gs_Lin_LinFrameDlc == gs_Lin_LinFrameMessageCounter){
                    gs_Lin_stateMachine = IDLE;
                }
                else{
                    if(TxBuffRdy){
                        UART_PutChar(Channel, *(gs_Lin_LinFrameData + gs_Lin_LinFrameMessageCounter));
                        gs_Lin_LinFrameMessageCounter++;
                    }

                }
                break;
            case(SLAVE_RESPONSE):
                gs_Lin_stateMachine = IDLE;
                break;
            default:
                break;
    }
}

