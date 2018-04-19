/*
 * LinNM.c
 *
 */ 

/*-----------------------------------------------------------------------------
 *         Includes
 *----------------------------------------------------------------------------*/

#include "Lin.h"

uint8_t Channel = 0;
LinPduType PduInfoPtr;

void LIN_10Task(void){
    PduInfoPtr.Pid = 0x3C;

	Lin_SendFrame(Channel, &PduInfoPtr);


}
