/*
 * LinNM.c
 *
 * Created:
 *  Author: Raul Cotero
 */ 

#include "Lin.h"



void LIN_10Task(void){
	uint8_t Channel = 0;
	LinPduType PduInfoPtr;

	Lin_SendFrame(Channel, &PduInfoPtr);


}