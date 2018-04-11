/**
 * \file
 *
 *  \section Purpose
 * 
 *  Small set of functions for simple and portable LIN usage.
 * 
 *  \section Usage
 * 
 *  -# Configure one or more LINs using LIN_Configure and
 *     LIN_ConfigureAll.
 *  -# Set, clear and toggle LINs using LIN_Set, LIN_Clear and
 *     LIN_Toggle.
 * 
 *  LINs are numbered starting from 0; the number of LINs depend on the
 *  board being used. All the functions defined here will compile properly
 *  regardless of whether the LIN is defined or not; they will simply
 *  return 0 when a LIN which does not exist is given as an argument.
 *  Also, these functions take into account how each LIN is connected on to
 *  board; thus, \ref LIN_Set might change the level on the corresponding pin
 *  to 0 or 1, but it will always light the LIN on; same thing for the other
 *  methods.
 */
#ifndef _LIN_TYPES_
#define _LIN_TYPES_

#include <stdint.h>

/*----------------------------------------------------------------------------
 *        TypeDefs
 *----------------------------------------------------------------------------*/


typedef enum
   {
     IDLE,
     SEND_BREAK,
     SEND_SYNC,
     SEND_PID,
     SEND_RESPONSE
}LinStateType;

/*Description: */
typedef enum
   {
     FIRST_BREAK_BYTE,
     SECOND_BREAK_BYTE,
     SYNC_BYTE,
     PID_BYTE
}FrameTypes;


/*Description: The LIN identifier (0..0x3F) along with its two parity bits*/
typedef uint8_t LinFramePidType;

/*Description: Specifies the Checksum model used in the LIN Frame*/
typedef enum {
    LIN_ENHANCED_CS,
    LIN_CLASSIC_CS
}LinFrameCsModelType;    


/* Specifies whether the frame processor is required to transmit the response part of the LIN fram*/
typedef enum{
    LIN_MASTER_RESPONSE,
    LIN_SLAVE_RESPONSE
}LinFrameResponseType;

typedef uint8_t LinFrameDlType;


/*This type is used to provide PID, checksum model, data length and SDU pointer to the LIN driver*/
typedef struct LinPduTypes{
    LinFramePidType         Pid;
    LinFrameCsModelType     Cs;
    LinFrameResponseType    Drc;
    LinFrameDlType          Dl;
    uint8_t*                SduPtr;
}LinPduType;



#endif