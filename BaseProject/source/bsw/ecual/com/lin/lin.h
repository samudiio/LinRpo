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

#ifndef _LIN_
#define _LIN_

/*----------------------------------------------------------------------------
 *       Includes
 *----------------------------------------------------------------------------*/

#include "lin_cfg.h"
#include "lin_types.h"

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/* Function prototype to Configure the System and the LIN BUS*/
void Lin_Init (const LinConfig_T* Config);

/* Function prototype to Send the LIN Frame*/
Std_ReturnType Lin_SendFrame (uint8_t Channel, LinPduType* PduInfoPtr);

Std_ReturnType Lin_GetSlaveResponse (uint8_t Channel, uint8_t** LinSduPtr);

/* Function prototype to manipulate interruptions on the TX*/
void Lin_Isr(void);





#endif /* #ifndef LIN_H */

