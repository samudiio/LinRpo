
/**
 * \file
 */

/*-----------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include "board.h"
#include "lin.h"

/*------------------------------------------------------------------------------
 *         Local Variables
 *----------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 *         Global Functions
 *----------------------------------------------------------------------------*/


void Lin_Init (uint16_t LinBaudrate){
      uint32_t mode = (UART_MR_CHMODE_NORMAL | UART_MR_PAR_NO
					| UART_MR_BRSRCCK_PERIPH_CLK);
      uint32_t masterClock;
      
      UART_Configure(UART0, mode, LinBaudrate, masterClock);
 }


 void Lin_Isr(void){

 	
 }


 void Lin_SendFrame (uint8_t LinPid){


 }