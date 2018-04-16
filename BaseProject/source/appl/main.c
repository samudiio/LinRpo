/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"
#include "app_scheduler.h"
#include "lin.h"
#include "Tasks.h"
#include "MemAlloc.h"
#include <stdbool.h>
#include <stdio.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

#define LIN_BAUDRATE    9600

FuncPtr array_func[]={
	vfnTsk_1ms,
	vfnTsk_2msA,
	vfnTsk_2msB,
	vfnTsk_10ms,
	vfnTsk_50ms,
	vfnTsk_100ms
};

struct LinConfigType Config;

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief Configure LEDs
 *
 *  Configures LEDs \#1 and \#2 (cleared by default).
 */
static void _ConfigureLeds( void )
{
	LED_Configure( 0 ) ;
	LED_Configure( 1 ) ;
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
/**
 *  \brief getting-started Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
extern int main( void )
{
    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Enable I and D cache */
    SCB_EnableICache();
    SCB_EnableDCache();

    printf( "Configure LED PIOs.\n\r" ) ;
    _ConfigureLeds() ;
  
    MemAllocInit();
    printf( "\n\r-- Memory Allocation Initialized!!! --\n\r" ) ;
  

    /*MCAN_InitTxQueue(loc_mcan_Config);
      printf( "\n\r-- MCAN Tx Queue Initialized!!! --\n\r" ) ;*/

    /* Initialize Task Scheduler */
    vfnScheduler_Init(&array_func[0]);
    /* Start execution of task scheduler */
    vfnScheduler_Start();
    Config.ChTyp.ChBr = LIN_BAUDRATE;

    Lin_Init(&Config);

    /*-- Loop through all the periodic tasks from Task Scheduler --*/
    for(;;)
    {
        /* Perform all scheduled tasks */
        vfnTask_Scheduler();
    }

}
