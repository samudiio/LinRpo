/*
 * lin_cfg.c
 *
 */

/*-----------------------------------------------------------------------------
 *         Includes
 *----------------------------------------------------------------------------*/

#include "lin_cfg.h"

/*-----------------------------------------------------------------------------
 *         Defines
 *----------------------------------------------------------------------------*/

#define LIN_BAUDRATE        9600
#define LIN_BAUDRATE_HS     115200

/*----------------------------------------------------------------------------
 *        Global variables
 *----------------------------------------------------------------------------*/

/*
 * LIN Channel Configuration
 */
const Linchannel_T LIN_ChannelConfig[] =
{
    /*Logical Channel 0*/
    {
        LINCfg_PhyCh_4,
        LIN_BAUDRATE
    },
    /*Logical Channel 1*/
    {
        LINCfg_PhyCh_0,
        LIN_BAUDRATE_HS
    }
};

/*
 * LIN Configuration
 */
const LinConfig_T LIN_Config[] =
{
    {
        sizeof(LIN_ChannelConfig)/sizeof(LIN_ChannelConfig[0]),
        &LIN_ChannelConfig[0]
    }
};
