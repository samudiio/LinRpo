/*
 * lin_cfg.h
 *
 * Definition of configurable parameters, LIN configuration types
 *
 */

#ifndef _LIN_CFG_H_
#define _LIN_CFG_H_

/*----------------------------------------------------------------------------
 *       Includes
 *----------------------------------------------------------------------------*/
#include "Std_Types.h"

/*-----------------------------------------------------------------------------
 *         Type Definitions
 *----------------------------------------------------------------------------*/

/*This container contains the configuration parameters of the LIN channel*/
typedef struct LinChannel_Tag
{
    uint8_t     LinChannelId;               /*Lin Channel Identifier*/
    uint16_t    LinChannelBaudrate;        /*Specifies the baud rate of the LIN channel*/
}Linchannel_T;

typedef struct LinConfig_Tag
{
    uint8_t         LinNumberOfChannels;    /*Number of channels to be configured*/
    Linchannel_T    LinChannel;             /*Parameters related to each LIN channel*/
}LinConfig_T;

#endif /*_LIN_CFG_H_ */
