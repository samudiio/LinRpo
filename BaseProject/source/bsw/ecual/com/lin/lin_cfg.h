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

typedef enum
{
    LINCfg_PhyCh_0 = 0,
    LINCfg_PhyCh_1,
    LINCfg_PhyCh_2,
    LINCfg_PhyCh_3,
    LINCfg_PhyCh_4
}Lin_Ch_T;

/*This container contains the configuration parameters of the LIN channel*/
typedef struct LinChannel_Tag
{
    Lin_Ch_T    LinChannelId;               /*Lin Channel Identifier*/
    uint32_t    LinChannelBaudrate;        /*Specifies the baud rate of the LIN channel*/
}Linchannel_T;

typedef struct LinConfig_Tag
{
    uint8_t             LinNumberOfChannels;    /*Number of channels to be configured*/
    const Linchannel_T  *PtrLinChannelCfg;      /*Parameters related to each LIN channel (1..n)*/
}LinConfig_T;

/*----------------------------------------------------------------------------
 *        Exported variables
 *----------------------------------------------------------------------------*/

extern const LinConfig_T LIN_Config[];

#endif /*_LIN_CFG_H_ */
