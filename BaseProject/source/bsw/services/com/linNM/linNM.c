/*
 * LinNM.c
 *
 */ 

/*-----------------------------------------------------------------------------
 *         Includes
 *----------------------------------------------------------------------------*/

#include "Lin.h"

uint8_t Channel = 0;

uint8_t DataResponse[] =
{
    0x13,
    0x07,
    0x0B,
    0x0D
};
uint8_t DataResponse2[] =
{
    0x00
};

LinPduType PduInfoPtr[] =
{
    {
        LIN_MASTER_REQUEST_FRAME,
        LIN_CLASSIC_CS,
        LIN_MASTER_RESPONSE,
        sizeof(DataResponse),
        DataResponse
    },
    {
        LIN_SLAVE_RESPONSE_FRAME,
        LIN_ENHANCED_CS,
        LIN_SLAVE_RESPONSE,
        sizeof(DataResponse2),
        DataResponse2
    }
};

void LIN_10Task(void)
{
    Lin_SendFrame(Channel, &PduInfoPtr[0]);
}
