#ifndef __IDS_UART_H__
#define __IDS_UART_H__

#include "MMI_features.h"
#include "kal_release.h"
#include "dcl.h"

void gps_uart_set_owner_id(UART_PORT port, module_type ownerid);

module_type gps_UART_GetOwnerID(UART_PORT port);

kal_bool gps_UART_Open(DCL_DEV port, DCL_UINT32 ownerid);

void gps_UART_SetDCBConfig(DCL_DEV port, UART_CONFIG_T *UART_Config, DCL_UINT32 ownerid);

void gps_clrtxrx_buf(DCL_DEV port);
DCL_UINT8 gps_getUARTByte(DCL_DEV port);

void gps_PutUARTByte(DCL_DEV port, DCL_UINT8 data);
kal_uint16 uart_put_bytes(UART_PORT port, kal_uint8 *Buffaddr, kal_uint16 Length, module_type ownerid);
kal_uint16 uart_get_bytes(UART_PORT port, kal_uint8 *Buffaddr, kal_uint16 Length, kal_uint8 *status, module_type ownerid);
void gps_clr_rx_buf(DCL_DEV port);
void gps_clr_tx_buf(DCL_DEV port);
void gps_putUARTByte(DCL_DEV port, DCL_UINT8 data);

#endif

