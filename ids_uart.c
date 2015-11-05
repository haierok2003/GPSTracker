
#include "ids_uart.h"



void gps_uart_set_owner_id(UART_PORT port, module_type ownerid)
{

	DCL_HANDLE handle;
	UART_CTRL_OWNER_T data;
	data.u4OwenrId = ownerid;

	handle = DclSerialPort_Open(port,0);
	DclSerialPort_Control(handle,SIO_CMD_SET_OWNER, (DCL_CTRL_DATA_T*)&data);
	DclSerialPort_Close(handle);
}

module_type gps_UART_GetOwnerID(UART_PORT port)
{

	DCL_HANDLE handle;
	UART_CTRL_OWNER_T data;

	handle = DclSerialPort_Open(port,0);
	DclSerialPort_Control(handle,SIO_CMD_GET_OWNER_ID, (DCL_CTRL_DATA_T*)&data);
	DclSerialPort_Close(handle);
	
	return data.u4OwenrId;	
}

void gps_UART_SetDCBConfig(DCL_DEV port, UART_CONFIG_T *UART_Config, DCL_UINT32 ownerid)
{
	DCL_HANDLE handle;
	UART_CTRL_DCB_T data;
	
	data.u4OwenrId = ownerid;
	data.rUARTConfig.u4Baud = UART_Config->u4Baud;
	data.rUARTConfig.u1DataBits = UART_Config->u1DataBits;
	data.rUARTConfig.u1StopBits = UART_Config->u1StopBits;
	data.rUARTConfig.u1Parity = UART_Config->u1DataBits;
	data.rUARTConfig.u1FlowControl = UART_Config->u1FlowControl;
	data.rUARTConfig.ucXonChar = UART_Config->ucXonChar;
	data.rUARTConfig.ucXoffChar = UART_Config->ucXoffChar;
	data.rUARTConfig.fgDSRCheck = UART_Config->fgDSRCheck;

	handle = DclSerialPort_Open(port,0);
	DclSerialPort_Control(handle,SIO_CMD_SET_DCB_CONFIG, (DCL_CTRL_DATA_T*)&data);
	DclSerialPort_Close(handle);

}

void gps_clr_rx_buf(DCL_DEV port)
{
   UART_CTRL_CLR_BUFFER_T data;
   DCL_HANDLE handle;
   data.u4OwenrId = MOD_MMI;
   handle = DclSerialPort_Open(port,0);
   DclSerialPort_Control( handle,SIO_CMD_CLR_RX_BUF, (DCL_CTRL_DATA_T*)&data);
   DclSerialPort_Close(handle);
}

void gps_clr_tx_buf(DCL_DEV port)
{
   UART_CTRL_CLR_BUFFER_T data;
   DCL_HANDLE handle;
   data.u4OwenrId = MOD_MMI;
   handle = DclSerialPort_Open(port,0);
   DclSerialPort_Control( handle,SIO_CMD_CLR_TX_BUF, (DCL_CTRL_DATA_T*)&data);
   DclSerialPort_Close(handle);
}


DCL_UINT8 gps_getUARTByte(DCL_DEV port)
{

	DCL_HANDLE handle;
	UART_CTRL_PUT_UART_BYTE_T data1;

	handle = DclSerialPort_Open(port,0);
	DclSerialPort_Control(handle,SIO_CMD_GET_UART_BYTE, (DCL_CTRL_DATA_T*)&data1);
	DclSerialPort_Close(handle);

	return data1.uData;

}

void gps_putUARTByte(DCL_DEV port, DCL_UINT8 data)
{

	DCL_HANDLE handle;
	UART_CTRL_PUT_UART_BYTE_T data1;
	data1.uData = data;

	handle = DclSerialPort_Open(port,0);
	DclSerialPort_Control(handle,SIO_CMD_PUT_UART_BYTE, (DCL_CTRL_DATA_T*)&data1);
	DclSerialPort_Close(handle);

}
/*
kal_uint16 uart_put_bytes(UART_PORT port, kal_uint8 *Buffaddr, kal_uint16 Length, module_type ownerid)
{

	DCL_HANDLE handle;
	UART_CTRL_PUT_BYTES_T data;

	data.u4OwenrId = ownerid;
	data.u2Length = Length;
	data.puBuffaddr = Buffaddr;

    handle = DclSerialPort_Open(port,0);
    DclSerialPort_Control(handle,SIO_CMD_PUT_BYTES, (DCL_CTRL_DATA_T*)&data);
    return data.u2RetSize;
}

kal_uint16 uart_get_bytes(UART_PORT port, kal_uint8 *Buffaddr, kal_uint16 Length, kal_uint8 *status, module_type ownerid)
{

    DCL_HANDLE handle;
    UART_CTRL_GET_BYTES_T data;

    data.u4OwenrId = ownerid;
    data.u2Length = Length;
    data.puBuffaddr = Buffaddr;
    data.pustatus = status;
    handle = DclSerialPort_Open(port,0);
    DclSerialPort_Control(handle,SIO_CMD_GET_BYTES, (DCL_CTRL_DATA_T*)&data);
    return data.u2RetSize;

}

*/

void gps_UART_turnon_power(UART_PORT port, kal_bool on)
{
    DCL_HANDLE handle;
    UART_CTRL_POWERON_T data;
    
    handle = DclSerialPort_Open(port, 0);
    data.bFlag_Poweron = on;
    DclSerialPort_Control(handle, UART_CMD_POWER_ON, (DCL_CTRL_DATA_T*)&data);
    
    DclSerialPort_Close(handle);
}

kal_bool gps_UART_open(UART_PORT port, module_type ownerid)
{
    DCL_HANDLE handle;
    UART_CTRL_OPEN_T data;
    DCL_STATUS status;
    data.u4OwenrId = ownerid;	
	
    handle = DclSerialPort_Open(port, 0);
    status = DclSerialPort_Control(handle, SIO_CMD_OPEN, (DCL_CTRL_DATA_T*)&data);
    DclSerialPort_Close(handle);   
	if(STATUS_OK != status)
		return KAL_FALSE;
    else
		return KAL_TRUE;
}


