/*********************************************************************
 
  (c) copyright Freescale Semiconductor Ltd. 2008
  ALL RIGHTS RESERVED
 
********************************************************************
 Ver 2.1.0	Released Oct.2008
********************************************************************
*	example:
*
*	VINT8 DX8, DY8, DZ8;
*	FSL_MMA_init();
*	FSL_MMA_IICWrite(0x16, 0x45); // Enable measurment mode
*	DX8 = FSL_MMA_IICRead(0x06); // Read X
*	DY8 = FSL_MMA_IICRead(0x07); // Read Y
*	DZ8 = FSL_MMA_IICRead(0x08); // Read Z
*	FSL_MMA_ReadXYZ8(&DX8, &DY8, &DZ8);
*
********************************************************************/
#ifndef __FSL_MMA_H__
#define __FSL_MMA_H__

#include "FSL_data.h"
#include "gpstracker.h"

#define MMA_XOUT	0X00 
#define MMA_YOUT	0X01 
#define MMA_ZOUT	0X02 
#define MMA_TILT	0X03 
#define MMA_SRST	0X04 
#define MMA_SPCNT	0X05 
#define MMA_INTSU   0X06   
#define MMA_MODE	0X07 
#define MMA_SR	 	0X08  
#define MMA_PDET	0X09 
#define MMA_PD	 	0X0A 

void FSL_MMA_init();										// IIC hardware init
void FSL_MMA_IICWrite(VUINT8 RegAdd, VUINT8 Data);			// Write 1 byte
VINT8 FSL_MMA_IICRead(VUINT8 RegAdd);						// Read 1 byte
void FSL_MMA_ReadXYZ8(VINT8 *X, VINT8 *Y, VINT8 *Z);		// Read 8bit XYZ 
void FSL_MMA_ReadXYZ10(VINT16 *X, VINT16 *Y, VINT16 *Z);		// Read 10bit XYZ
void FSL_MMA_Testing_SCL_Waveform(void);					// Hardware check on SCL line (dump square wave to SCL line)
void FSL_MMA_Testing_SDA_Waveform(void);					// Hardware check on SDA line (dump square wave to SDA line)

#endif//__FSL_MMA_H__

