/*********************************************************************
 
  (c) copyright Freescale Semiconductor Ltd. 2008
  ALL RIGHTS RESERVED
 
********************************************************************
 Ver 2.0.1	Released Sep.2008
********************************************************************/
#ifndef __FSL_DATA_H__
#define __FSL_DATA_H__

typedef signed char VINT8;
typedef unsigned char VUINT8;
typedef signed short VINT16;
typedef unsigned short VUINT16;
typedef unsigned long VUINT32;
typedef signed long VINT32;

typedef struct {
  VINT16  X;
  VINT16  Y;
  VINT16  Z;  
} SPARAMETERS;

//#define MOTION_SENSOR_SUPPORT
//#define ACC_DEBUG 0

#endif//__FSL_DATA_H__