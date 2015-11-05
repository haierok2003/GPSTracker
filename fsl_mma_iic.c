/****************************************************************************
 
  (c) copyright Freescale Semiconductor Ltd. 2009
  ALL RIGHTS RESERVED
 
*****************************************************************************
 Ver 2.2.0	Released Jun.2009
*****************************************************************************

*****************************************************************************
*                                                                           *
* C O N F I D E N T I A L                                                   *
*                                                                           *
* This code is released under the Non-Disclosure Agreement (NDA) between    *
* customer and Freescale, and is allowed to use in Freescale's products     *
* only                                                                      *
*                                                                           *
*****************************************************************************
* THIS  CODE IS ONLY INTENDED AS AN EXAMPLE FOR DEMONSTRATING THE FREESCALE *
* MICROCONTROLLERS.  IT  HAS ONLY BEEN GIVEN A MINIMUM LEVEL OF TEST. IT IS *
* PROVIDED  'AS  SEEN'  WITH  NO  GUARANTEES  AND  NO  PROMISE  OF SUPPORT. *
*****************************************************************************
*                                                                           *
*****************************************************************************
* Freescale  is  not  obligated  to  provide  any  support, upgrades or new *
* releases  of  the Software. Freescale may make changes to the Software at *
* any time, without any obligation to notify or provide updated versions of *
* the  Software  to you. Freescale expressly disclaims any warranty for the *
* Software.  The  Software is provided as is, without warranty of any kind, *
* either  express  or  implied,  including, without limitation, the implied *
* warranties  of  merchantability,  fitness  for  a  particular purpose, or *
* non-infringement.  You  assume  the entire risk arising out of the use or *
* performance of the Software, or any systems you design using the software *
* (if  any).  Nothing  may  be construed as a warranty or representation by *
* Freescale  that  the  Software  or  any derivative work developed with or *
* incorporating  the  Software  will  be  free  from  infringement  of  the *
* intellectual property rights of third parties. In no event will Freescale *
* be  liable,  whether in contract, tort, or otherwise, for any incidental, *
* special,  indirect, consequential or punitive damages, including, but not *
* limited  to,  damages  for  any loss of use, loss of time, inconvenience, *
* commercial loss, or lost profits, savings, or revenues to the full extent *
* such  may be disclaimed by law. The Software is not fault tolerant and is *
* not  designed,  manufactured  or  intended by Freescale for incorporation *
* into  products intended for use or resale in on-line control equipment in *
* hazardous, dangerous to life or potentially life-threatening environments *
* requiring  fail-safe  performance,  such  as  in the operation of nuclear *
* facilities,  aircraft  navigation  or  communication systems, air traffic *
* control,  direct  life  support machines or weapons systems, in which the *
* failure  of  products  could  lead  directly to death, personal injury or *
* severe  physical  or  environmental  damage  (High  Risk Activities). You *
* specifically  represent and warrant that you will not use the Software or *
* any  derivative  work of the Software for High Risk Activities.           *
* Freescale  and the Freescale logos are registered trademarks of Freescale *
* Semiconductor Inc.                                                        *
****************************************************************************/
/*INTELIOT*/
#include "gpio_hw.h"
#include "gpio_sw.h"
#include "sccb_v2.h"
#include "FSL_MMA.h"

//#define ACC_DEBUG
//#define HW_I2C

#if defined(__GPS_BAT_CONNECT__) && defined(__BAT_TRACE__) || defined(__GPS_MCU_CONNECT__)
#define IIC_DEBUG
#endif


#ifdef ACC_DEBUG
#include "kal_release.h"
#define MOD_MMA MOD_MMI 
#define prompt_trace kal_prompt_trace
#endif 

extern void GPIO_ModeSetup(unsigned short pin, unsigned short conf_dada);
extern void GPIO_InitIO(char direction, kal_uint16 port);
extern void GPIO_WriteIO(char data, kal_uint16 port);
extern char GPIO_ReadIO(kal_uint16 port);

/****************************************************************************/
#define SET_I2C_CLK_OUTPUT		GPIO_InitIO(1,ACC_SENSOR_SCK)	//Set I2C CLK pin as output (Need to change)
#define SET_I2C_DATA_OUTPUT		GPIO_InitIO(1,ACC_SENSOR_SDA)	//Set I2C DATA pin as output (Need to change)
#define SET_I2C_DATA_INPUT		GPIO_InitIO(0,ACC_SENSOR_SDA)	//Set I2C DATA pin as input (Need to change)
#define SET_I2C_CLK_HIGH		GPIO_WriteIO(1,ACC_SENSOR_SCK)	//I2C CLK pin output high(1) (Need to change)
#define SET_I2C_CLK_LOW			GPIO_WriteIO(0,ACC_SENSOR_SCK)	//I2C CLK pin output low(0) (Need to change)
#define SET_I2C_DATA_HIGH		GPIO_WriteIO(1,ACC_SENSOR_SDA)	//I2C DATA pin output high(1) (Need to change)
#define SET_I2C_DATA_LOW		GPIO_WriteIO(0,ACC_SENSOR_SDA)	//I2C DATA pin output low(0) (Need to change)
#define GET_I2C_DATA_BIT        GPIO_ReadIO(ACC_SENSOR_SDA)

//extern prompt_trace(VINT16 mode, VUINT8 *text);		//Debug mode dump trace code


#define _FSL_MMA_7660_

#ifdef _FSL_MMA_7455_
#define FSL_MMA_AddW 0x3A          //MMA745x Address for Writing
#define FSL_MMA_AddR 0x3B          //MMA745x Address for Reading
#define FSL_MMA_StartAddress 0x06  //MMA745x g-value registers' start address
#endif

#ifdef _FSL_MMA_7660_
#define FSL_MMA_AddW 0x98          //MMA7660 Address for Writing
#define FSL_MMA_AddR 0x99          //MMA7660 Address for Reading
#define FSL_MMA_StartAddress 0x00  //MMA7660 g-value registers' start address
#endif

#ifdef __GPS_BAT_CONNECT__
#define FSL_BAT_AddW 0x16          //battery Address for Writing
#define FSL_BAT_AddR 0x17          //battery Address for Reading
#endif

/*
#define Delay20us 	95              //Loop times to delay 20uS (Need to change according to system clock)
#define Delay10us 	45              //Loop times to delay 45uS (Need to change according to system clock)
#define Delay5us	21              //Loop times to delay 21uS (Need to change according to system clock)
#define Delay2p5us	10              //Loop times to delay 2.5uS (Need to change according to system clock)
*/

#if defined( __GPS_BAT_CONNECT__) || defined(__GPS_MCU_CONNECT__)
#define BatDelay20us 	1032              //Loop times to delay 20uS (Need to change according to system clock)
#define BatDelay10us 	516            //Loop times to delay 45uS (Need to change according to system clock)
#define BatDelay5us	258              //Loop times to delay 21uS (Need to change according to system clock)
#define BatDelay2p5us	129              //Loop times to delay 2.5uS (Need to change according to system clock)
#define BatDelay100us 5160
#define Wait10ms    129000	

#define SET_IIC_CLK_OUTPUT	GPIO_InitIO(1, BAT_SCK)
#define SET_IIC_CLK_INPUT GPIO_InitIO(0, BAT_SCK)
#define SET_IIC_CLK_HIGH GPIO_WriteIO(1, BAT_SCK)
#define SET_IIC_CLK_LOW GPIO_WriteIO(0, BAT_SCK)
#define SET_IIC_SDA_OUTPUT GPIO_InitIO(1, BAT_SDA)
#define SET_IIC_SDA_INPUT GPIO_InitIO(0, BAT_SDA)
#define SET_IIC_SDA_HIGH GPIO_WriteIO(1, BAT_SDA)
#define SET_IIC_SDA_LOW GPIO_WriteIO(0, BAT_SDA)
#define GET_IIC_SDA_DATA GPIO_ReadIO(BAT_SDA)
#endif

#define Delay20us 	258              //Loop times to delay 20uS (Need to change according to system clock)
#define Delay10us 	129            //Loop times to delay 45uS (Need to change according to system clock)
#define Delay5us	64              //Loop times to delay 21uS (Need to change according to system clock)
#define Delay2p5us	32              //Loop times to delay 2.5uS (Need to change according to system clock)


void FSL_MMA_init() {                     //FSL Motion Sensor Initialize

#ifdef HW_I2C
	kal_uint32 data;

	GPIO_ModeSetup(ACC_SENSOR_SCK, 0x02);
	GPIO_ModeSetup(ACC_SENSOR_SCK, 0x02);
/*
	//设置 I2C 寄存器
	data = DRV_GPIO_Reg32(GPIO_SPMODE1);
	data |= 0x00000550;
	data &= 0xFFFFFDDF;
	DRV_GPIO_WriteReg32(GPIO_SPMODE1,data);*/
#else
	GPIO_ModeSetup(ACC_SENSOR_SCK, 0x00);   //Set I2C CLK pin as GPIO   
	SET_I2C_CLK_OUTPUT;                     //Set I2C CLK pin as output
	SET_I2C_CLK_HIGH;                       //I2C CLK pin output high(1)

	GPIO_ModeSetup(ACC_SENSOR_SDA, 0x00);   //Set I2C DATA pin as GPIO
	SET_I2C_DATA_OUTPUT;                    //Set I2C DATA pin as output
	SET_I2C_DATA_HIGH;                      //I2C DATA pin output high(1)
#endif

	FSL_MMA_IICWrite(MMA_MODE,0X00);	//配置寄存器的时候,必须进入standby模式 
	FSL_MMA_IICWrite(MMA_SRST,0X03);	//使能 AMSR AWSR分频计数器
	//每秒检测8次翻转，发生一次，更新横竖状态
	FSL_MMA_IICWrite(MMA_SR,0X14);	
#ifdef __ACC_EINT_MODE__
	FSL_MMA_IICWrite(MMA_INTSU,0xE7); 
#else	
	FSL_MMA_IICWrite(MMA_INTSU,0X00); 	//00:全关;E7:3轴震动中断全开 前后 tap  up、down、right、left 中断全开
#endif
	FSL_MMA_IICWrite(MMA_PDET,0X01);    //连续检测1个tap才算tap发生
	FSL_MMA_IICWrite(MMA_PD,0X00);    	//检测到连续2个tap 才会更新 TILT
	FSL_MMA_IICWrite(MMA_MODE,0X59);	//中断上拉； auto-wake auto-sleep全开；active mode  	

}

void mma_delay(VUINT16 time) {             //Time delay function
//	VUINT8 i;
	VUINT16 i;

	for (i=0; i<time; i++) ;                //Software loop for time delay
}

#if defined(__GPS_BAT_CONNECT__) || defined(__GPS_MCU_CONNECT__)
#define POLYNOME            0x07                // Polynome for CRC generation
VUINT8 GetCrc8(VUINT8 chkSum, VUINT8 crcData)
{
    VUINT8 j = 8;        // Counter for 8 shifts
 
    chkSum ^= crcData;              // Initial XOR
    do
    {
        if (!(chkSum & 0x80))        // Check MSB
        {
            chkSum = chkSum << 1;   // If MSB = 0, shift left
        }
        else
        {
            chkSum = (chkSum << 1) ^ POLYNOME;  // If MSB = 1, shift and XOR
        }
    } while (--j);                               // Continue for 8 bits        
    return chkSum;                             // Return final value    
}

void I2C_Start(void)
{
/*	GPIO_ModeSetup(BAT_SCK, 0);
	GPIO_ModeSetup(BAT_SDA, 0);*/

	SET_IIC_CLK_OUTPUT;
	SET_IIC_SDA_OUTPUT;

	SET_IIC_CLK_HIGH;
	SET_IIC_SDA_HIGH;

	mma_delay(BatDelay20us);
	SET_IIC_SDA_LOW;
	mma_delay(BatDelay10us);
	SET_IIC_CLK_LOW;
	mma_delay(BatDelay100us);
}
void I2C_ReStart(void)
{
	mma_delay(BatDelay100us);
	SET_IIC_SDA_HIGH;
	mma_delay(BatDelay10us);
	SET_IIC_CLK_HIGH;
	
	mma_delay(BatDelay100us);
	SET_IIC_SDA_LOW;
	mma_delay(BatDelay10us);
	SET_IIC_CLK_LOW;
	mma_delay(BatDelay100us);
}
void I2C_Stop(void)
{
	SET_IIC_SDA_OUTPUT;
	SET_IIC_SDA_LOW;
	
	mma_delay(BatDelay10us);
	SET_IIC_CLK_HIGH;
	mma_delay(BatDelay10us);
	SET_IIC_SDA_HIGH;
	mma_delay(BatDelay100us);
}
void I2C_SendByte(VUINT8 data)
{
	VINT8 i;
	for(i=7; i>=0; i--)
	{
		if((data>>i)&0x01)
		{
			SET_IIC_SDA_HIGH;  
		}
		else
		{	
			SET_IIC_SDA_LOW;
		}
		mma_delay(BatDelay5us);
		SET_IIC_CLK_HIGH;
		mma_delay(BatDelay10us);
		SET_IIC_CLK_LOW;
		mma_delay(BatDelay5us);
	}
}

VUINT8 I2C_ReadByte(void)
{
	VINT8 i,data=0;

	SET_IIC_SDA_INPUT;
	for(i=7; i>=0; i--)
	{
		mma_delay(BatDelay5us);
		SET_IIC_CLK_HIGH;
		mma_delay(BatDelay5us);
		if(GET_IIC_SDA_DATA)
			data |=(0x01<<i);
		mma_delay(BatDelay5us);
		SET_IIC_CLK_LOW;
		mma_delay(BatDelay5us);
	}
	
	//don't have 9' clk
	return data;
}
VUINT8 I2C_ReadByteACK(void)
{
	VINT8 i,data=0;

	SET_IIC_SDA_INPUT;
	for(i=7; i>=0; i--)
	{
		mma_delay(BatDelay5us);
		SET_IIC_CLK_HIGH;
		mma_delay(BatDelay5us);
		if(GET_IIC_SDA_DATA)
			data |=(0x01<<i);
		mma_delay(BatDelay5us);
		SET_IIC_CLK_LOW;
		mma_delay(BatDelay5us);
	}
	SET_IIC_SDA_OUTPUT;
	SET_IIC_SDA_LOW;
	mma_delay(BatDelay5us);
	SET_IIC_CLK_HIGH;
	mma_delay(BatDelay10us);
	SET_IIC_CLK_LOW;
	mma_delay(BatDelay10us);
	
	return data;
}
VUINT8 I2C_ReadByteNCK(void)
{
	VINT8 i,data=0;

	SET_IIC_SDA_INPUT;
	for(i=7; i>=0; i--)
	{
		mma_delay(BatDelay5us);
		SET_IIC_CLK_HIGH;
		mma_delay(BatDelay5us);
		if(GET_IIC_SDA_DATA)
			data |=(0x01<<i);
		mma_delay(BatDelay5us);
		SET_IIC_CLK_LOW;
		mma_delay(BatDelay5us);
	}
	
	SET_IIC_SDA_OUTPUT;
	SET_IIC_SDA_HIGH;
	mma_delay(BatDelay5us);
	SET_IIC_CLK_HIGH;
	mma_delay(BatDelay10us);
	SET_IIC_CLK_LOW;
	mma_delay(BatDelay10us);
	return data;
}
void I2C_Send_WR_FLAG(VUINT8 flag)
{
	SET_IIC_SDA_OUTPUT;
			
	if(flag){//read
		SET_IIC_SDA_HIGH;
	}else{//write
		SET_IIC_SDA_LOW;
	}
	mma_delay(BatDelay5us);
	SET_IIC_CLK_HIGH;

	mma_delay(BatDelay10us);
	SET_IIC_CLK_LOW;
	mma_delay(BatDelay5us);
}
VUINT8 I2C_ChkAck(void)
{
	kal_uint32 count = Wait10ms;
	SET_IIC_SDA_INPUT;
	
	mma_delay(BatDelay5us);
	SET_IIC_CLK_HIGH;
	mma_delay(BatDelay5us);

	while(count--)
	{
		if(!GET_IIC_SDA_DATA)	//success
		{
			mma_delay(BatDelay5us);
			SET_IIC_CLK_LOW;
			mma_delay(BatDelay5us);
			SET_IIC_SDA_OUTPUT;
			SET_IIC_SDA_LOW;
			mma_delay(BatDelay100us);
			return 0;
		}
	}

	//ack fail
	mma_delay(BatDelay5us);
	SET_IIC_CLK_LOW;
	mma_delay(BatDelay5us);
	SET_IIC_SDA_OUTPUT;
	SET_IIC_SDA_LOW;
	return 1;
}
#endif

void IIC_Start(void) {                    //I2C Start signal generation: Data pin falls down when clock is high
	GPIO_ModeSetup(ACC_SENSOR_SCK, 0x00);   //Set I2C CLK pin as GPIO
	SET_I2C_CLK_OUTPUT;                     //Set I2C CLK pin as output
	GPIO_ModeSetup(ACC_SENSOR_SDA, 0x00);   //Set I2C DATA pin as GPIO
	SET_I2C_DATA_OUTPUT;                    //Set I2C DATA pin as output

	SET_I2C_DATA_HIGH;	                    //I2C DATA pin output high(1)
	SET_I2C_CLK_HIGH;		                    //I2C CLK pin output high(1)
	mma_delay(Delay20us);                   //Delay 20uS
	SET_I2C_DATA_LOW;                       //I2C DATA pin output low(0)
	mma_delay(Delay10us);                   //Delay 10uS
	SET_I2C_CLK_LOW;                        //I2C CLK pin output low(0)
	mma_delay(Delay10us);                   //Delay 10uS
}

void IIC_Stop (void) {                    //I2C Stop signal generation: Data pin rises up when clock in is high
	mma_delay(Delay10us);                   //Delay 10uS
	SET_I2C_CLK_HIGH;		                    //I2C CLK pin output high(1)
	mma_delay(Delay10us);                   //Delay 10uS
	SET_I2C_DATA_HIGH;	                    //I2C DATA pin output high(1)
}

void IIC_RepeatedStart(void) {            //I2C Repeat Start signal generation: Data pin falls down when clock is high
	mma_delay(Delay20us);                   //Delay 20uS
	mma_delay(Delay20us);                   //Delay 20uS
	SET_I2C_DATA_HIGH;	                    //I2C DATA pin output high(1)
	mma_delay(Delay10us);                   //Delay 10uS
	SET_I2C_CLK_HIGH;		                    //I2C CLK pin output high(1)
	mma_delay(Delay20us);                   //Delay 20uS
	mma_delay(Delay20us);                   //Delay 20uS
	SET_I2C_DATA_LOW;                       //I2C DATA pin output low(0)
	mma_delay(Delay10us);                   //Delay 10uS
	SET_I2C_CLK_LOW;                        //I2C CLK pin output low(0)
	mma_delay(Delay10us);                   //Delay 10uS	
}

void IIC_OneClk(void) {                   //I2C CLK pin output one clock: CLK pin rises up before falls down
	mma_delay(Delay5us);
	SET_I2C_CLK_HIGH;		                    //I2C CLK pin output high(1)		
	mma_delay(Delay10us);                   //Delay 10uS
	SET_I2C_CLK_LOW;                        //I2C CLK pin output low(0)
	mma_delay(Delay5us);	
}

void IIC_SendByte(VUINT8 sData) {         //I2C send one byte out
	VINT8 i;
	for (i=7; i>=0; i--) {                  //Loop 8 times to send 8 bits
		if ((sData>>i)&0x01) {                //Judge output 1 or 0
			SET_I2C_DATA_HIGH;	                //I2C DATA pin output high(1) if output 1
		} else { 
			SET_I2C_DATA_LOW;                   //I2C DATA pin output low(0) if output 0
			}
		IIC_OneClk();                         //Output one clock pulse after data pin is ready
	}		
}

VUINT8  IIC_ChkAck(void) {                //Check I2C Acknowledgement signal
	SET_I2C_DATA_INPUT;                     //Set I2C DATA pin as input
	mma_delay(Delay5us);                    //Delay 5uS
	SET_I2C_CLK_HIGH;		                    //I2C CLK pin output high(1)		
	mma_delay(Delay5us);                    //Delay 5uS again
	if (GET_I2C_DATA_BIT) {                 //Read I2C DATA pin
		mma_delay(Delay5us);                  //Delay 5uS
		SET_I2C_CLK_LOW;                      //I2C CLK pin output low(0)
		mma_delay(Delay5us);                  //Delay 5us again
		SET_I2C_DATA_OUTPUT;                  //Set I2C DATA pin as output
		SET_I2C_DATA_LOW;                     //I2C DATA pin output low(0)
		return 1;                             //Return 1 if read 1 from I2C DATA pin
		} else {                              //If I2C DATA pin is invalid for acknowledgement signal
		mma_delay(Delay5us);                  //Delay 5uS
		SET_I2C_CLK_LOW;                      //I2C CLK pin output low(0)
		mma_delay(Delay5us);                  //Delay 5uS again
		SET_I2C_DATA_OUTPUT;                  //Set I2C DATA pin as output
		SET_I2C_DATA_LOW;                     //I2C DATA pin output low(0)
		return 0;                             //Return 0 if read 0 from I2C DATA pin
		}			
}

VUINT8 IIC_ReadByteACK(void) {            //Read one byte and send an acknowledgement signal
	VINT8 i;
	VUINT8 data;

	SET_I2C_DATA_INPUT;                     //Set I2C DATA pin as input
	data = 0;                               //Prepare to receive data
	for (i=7; i>=0; i--) {                  //Loop 8 times to receive 8 bits
		if (GET_I2C_DATA_BIT) data |= (0x01<<i);    //If read a 1, set to data bit
		IIC_OneClk();
		}			                                //Output one clock pulse after a bit is read

	SET_I2C_DATA_OUTPUT;                    //Set I2C DATA pin as output
	SET_I2C_DATA_LOW;                       //I2C DATA pin output low(0): the acknowledgement signal
	IIC_OneClk();                           //Output one clock pulse after data pin is ready

	return data;                            //Return eceived data
}

VUINT8 IIC_ReadByteNCK(void) {            //Read one byte but do not send acknowledgement signal
	VINT8 i;
	VUINT8 data;

	SET_I2C_DATA_INPUT;                     //Set I2C DATA pin as input
	data = 0;                               //Prepare to receive data
	for (i=7; i>=0; i--) {                  //Loop 8 times to receive 8 bits
		if (GET_I2C_DATA_BIT) data |= (0x01<<i);    //If read a 1, set to data bit
		IIC_OneClk();
		}			                                //Output one clock pulse after a bit is read

	SET_I2C_DATA_OUTPUT;                    //Set I2C DATA pin as output
	SET_I2C_DATA_HIGH;	                    //I2C DATA pin output high(1): no acknowledge
	IIC_OneClk();                           //Output one clock pulse after data pin is ready
	SET_I2C_DATA_LOW;                       //I2C DATA pin output low(0)

	return data;                            //Return received data
}

#ifdef __GPS_BAT_CONNECT__
void FSL_bat_IICWrite(VUINT8 RegAdd, VUINT8 Data) {   //Write one byte to a sensor register via I2C

	//Start
	I2C_Start();                                        //Output a START signal

	// Device hardware address
	I2C_SendByte(FSL_BAT_AddW);                         //Send one byte of sensor IIC address for writing
	if (I2C_ChkAck()) {                                 //Check acknowledge signal
   	#ifdef IIC_DEBUG	                                //Only for debug
		kal_prompt_trace(MOD_WPS, "# BATTERY Device Write Address Error #\r\n");   //Print error information
	#endif 
		I2C_Stop();	                                      //Output a STOP signal
		return;                                           //If acknowledgement signal is read as 1, then return to end
		}
                                                      //If acknowledgement signal is read as 0, then go to next step
	// Register address to read                         
	I2C_SendByte(RegAdd);                               //Send one byte of register address in the sensor
	if (I2C_ChkAck()) {                                 //Check acknowledgement signal
   	#ifdef IIC_DEBUG	                                //Only for debug
		kal_prompt_trace(MOD_WPS, "# BATTERY Reg Address NACK #\r\n");   //Print error information
	#endif                                              
		I2C_Stop();                                       //Output a STOP signal
		return;    	                                      //If acknowledgement signal is read as 1, then return to end
		}
	
                                                      //If acknowledgement signal is read as 0, then go to next step
	// Data to send
	I2C_SendByte(Data);                                 //Send one byte of data
	if (I2C_ChkAck()) {                                 //Check acknowledgement signal
   	#ifdef IIC_DEBUG	                                //Only for debug
		kal_prompt_trace(MOD_WPS, "# Sensor Data NACK #\r\n");    //Print error information
	#endif
		I2C_Stop();	                                      //Output a STOP signal
		return;                                           //If acknowledgement signal is read as 1, then return to end
		}
                                                      //If acknowledgement signal is read as 0, then go to next step
	// Stop	
	I2C_Stop();	                                        //Output a STOP signal	
}
VUINT16 FSL_bat_IICReadWord(VUINT8 RegAdd){                //Read a byte from sensor register via I2C
	VUINT16 data,dataHigh,dataLow;
	VUINT8 checkSum = 0;
	VUINT8 readCheckSum = 0;

	//Start
	I2C_Start();                                        //Output a START signal
                                                      
	// Device hardware address
	I2C_SendByte(FSL_BAT_AddW);                         //Send a byte of sensor IIC address for writing
	if (I2C_ChkAck()) {                                 //Check acknowledge signal 
   	#ifdef IIC_DEBUG	                                //Only for debug
		kal_prompt_trace(MOD_WPS, "# Battery Device Write Address Error #\r\n");   //Print error information
	#endif
		I2C_Stop();	                                      //Output a STOP signal	
		return 0;                                         //If acknowledgement signal is read as 1, then return to end
		}
                                                      //If acknowledgement signal is read as 0, then go to next step
	// Register address to read
	I2C_SendByte(RegAdd);                               //Send one byte of register address in the sensor
	if (I2C_ChkAck()) {                                 //Check acknowledge signal 
   	#ifdef IIC_DEBUG	                                //Only for debug
		kal_prompt_trace(MOD_WPS, "# Battery Reg Address NACK #\r\n");   //Print error information
	#endif
		I2C_Stop();	                                      //Output a STOP signal	
		return 0;                                         //If acknowledgement signal is read as 1, then return to end
		}

                                                      //If acknowledgement signal is read as 0, then go to next step
	// Repeated Start
	I2C_ReStart();                                //Output a REPEAT START signal
	
	// Device hardware address                          
	I2C_SendByte(FSL_BAT_AddR);                         //Send the sensor IIC address for reading
	if (I2C_ChkAck()) {                                 //Check acknowledge signal 
   	#ifdef IIC_DEBUG	                                //Only for debug
		kal_prompt_trace(MOD_WPS, "#Battery Device Read Address Error #\r\n");    //Print error information
	#endif
		}                                                 //If acknowledgement signal is read as 1, do nothing
                                                      //If acknowledgement signal is read as 0, then go to next step
                                                      
	dataLow = I2C_ReadByteACK();                           //Read one byte but do not output acknowledgement
	dataHigh = I2C_ReadByteACK();
	
	readCheckSum = I2C_ReadByteNCK();

	// Stop	
	I2C_Stop();	                                        //Output a STOP signal

	checkSum = GetCrc8(checkSum,FSL_BAT_AddW);
	checkSum = GetCrc8(checkSum,RegAdd);
	checkSum = GetCrc8(checkSum,FSL_BAT_AddR);
	checkSum = GetCrc8(checkSum,dataLow);
	checkSum = GetCrc8(checkSum,dataHigh);

#ifdef IIC_DEBUG
	kal_prompt_trace(MOD_WPS, "checkSum=%x,readCheckSum=%x\r\n",checkSum, readCheckSum);
#endif

	if(checkSum == readCheckSum)
	{
		data = (dataHigh<<8)&0xff00;
		data += dataLow&0x00ff;
#ifdef IIC_DEBUG
	kal_prompt_trace(MOD_WPS, "IIC read data=%d\r\n",data);
#endif
		
	}
	else
	{
		data = 0xffff;
	}
	
	return data;                                        //Return received data
}
VUINT8 FSL_bat_IICReadBlock(VUINT8 RegAdd,VUINT8* buf,VUINT8 bufSize){                //Read a byte from sensor register via I2C
	VINT8 len,i;
	VUINT8 checkSum = 0;
	VUINT8 readCheckSum = 0;

	//Start
	I2C_Start();                                        //Output a START signal
                                                      
	// Device hardware address
	I2C_SendByte(FSL_BAT_AddW);                         //Send a byte of sensor IIC address for writing
	if (I2C_ChkAck()) {                                 //Check acknowledge signal 
   	#ifdef IIC_DEBUG	                                //Only for debug
		kal_prompt_trace(MOD_WPS, "# Battery Device Write Address Error #\r\n");   //Print error information
	#endif
		I2C_Stop();	                                      //Output a STOP signal	
		return;                                         //If acknowledgement signal is read as 1, then return to end
		}
                                                      //If acknowledgement signal is read as 0, then go to next step
	// Register address to read
	I2C_SendByte(RegAdd);                               //Send one byte of register address in the sensor
	if (I2C_ChkAck()) {                                 //Check acknowledge signal 
   	#ifdef IIC_DEBUG	                                //Only for debug
		kal_prompt_trace(MOD_WPS, "# Battery Reg Address NACK #\r\n");   //Print error information
	#endif
		I2C_Stop();	                                      //Output a STOP signal	
		return;                                         //If acknowledgement signal is read as 1, then return to end
		}
                                                      //If acknowledgement signal is read as 0, then go to next step
	// Repeated Start
	I2C_ReStart();                                //Output a REPEAT START signal
	
	// Device hardware address                          
	I2C_SendByte(FSL_BAT_AddR);                         //Send the sensor IIC address for reading
	if (I2C_ChkAck()) {                                 //Check acknowledge signal 
   	#ifdef IIC_DEBUG	                                //Only for debug
		kal_prompt_trace(MOD_WPS, "#Battery Device Read Address Error #\r\n");    //Print error information
	#endif
		}                                                 //If acknowledgement signal is read as 1, do nothing
                                                      //If acknowledgement signal is read as 0, then go to next step

	len = I2C_ReadByteACK();
	for(i=0; i<len; i++)
	{
		buf[i] = I2C_ReadByteACK();
	}
	
	readCheckSum = I2C_ReadByteNCK();
	// Stop	
	I2C_Stop();	                                        //Output a STOP signal

	
	checkSum = GetCrc8(checkSum, FSL_BAT_AddW);
	checkSum = GetCrc8(checkSum, RegAdd);
	checkSum = GetCrc8(checkSum, FSL_BAT_AddR);
	checkSum = GetCrc8(checkSum, len);

	for(i=0; i<len; i++)
	{
		checkSum = GetCrc8(checkSum,buf[i]);
	}

#ifdef IIC_DEBUG
	kal_prompt_trace(MOD_WPS, "checkSum=%x,readCheckSum=%x\r\n",checkSum, readCheckSum);
#endif

	if(checkSum == readCheckSum)	//correct data
	{
		return len;
	}
	else	//error
	{
		memset(buf, 0, bufSize);
		return 0;
	}
//	return Data;                                        //Return received data
}
#endif

#ifdef __GPS_MCU_CONNECT__
U8 MCU_Get_I2C_CTR_PIN_DATA(void)
{
	U8 data;
	data = GPIO_ReadIO(MCU_START_READY);
	return data;
}
void MCU_IICWriteData(VUINT8 data)
{
	kal_uint32 count = Wait10ms;

	I2C_Start();                                        //Output a START signal	
	while(count--)
	{
		if(MCU_Get_I2C_CTR_PIN_DATA())
			break;
	}
	I2C_Send_WR_FLAG(0);
	I2C_SendByte(data);
	I2C_Stop();
}

VUINT8 MCU_IICReadData(void)
{
	kal_uint32 count = Wait10ms;
	VUINT8 data;
	
	I2C_Start();
	while(count--)
	{
		if(MCU_Get_I2C_CTR_PIN_DATA())
			break;
	}
	I2C_Send_WR_FLAG(1);
	data = I2C_ReadByte();
	I2C_Stop();
	
	return data;
}
#endif
// Master Write
void FSL_MMA_IICWrite(VUINT8 RegAdd, VUINT8 Data) {   //Write one byte to a sensor register via I2C

#ifdef HW_I2C
    sccb_config_struct config;
    i2c_init();
    /***************************Config SCCB for ACCE***************************/
	DRV_I2C_WriteReg16(REG_I2C_IO_CONFIG,0x000b);
	
    config.sccb_mode= SCCB_HW_8BIT ; 
    config.get_handle_wait=KAL_TRUE;	 
    config.slave_address= FSL_MMA_AddW; 
    config.delay_len = 0 ;
    config.transaction_mode=SCCB_TRANSACTION_FAST_MODE;
    config.Fast_Mode_Speed=50;
    i2c_config(SCCB_OWNER_ACCE,&config);
    sccb_write(SCCB_OWNER_ACCE, RegAdd, Data);
#else 
	//Start
	IIC_Start();                                        //Output a START signal

	// Device hardware address
	IIC_SendByte(FSL_MMA_AddW);                         //Send one byte of sensor IIC address for writing
	if (IIC_ChkAck()) {                                 //Check acknowledge signal
   	#ifdef ACC_DEBUG	                                //Only for debug
		prompt_trace(MOD_MMA, "# Device Write Address Error #\r\n");   //Print error information
	#endif
		IIC_Stop();	                                      //Output a STOP signal
		return;                                           //If acknowledgement signal is read as 1, then return to end
		}
                                                      //If acknowledgement signal is read as 0, then go to next step
	// Register address to read                         
	IIC_SendByte(RegAdd);                               //Send one byte of register address in the sensor
	if (IIC_ChkAck()) {                                 //Check acknowledgement signal
   	#ifdef ACC_DEBUG	                                //Only for debug
		prompt_trace(MOD_MMA, "# Sensor Reg Address NACK #\r\n");   //Print error information
	#endif                                              
		IIC_Stop();                                       //Output a STOP signal
		return;    	                                      //If acknowledgement signal is read as 1, then return to end
		}
                                                      //If acknowledgement signal is read as 0, then go to next step
	// Data to send
	IIC_SendByte(Data);                                 //Send one byte of data
	if (IIC_ChkAck()) {                                 //Check acknowledgement signal
   	#ifdef ACC_DEBUG	                                //Only for debug
		prompt_trace(MOD_MMA, "# Sensor Data NACK #\r\n");    //Print error information
	#endif
		IIC_Stop();	                                      //Output a STOP signal
		return;                                           //If acknowledgement signal is read as 1, then return to end
		}
                                                      //If acknowledgement signal is read as 0, then go to next step
	// Stop	
	IIC_Stop();	                                        //Output a STOP signal	
#endif
}

// Master Read
VINT8 FSL_MMA_IICRead(VUINT8 RegAdd) {                //Read a byte from sensor register via I2C
	VINT8 Data;

#ifdef HW_I2C
		sccb_config_struct config;
		i2c_init();
		/***************************Config SCCB for ACCE***************************/
		config.sccb_mode= SCCB_HW_8BIT;
		config.get_handle_wait=KAL_TRUE; 
		config.slave_address= FSL_MMA_AddR;
		config.delay_len = 0 ;
		config.transaction_mode=SCCB_TRANSACTION_FAST_MODE;
		config.Fast_Mode_Speed=50;//Kb
		i2c_config(SCCB_OWNER_ACCE,&config);
		Data = sccb_read(SCCB_OWNER_ACCE,RegAdd);
#else

	//Start
	IIC_Start();                                        //Output a START signal
                                                      
	// Device hardware address
	IIC_SendByte(FSL_MMA_AddW);                         //Send a byte of sensor IIC address for writing
	if (IIC_ChkAck()) {                                 //Check acknowledge signal 
   	#ifdef ACC_DEBUG	                                //Only for debug
		prompt_trace(MOD_MMA, "# Device Write Address Error #\r\n");   //Print error information
	#endif
		IIC_Stop();	                                      //Output a STOP signal	
		return 0;                                         //If acknowledgement signal is read as 1, then return to end
		}
                                                      //If acknowledgement signal is read as 0, then go to next step
	// Register address to read
	IIC_SendByte(RegAdd);                               //Send one byte of register address in the sensor
	if (IIC_ChkAck()) {                                 //Check acknowledge signal 
   	#ifdef ACC_DEBUG	                                //Only for debug
		prompt_trace(MOD_MMA, "# Sensor Reg Address NACK #\r\n");   //Print error information
	#endif
		IIC_Stop();	                                      //Output a STOP signal	
		return 0;                                         //If acknowledgement signal is read as 1, then return to end
		}
                                                      //If acknowledgement signal is read as 0, then go to next step
	// Repeated Start
	IIC_RepeatedStart();                                //Output a REPEAT START signal
	
	// Device hardware address                          
	IIC_SendByte(FSL_MMA_AddR);                         //Send the sensor IIC address for reading
	if (IIC_ChkAck()) {                                 //Check acknowledge signal 
   	#ifdef ACC_DEBUG	                                //Only for debug
		prompt_trace(MOD_MMA, "# Device Read Address Error #\r\n");    //Print error information
	#endif
		}                                                 //If acknowledgement signal is read as 1, do nothing
                                                      //If acknowledgement signal is read as 0, then go to next step
	Data = IIC_ReadByteNCK();                           //Read one byte but do not output acknowledgement
   	#ifdef ACC_DEBUG	                                //Only for debug
		prompt_trace(MOD_MMA, "# Sensor Read Data = %d #\r\n", Data);   //Print received data
	#endif

	// Stop	
	IIC_Stop();	                                        //Output a STOP signal
#endif
	return Data;                                        //Return received data
}

// Read 3 Continuous Registers
void FSL_MMA_ReadXYZ8(VINT8 *X, VINT8 *Y, VINT8 *Z) { //Read 3 continous bytes
#ifdef HW_I2C
	*X = FSL_MMA_IICRead(MMA_XOUT);
	*Y = FSL_MMA_IICRead(MMA_YOUT);
	*Z = FSL_MMA_IICRead(MMA_ZOUT);
#else
	//Start
	IIC_Start();                                        //Output a START signal

	// Device hardware address
	IIC_SendByte(FSL_MMA_AddW);                        //Send a byte of MMA7455 address for writing
	if (IIC_ChkAck()) {                                 //Check acknowledge signal 
   	#ifdef ACC_DEBUG	                                //Only for debug
		prompt_trace(MOD_MMA, "# MMA7455L Device Write Address Error #\r\n");   //Print error information
	#endif
		IIC_Stop();	                                      //Output a STOP signal	
		return;                                           //If acknowledgement signal is read as 1, then return to end
		}
                                                      //If acknowledgement signal is read as 0, then go to next step
	// Register address to read
	IIC_SendByte(FSL_MMA_StartAddress);                 //Send the start address to read
	if (IIC_ChkAck()) {                                 //Check acknowledge signal 
   	#ifdef ACC_DEBUG	                                //Only for debug	
		prompt_trace(MOD_MMA, "# MMA7455L Reg Address NACK #\r\n");
	#endif
		IIC_Stop();	                                      //Output a STOP signal	
		return;
		}

	// Repeated Start
	IIC_RepeatedStart();                                //Output a REPEAT START signal
	
	// Device hardware address
	IIC_SendByte(FSL_MMA_AddR);                        //Send a MMA7455 address for reading
	if (IIC_ChkAck()) {                                 //Check acknowledge signal 
   	#ifdef ACC_DEBUG	                                //Only for debug	
		prompt_trace(MOD_MMA, "# MMA7455L Device Read Address Error #\r\n");
	#endif
		IIC_Stop();	                                      //Output a STOP signal	
		return;                                           //If acknowledgement signal is read as 1, then return to end
		}
                                                      //If acknowledgement signal is read as 0, then go to next step
	*X = IIC_ReadByteACK();                             //Read one byte and output an acknowledgement signal
   	#ifdef ACC_DEBUG	                                //Only for debug	
		prompt_trace(MOD_MMA, "# MMA7455L Read X Data = %d #\r\n", *X);   //Print read value of X data
	#endif

	*Y = IIC_ReadByteACK();                             //Read one byte and output an acknowledgement signal
   	#ifdef ACC_DEBUG	                                //Only for debug	
		prompt_trace(MOD_MMA, "# MMA7455L Read Y Data = %d #\r\n", *Y);   //Print read value of Y data
	#endif

	*Z = IIC_ReadByteNCK();                             //Read one byte and output an acknowledgement signal
   	#ifdef ACC_DEBUG	                                //Only for debug	
		prompt_trace(MOD_MMA, "# MMA7455L Read Z Data = %d #\r\n", *Z);   //Print read value of Z data
	#endif
	// Stop	
	IIC_Stop();	                                        //Output a STOP signal	
#endif
}

// Read MMA7455 XYZ 10bit Registers
void FSL_MMA_ReadXYZ10(VINT16 *X, VINT16 *Y, VINT16 *Z) { //Read 6 continous bytes of X, Y and Z 10-bit data, start from address 0x00 on MMA7455
	VINT16 temp;
	//Start
	IIC_Start();                                        //Output a START signal

	// Device hardware address
	IIC_SendByte(FSL_MMA_AddW);                        //Send a byte of MMA7455 address for writing
	if (IIC_ChkAck()) {                                 //Check acknowledge signal 
   	#ifdef ACC_DEBUG	                                //Only for debug	
		prompt_trace(MOD_MMA, "# MMA7455L Device Write Address Error #\r\n");
	#endif
		IIC_Stop();	                                      //Output a STOP signal	
		return;                                           //If acknowledgement signal is read as 1, then return to end
		}
                                                      //If acknowledgement signal is read as 0, then go to next step
	// Register address to read
	IIC_SendByte(0x00);                                 //Send the start address to read
	if (IIC_ChkAck()) {                                 //Check acknowledge signal 
   	#ifdef ACC_DEBUG	                                //Only for debug	
		prompt_trace(MOD_MMA, "# MMA7455L Reg Address NACK #\r\n");   //Print error information
	#endif
		IIC_Stop();	                                      //Output a STOP signal	
		return;                                           //If acknowledgement signal is read as 1, then return to end
		}
                                                      //If acknowledgement signal is read as 0, then go to next step
	// Repeated Start
	IIC_RepeatedStart();                                //Output a REPEAT START signal
	
	// Device hardware address
	IIC_SendByte(FSL_MMA_AddR);                        //Send a MMA7455 address for reading
	if (IIC_ChkAck()) {                                 //Check acknowledge signal 
   	#ifdef ACC_DEBUG	                                //Only for debug	
		prompt_trace(MOD_MMA, "# MMA7455L Device Read Address Error #\r\n");    //Print error information
	#endif
		IIC_Stop();	                                      //Output a STOP signal	
		return;                                           //If acknowledgement signal is read as 1, then return to end
		}
                                                      //If acknowledgement signal is read as 0, then go to next step
	*X = (VINT16)IIC_ReadByteACK();                     //Read one byte (lower 8 bits of 10-bit data) and store in a 16-bit address
	temp = (VINT16)IIC_ReadByteACK();                   //Read the next byte (higher 2 bits of 10-bit data)
	*X += (temp<<8);                                    //Combine them to a 10-bit data
	if (*X&0x0200) *X |= 0xfc00;	                      //Change the 10-bit data into a 16-bit signed data
   	#ifdef ACC_DEBUG	                                //Only for debug	
		prompt_trace(MOD_MMA, "# MMA7455L Read X Data = %d #\r\n", *X);   //Print the read data
	#endif

	*Y = (VINT16)IIC_ReadByteACK();                     //Read one byte (lower 8 bits of 10-bit data) and store in a 16-bit address
	temp = (VINT16)IIC_ReadByteACK();                   //Read the next byte (higher 2 bits of 10-bit data)
	*Y += (temp<<8);                                    //Combine them to a 10-bit data
	if (*Y&0x0200) *Y |= 0xfc00;                        //Change the 10-bit data into a 16-bit signed data
   	#ifdef ACC_DEBUG	                                //Only for debug	
		prompt_trace(MOD_MMA, "# MMA7455L Read Y Data = %d #\r\n", *Y);
	#endif

	*Z = (VINT16)IIC_ReadByteACK();                     //Read one byte (lower 8 bits of 10-bit data) and store in a 16-bit address
	temp = (VINT16)IIC_ReadByteNCK();                   //Read the next byte (higher 2 bits of 10-bit data)
	*Z += (temp<<8);                                    //Combine them to a 10-bit data
	if (*Z&0x0200) *Z |= 0xfc00;                        //Change the 10-bit data into a 16-bit signed data
   	#ifdef ACC_DEBUG	                                //Only for debug	
		prompt_trace(MOD_MMA, "# MMA7455L Read Z Data = %d #\r\n", *Z);
	#endif
	// Stop	
	IIC_Stop();	                                        //Output a STOP signal	

}


void FSL_MMA_Testing_SCL_Waveform(void) {   //Function used to test waveforms on I2C CLK pin
	GPIO_ModeSetup(ACC_SENSOR_SCK, 0x00);     //Set I2C CLK pin as GPIO
	SET_I2C_CLK_OUTPUT;

	while (1) {
		SET_I2C_CLK_HIGH;		                    //I2C CLK pin output high(1)
		mma_delay(Delay20us);                   //Delay 20uS
		SET_I2C_CLK_LOW;                        //I2C CLK pin output low(0)
		mma_delay(Delay20us);                   //Delay 20uS

		SET_I2C_CLK_HIGH;		                    //I2C CLK pin output high(1)
		mma_delay(Delay10us);                   //Delay 10uS
		SET_I2C_CLK_LOW;                        //I2C CLK pin output low(0)
		mma_delay(Delay10us);                   //Delay 10uS

		SET_I2C_CLK_HIGH;		                    //I2C CLK pin output high(1)
		mma_delay(Delay5us);                    //Delay 5uS
		SET_I2C_CLK_LOW;                        //I2C CLK pin output low(0)
		mma_delay(Delay5us);                    //Delay 5uS
	}                                         //Infinite loop

}

void FSL_MMA_Testing_SDA_Waveform(void) {   //Function used to test waveforms on I2C DATA pin
	GPIO_ModeSetup(ACC_SENSOR_SCK, 0x00);     //Set I2C CLK pin as GPIO
	SET_I2C_DATA_OUTPUT;                      //Set I2C DATA pin as output

	while (1) {
		SET_I2C_DATA_HIGH;	                    //I2C DATA pin output high(1)
		mma_delay(Delay20us);                   //Delay 20uS
		SET_I2C_DATA_LOW;                       //I2C DATA pin output low(0)
		mma_delay(Delay20us);                   //Delay 20uS

		SET_I2C_DATA_HIGH;	                    //I2C DATA pin output high(1)
		mma_delay(Delay10us);                   //Delay 10uS
		SET_I2C_DATA_LOW;                       //I2C DATA pin output low(0)
		mma_delay(Delay10us);                   //Delay 10uS

		SET_I2C_DATA_HIGH;	                    //I2C DATA pin output high(1)
		mma_delay(Delay5us);                    //Delay 5uS
		SET_I2C_DATA_LOW;                       //I2C DATA pin output low(0)
		mma_delay(Delay5us);                    //Delay 5uS
	}                                         //Infinite loop

}



