
/*******************************************************
* gps tracker 
*********************************************************/
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	gpstracker.h
 *
 * Project:
 * --------
 *	MAUI
 *
 * Description:
 * ------------
 *	SMS Service Internal Prototype Header File
 *
 * Author:
 * -------
 * -------
 ****************************************************************************/

#ifndef __SRV_GPS_TRACKER_H__
#define __SRV_GPS_TRACKER_H__

#include "MMI_features.h"
#include "soc_api.h"
#include "smssrvgprot.h"

#include "kal_release.h"

#include "SmsProtSrv.h"

#include "c2java.h"

/*******************************************************
*�궨��
*********************************************************/
//#define __GPS_CONTROL_CONNECT__		//GPSģ������ư�֮��ͨ��
//#define __GPS_BAT_CONNECT__			//GPSģ�����ذ�֮��ͨ��
#define __DEL_CONNET_FAIL_RESTRART__	//ɾ������ʧ����������
#define __GPS_BEST_HDOP__				//�ϴ���Ѿ������ӵ�GPS����	//zzt.20150826.add
#define __LED_INDICATE_STATE__			//LED�Ʊ�ʶ��GPS/GSM״̬		//zzt.20150901.add
//#define __ACC_EINT_MODE__				//G-SENSOR�����ж�ģʽ,	//zzt.20150831.add
#define __GPS_MCU_CONNECT__			//GPSģ���뵥Ƭ��֮���ͨ��
#define __GPS_PACKAGE_BY_QUEUE__			//GPS�����ϱ������д洢�ϴ�	//zzt.20150925.add

#if defined(__GPS_CONTROL_CONNECT__)||defined(__GPS_MCU_CONNECT__)
#define __PROTOCOL_CONTROL__			//Э�鴫��ָ��
#endif


//#define DEBUG_DOMAIN	//���������


/*log ��Ϣ*/
#define __GPRS_TRACE__			//��GPRS�����log
#define __GPS_TRACE__			//GPS��log
//#define __G_SENSOR_TRACE__		//�𶯴�������log
//#define __SMS_TRACE__			//���ſ��Ƶ�log
//#define __CONTROL_TRACE__		//��������log
//#define __LBS_TRACE__			//LBS �����log
//#define __TRACE_WRITE_FS__		//logд���ļ�
//#define __BAT_TRACE__			//��Դ���log
//#define __MCU_TRACE__				//�����ǿ�����log
/*******************************************************
*Pin defination
*********************************************************/
//LED defination
#define gpio_gsm_led (63|0x80)
#define gpio_gps_led (62|0x80)

//uart2 defination
#define gpio_uart2_rx (20|0x80)
#define gpio_uart2_tx (21|0x80)

#ifdef __GPS_CONTROL_CONNECT__ 
#define gpio_uart1_rx (22|0x80)	//(20|0x80)
#define gpio_uart1_tx (23|0x80)	//(21|0x80)
#endif
#ifdef __GPS_MCU_CONNECT__
//#define MCU_RESULT_READY (1)
#define MCU_START_READY  (58)	//(2)	//zzt.debug
#endif
#if defined(__GPS_BAT_CONNECT__) || defined(__GPS_MCU_CONNECT__)
//IIC pin defination
#define BAT_SDA    (56|0x80)	//(18|0x80)	 
#define BAT_SCK    (19|0x80)
#endif

//IIC pin defination
#define ACC_SENSOR_SDA    (16|0x80)	 
#define ACC_SENSOR_SCK    (17|0x80)
#define ACC_INT_PIN (10|0x80)

//DC interrupt
#define DC_INT_NO  0//EINT0
#define DC_INPUT_DET_PIN_NO  0
#define DC_INPUT_DET_IO  (0|0x80)

//relay control
#define RELAY_CTL_IO (58|0x80)
#define RELAY_CTL_PIN_NO 58
/********************************************************/

#define MAX_GT_SEND_LEN 255

#define MAX_GT_RCV_LEN 512

//�ϱ�ʱ��
#define MAX_GT_GPRS_UPLOAD_TIME_INTERVAL (10*1000)


//��Ϣ���е���󳤶�
#define MAX_GT_MSG_QUEUE_LEN 10

//log buffer�ĳ���
#define MAX_GT_LOG_BUF_LEN 200

//log �ļ����������
#define MAX_GT_LOG_FILE_SIZE (5*1024)

//gps �����ļ�
#define MAX_GT_GPS_FILE_SIZE (20*1024)
#define MAX_ALARM_RESTORE_FILE_SIZE (20*1024)
//״̬����


#define MAX_GT_PARA_NUM 4

#define MAX_GT_PARA_LEN 32

#define MAX_GT_CMD_PREFIEX 4

#define MAX_GT_PROCESSING_TIMES 3
/***********************ϵͳ����ʱ�����****************************/
//DNSʱ����� 25*5 = 125 Լ2����
//DNS��ʱ�����
#define MAX_GT_DNS_CHECK_INTV (25*1000)
//DNS ���Դ���
#define MAX_GT_DNS_TRY_TIMES (3)

//��½ : 5*10 = 50s
//��½��ʱ�����
#define MAX_GT_LOGIN_INTV (15*1000)

#ifdef TEST
//��½���Դ���
#define MAX_GT_LOGIN_TRY_TIMES (4*20)
#else
#define MAX_GT_LOGIN_TRY_TIMES (3)
#endif

//����ʱ�����  10*6*20 = 1200 = 20����
//���Ӷ�ʱ�����
#define MAX_GT_CONN_INTV (30*1000)		//(10*1000)	//zzt.20150730.modify 30
//���� ���Դ���
#define MAX_GT_CONN_TRY_TIMES (6*5)// x�����޷�����������

#define MAX_GT_HB_INTV (120)	//(180) // 3 ����	//zzt.20150801

//����Ķ��Żظ���ʱҪ���Ƕ��Ŷ��б���Ĵ�����ʱ
#define MAX_GT_SMS_DELAY_REPLY_TIME (gps_tracker_config.sms_send_intv*1000 + 20*1000)

#define MAX_GT_SHAKE_DETECT_INTV   1*1000	//zzt.20150801

//����3���Ӿ�ֹ��ϵͳ�ͻ���뾲ֹ״̬
#define SHAKE_DETECT_COUNTS (6*5)

/********************************************************************/
#define MAX_GT_LED_REFRESH_INTV (250)

//���͵����ʧ�ܴ���
#ifdef _WIN32
#define MAX_GT_SEND_FAILED_TIMES 30//
#else
#define MAX_GT_SEND_FAILED_TIMES 5//
#endif

#define MOVING_VIBR_THRESHOLD (3) 


#define MAX_GT_CMD_LEN 32		//�� null-terminal
#define MAX_GT_ERR_DESC_LEN 32	//�� null-terminal

#define MAX_GT_PWD_LEN 7

#define MAX_GT_IMSI_LEN 17

#define PACKET_FRAME_LEN (sizeof(gps_tracker_msg_head_struct) + sizeof(gps_tracker_msg_tail_struct))

#define GPS_PACKET_SIZE (sizeof(gps_tracker_gps_req_content_struct) + PACKET_FRAME_LEN)
#define STATUS_PACKET_SIZE (sizeof(gps_tracker_status_req_content_struct) + PACKET_FRAME_LEN)
#define ALARM_PACKET_SIZE (sizeof(gps_tracker_alarm_req_content_struct) + PACKET_FRAME_LEN)

#define PACKET_START  0XFFFF
#define PACKET_STOP 0X0A0D


//sos �����б����
#define MAX_GT_SOS_COUNT (4)

// imei �ַ����ĳ��ȣ�������16
#define MAX_GT_IMEI_LEN (16) 

#define MAX_GT_ACC_NAME_LEN (32)

#ifdef DEBUG_DOMAIN
#define SERVER_PORT 17472
#else
#define SERVER_PORT 9000;	//9001;		
#endif

///////////////////////////////////////
//#define MIN(a, b) ((a) < (b) ? (a) : (b))
//#define MAX(a,b)  (((a) > (b)) ? (a) : (b))

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_trace
 * DESCRIPTION
 *  ��ӡtrace��ͬʱ���浽����log�ļ���
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
#define gps_tracker_trace \
{ \
	memset(log_buf, 0, sizeof(log_buf)); \
	_snprintf(log_buf, sizeof(log_buf), "%s %s:%d ", __TIME__, strrchr(__FILE__, '\\'), __LINE__); \
}trace

/*******************************************************
* ö�����Ͷ���
*********************************************************/

typedef enum
{
    EN_SRV_GT_IDS_BASE = 0,//SRV_GPS_TRACKER_BASE,
    EN_SRV_GT_IDS_TEXT,
    EN_SRV_GT_IDS_ICON,   
    
    EN_SRV_GT_IDS_END
} SRV_GT_IDS_EN;


typedef enum
{
    GT_MSG_TYPE_BASE = 0,
	GT_MSG_TYPE_UPLOAD_TIMER,
	GT_MSG_TYPE_SMS,
    
    GT_MSG_TYPE_END
} GT_MSG_TYPE_EN;

typedef enum
{
	EN_GT_PT_BASE = 0,
	EN_GT_PT_LOGIN,
	EN_GT_PT_GPS,
	EN_GT_PT_STATUS,	
	EN_GT_PT_HB,
	EN_GT_PT_ALARM,	
	EN_GT_PT_DEV_DATA,	
#ifdef __PROTOCOL_CONTROL__		
	EN_GT_PT_CONTROL,
#endif	
	
	EN_GT_PT_SRV_DATA = 0x20,

    EN_GT_PT_END
} GT_PROT_TYPE_EN;

typedef enum
{
	EN_GT_SMS_CMD_ADMIN = 0,
	EN_GT_SMS_CMD_PWD,
	EN_GT_SMS_CMD_USER,
	EN_GT_SMS_CMD_UP_INTV,
	EN_GT_SMS_CMD_HB_INTV,
	EN_GT_SMS_CMD_SMS_ALARM_INTV,
	EN_GT_SMS_CMD_TEMP_THR,
	EN_GT_SMS_CMD_VIBR_THR,
	EN_GT_SMS_CMD_SPEED_THR,
	EN_GT_SMS_CMD_LOC,
	EN_GT_SMS_CMD_CELL_INFO,
	EN_GT_SMS_CMD_LANG,
	EN_GT_SMS_CMD_LOG_LEVEL,
	EN_GT_SMS_CMD_TIME_ZONE,
	EN_GT_SMS_CMD_SHUTDOWN,
	EN_GT_SMS_CMD_RESTORE,

	EN_GT_SMS_CMD_ALARM_SWITCH,
	EN_GT_SMS_CMD_SMS_SWITCH,
	
	EN_GT_SMS_CMD_IGNORE_ALARM,
	
	EN_GT_SMS_CMD_APN,
	EN_GT_SMS_CMD_SERVER,

	EN_GT_SMS_CMD_DEFENCE,

	EN_GT_SMS_CMD_SMS_CENTER,
	EN_GT_SMS_CMD_VER,

	EN_GT_SMS_CMD_IMEI,
	EN_GT_SMS_CMD_DEV_ID,
	
	EN_GT_SMS_CMD_PARA,
	EN_GT_SMS_CMD_PWR_OIL_SWITCH,
	
	EN_GT_SMS_CMD_INVALID,
    EN_GT_SMS_CMD_END = EN_GT_SMS_CMD_INVALID
} GT_SMS_CMD_EN;
#ifdef __GPS_BAT_CONNECT__
typedef enum
{
	GPS_BAT_SUM_VOLTAGE = 0x09,			//�ܵ�ѹ
	GPS_BAT_CURRENT = 0x0a,				//��ŵ����
	GPS_BAT_DESIGN_CAPABILITY = 0x18,		//�������
	GPS_BAT_DESIGN_VOLITAGE = 0x19,		//��Ƶ�ѹ
	GPS_BAT_FCC = 0x10,						//��ذ��������
	GPS_BAT_REMAIN_CAPABILITY = 0x0f,		//ʣ������
	GPS_BAT_PERCENT_CAPABILITY = 0x0d,	//�����ٷֱ�
	GPS_BAT_CYCLE_COUNT = 0x17,			//ѭ������
	GPS_BAT_MANUFACTURENAME = 0x20,		//����������	//block
	GPS_BAT_TEMPERATURE = 0x08,			//����¶�
	GPS_BAT_STATUS = 0x16,					//��ذ�״̬
	GPS_BAT_INFO = 0x24,					//��ذ���Ϣ	//block
	GPS_BAT_BUNCH_NUM = 0x3c,				//��ش���
	GPS_BAT_BAR_CODE1 = 0xf0,				//�������1		//block
	GPS_BAT_BAR_CODE2 = 0xf1,				//�������2		//block
	GPS_BAT_BAR_CODE3 = 0xf2,				//�������3		//block
	
}GT_GPS_BAT_CMD;
#endif

typedef enum
{
    EN_GT_EC_BASE = KAL_TIMEOUT,
	EN_GT_EC_INVALID_SMS,//17
	EN_GT_EC_INVALID_CMD,
	EN_GT_EC_INVALID_PARA,
	EN_GT_EC_NVRAM_OPT_ERR,
	EN_GT_EC_SEND_ERR,
	EN_GT_EC_RCV_ERR,
	
    EN_GT_EC_END
} GT_ERR_CODE_EN;

typedef enum
{
	INFO,
	WARN,
	ERR
} GT_TRACE_LEVEL_EN;

typedef enum
{
    EN_GT_LANG_EN = 0,
    EN_GT_LANG_CN,

	EN_GT_LANG_INVALID,	
    EN_GT_LANG_END =EN_GT_LANG_INVALID
} GT_LANG_EN;


typedef enum
{
    EN_GT_NT_GPRMC = 0,
    EN_GT_NT_GPVTG,
    EN_GT_NT_GPGGA,
    EN_GT_NT_GPGSA,
    EN_GT_NT_GPGSV,
    EN_GT_NT_GPGLL
} GT_NMEA_TYPE_EN;

typedef enum
{
    EN_GT_GS_A = 1,	//0,		//zzt.20150906.modify 1
    EN_GT_GS_V,
    EN_GT_GS_INV
} GT_GPS_STATE_EN;

typedef enum
{
    EN_GT_EAST = 0,
    EN_GT_SOUTH,
    EN_GT_WEST,
    EN_GT_NORTH,
    EN_GT_INV
} GT_ORIENTATION_EN;

typedef enum
{
	EN_GT_GM_N = 0,
	EN_GT_GM_A,
    EN_GT_GM_D,
    EN_GT_GM_E,
    EN_GT_GM_INV
} GT_GPS_MODE_EN;

typedef enum
{
    EN_GT_GSMS_INIT = 0,
	EN_GT_GSMS_DNS,

    EN_GT_GSMS_CONN,

    EN_GT_GSMS_LOGIN,

	EN_GT_GSMS_WORKING

} GT_GSM_STAGE_EN;

typedef enum
{
	EN_GT_GPSS_INIT = 0,
	EN_GT_GPSS_SEARCH,
    EN_GT_GPSS_LOCATED
} GT_GPS_STAGE_EN;

typedef enum
{
	EN_GT_DT_ADMIN_NUM = 0x00,
	EN_GT_DT_PWD = 0x01,
	EN_GT_DT_USER = 0x02,
	
	EN_GT_DT_UP_INTV = 0x03,
	EN_GT_DT_HB_INTV = 0x04,
	EN_GT_DT_SMS_ALARM_INTV = 0x05,
	
	EN_GT_DT_TEMP_THR = 0x06,
	EN_GT_DT_VIBR_THR = 0x07,
	EN_GT_DT_SPEED_THR = 0x08,
	
	EN_GT_DT_LANG = 0x09,
	EN_GT_DT_TIME_ZONE = 0x0a,
	
	EN_GT_DT_ALARM_SWITCH = 0x0b,
	EN_GT_DT_SMS_ALARM_SWITCH = 0x0c,
	
	EN_GT_DT_LOC = 0x0d,
	EN_GT_DT_IGNORE_ALARM = 0x0e,
	EN_GT_DT_LOG_LEVEL = 0x0f,
	
	EN_GT_DT_SET_TIME = 0x10,
	EN_GT_DT_SHUTDOWN = 0x12,
	EN_GT_DT_RESTORE = 0x13,
	EN_GT_DT_SMS = 0x14,

	EN_GT_DT_DEFENCE = 0x16,
	EN_GT_DT_SERVER = 0x17,
	EN_GT_DT_APN = 0x18,
	EN_GT_DT_SMS_CENTER = 0x19,
	EN_GT_DT_VER = 0x1A,

	EN_GT_DT_PWR_OIL_SWITCH = 0x20,
	EN_GT_DT_IO = 0x21,
	
    EN_GT_CT_END
} GT_DATA_TYPE_EN;

typedef enum
{
    EN_GT_AT_PWR_LOW = 0,
	EN_GT_AT_PWR_OFF = 1,
	EN_GT_AT_VIBR = 2,
	EN_GT_AT_OIL_PWR = 3,
	EN_GT_AT_SPEED = 6,
    EN_GT_AT_END
} GT_ALARM_TYPE_EN;

typedef enum
{
    EN_GT_ADT_IP = 0,
	EN_GT_ADT_DOMAIN,
    EN_GT_ADT_END
} GT_ADDR_TYPE_EN;

typedef enum
{	
    EN_GT_LT_GPS = 0,
	EN_GT_LT_CELL ,
    EN_GT_LT_END
} GT_LOC_TYPE_EN;

typedef enum
{
	EN_GT_WS_GPS,
    EN_GT_WS_END
} GT_WORKING_STAGE_EN;

typedef enum
{
	EN_GT_RID_GPS,
    EN_GT_RID_END
} GT_REQUEST_ID_EN;

typedef enum
{
    EN_GT_SOCS_INIT = 0,
	EN_GT_SOCS_CONN,
	EN_GT_SOCS_READ,
	EN_GT_SOCS_WRITE,
	EN_GT_SOCS_CLOSE,
    EN_GT_SOCS_END
} GT_SOC_STATUS_EN;

typedef enum
{
    EN_GT_MS_MOTIONLESS = 0,
	EN_GT_MS_MOVING,
    EN_GT_MS_END
} GT_MOVING_STATE_EN;

typedef enum
{
    EN_GT_SWT_OFF = 0,
	EN_GT_SWT_ON,
    EN_GT_SWT_END
} GT_SWITCH_VALUE_EN;

typedef enum
{
	EN_GT_PI_ADMIN_NUM = 0x00,
	EN_GT_PI_PWD = 0x01,
	EN_GT_PI_USER = 0x02,
	EN_GT_PI_UP_INTV = 0x03,
	EN_GT_PI_HB_INTV = 0x04,
	EN_GT_PI_SMS_ALARM_INTV = 0x05,
	EN_GT_PI_TEMP_THR = 0x06,
	EN_GT_PI_VIBR_THR = 0x07,
	EN_GT_PI_SPEED_THR = 0x08,
	EN_GT_PI_LANG = 0x09,
	EN_GT_PI_TIME_ZONE = 0x0A,
	EN_GT_PI_PWR_LOW_SWITCH = 0x0B,
	EN_GT_PI_PWR_OFF_SWITCH = 0x0C,
	EN_GT_PI_VIBR_SWITCH = 0x0D,
	EN_GT_PI_OIL_PWR_SWITCH = 0x0E,
	EN_GT_PI_SOS_SWITCH = 0x0F,
	EN_GT_PI_TEMP_SWITCH = 0x10,
	EN_GT_PI_SPEED_SWITCH = 0x11,
	EN_GT_PI_FENCE_SWITCH = 0x12,
	EN_GT_PI_SMS_PWR_LOW_SWITCH = 0x13,
	EN_GT_PI_SMS_PWR_OFF_SWITCH = 0x14,
	EN_GT_PI_SMS_VIBR_SWITCH = 0x15,
	EN_GT_PI_SMS_OIL_PWR_SWITCH = 0x16,
	EN_GT_PI_SMS_SOS_SWITCH = 0x17,
	EN_GT_PI_SMS_TEMP_SWITCH = 0x18,
	EN_GT_PI_SMS_SPEED_SWITCH = 0x19,
	EN_GT_PI_SMS_FENCE_SWITCH = 0x1A,
	EN_GT_PI_SERVER = 0x1B,
	EN_GT_PI_APN = 0x1C,
	EN_GT_PI_AGPS_SERVER = 0x1D,
	EN_GT_PI_FENCE = 0x1E,
	EN_GT_PI_DEFENCE = 0x1F,
	EN_GT_PI_SMS_CENTER_NUM = 0x20,
	EN_GT_PI_VER = 0x21,
	EN_GT_PI_IMEI = 0x22,
	EN_GT_PI_DEV_ID = 0x23,
	EN_GT_PI_END
} GT_PARA_INDEX_EN;

#ifdef __GPS_MCU_CONNECT__
typedef enum{
	GT_MCU_SEND_BATTERY_STATUS = 0x01,
	GT_MCU_SEND_SEARCH = 0x02,
	GT_MCU_SEND_LOCK = 0x03,
	GT_MCU_SEND_ALARM = 0x04,
	GT_MCU_SEND_FAULT_STATUS = 0x05,
	GT_MCU_SEND_LOCK_STATUS = 0x06,
	GT_MCU_SEND_KM_STATUS = 0x07,
	GT_MCU_SEND_VIBRATE_DATA = 0x08,
	GT_MCU_SEND_END,
}GT_MCU_SEND_TYPE;
#define MCU_CATEGORY_BATTERY_STATUS 1
#define MCU_CATEGORY_SEARCH 1<<1
#define MCU_CATEGORY_LOCK 1<<2
#define MCU_CATEGORY_ALARM 1<<3
#define MCU_CATEGORY_FAULT_STATUS 1<<4
#define MCU_CATEGORY_LOCK_STATUS 1<<5
#define MCU_CATEGORY_KM_STATUS 1<<6
#define MCU_CATEGORY_VIBA 1<<7
typedef enum{
	GT_MCU_REPLY_ERROR = 0x00,
	GT_MCU_REPLY_OK = 0x01,
	GT_MCU_REPLY_BATTERY_STATUS = 0x02,
	GT_MCU_REPLY_FAULT_STATUS = 0x03,
	GT_MCU_REPLY_LOCK_STATUS = 0x04,
	GT_MCU_REPLY_HALL_STATUS = 0x05,
	GT_MCU_END,
}GT_MCU_REPLY_TYPE;
#endif
/*******************************************************
* �ṹ�嶨��
*********************************************************/
//soc �ṹ����
typedef struct
{
	S32 socketid;
	U8 	soc_status;
	sockaddr_struct sockaddr;	
} gps_tracker_soc_struct;

//gprs app info
typedef struct
{    
	kal_uint8 app_id;
	kal_uint32 acc_id;
} gps_tracker_app_struct;


typedef struct
{   
	kal_char num_s8[MAX_GT_PHONE_NUM_LEN];
	kal_uint16 num_u16[MAX_GT_PHONE_NUM_LEN];
} gps_tracker_sms_number_struct;

typedef struct
{   
	kal_char content_s8[MAX_GT_SMS_CONTENT_LEN];
	kal_uint16 content_u16[MAX_GT_SMS_CONTENT_LEN];
} gps_tracker_sms_content_struct;


//sms rsp info
typedef struct
{    
	kal_int32 rsp_code;
	
	kal_uint16 rsp_msg[MAX_GT_SMS_CONTENT_LEN];
} gps_tracker_sms_rsp_struct;

typedef struct
{
	U8 oil_pwr_state:1;		//���͵�״̬ 0 �͵��ͨ 1 ���Ͷϵ�
	U8 sos_state:1;			//sos ״̬ 0 ��sos  1 ��sos
} gps_tracker_state_struct;



typedef struct 
{	
	U8 pwr_low:1;//�͵籨������
	U8 pwr_off:1;//�ϵ�澯����
	U8 vibr:1;   //�𶯸澯����
	U8 oil_pwr:1;//���͵�澯����
	U8 sos:1;	 //sos�澯����
	U8 temp:1;   //�¶ȸ澯����
	U8 speed:1;  //���ٸ澯����
	U8 fence:1;
}gps_tracker_switch_struct;


typedef struct 
{	
	U32 last_lat; 	//�ϴο�����λ����γ��
	U32 last_long;	//�ϴο�����λ���ľ���
	U32 last_acc;	//�ϴο�����λ����
}gps_tracker_agps_loc_struct;

typedef struct 
{
	S8 local_index;
	gps_tracker_fence_struct fence;
}gps_tracker_calling_fence_struct;


//gps tracker ϵͳ���ò���
typedef  struct
{
	//ϵͳ����
	S8 	ver[MAX_GT_VER_STR_LEN];
	S8  dev_type;
	S8 	admin_num[MAX_GT_PHONE_NUM_LEN];				//����Ա�ֻ���
	U32 pwd;											//�豸����
	S8 	users[MAX_GT_USER_COUNT][MAX_GT_PHONE_NUM_LEN];

	//ʱ����
	kal_uint16 up_intv;
	kal_uint16 hb_intv;
	kal_uint16 sms_send_intv;
	
	//��������
	kal_int8 	temp_thr;
	kal_uint8 	vibr_thr;	
	kal_uint16 	speed_thr;

	kal_uint8   lang;
	S16  		time_zone; //����24ʱ��	
	
	gps_tracker_switch_struct 	alarm_switch;		//��������
	gps_tracker_switch_struct	sms_alarm_switch; 	//����֪ͨ����

	gps_tracker_server_struct	server;
	
	gps_tracker_apn_struct		apn;
	S8 							apn_prof_id;

	S8 							defence; 					//0 ���� 1���
	S8 sms_center_num[MAX_GT_PHONE_NUM_LEN];//�������ĺ���
	S8 imsi[MAX_GT_IMSI_LEN]; 
} gps_tracker_config_struct;

//�����������
typedef struct
{    
	//U32 	auth_id;//������֤��
	U32 	pwd;
	U32 	new_pwd;	
	U8		user_index;
	S8 		user[MAX_GT_PHONE_NUM_LEN];
	
	U16 	upload_intv;
	U16 	hb_intv;
	U16 	sms_send_intv;
	
	//��������
	S8 		temp_thr;
	U8 		vibr_thr;
	U16 	speed_thr;

	U8 		lang;
	S16 	time_zone;
	
	gps_tracker_data_switch_struct alarm_switch;
	gps_tracker_data_switch_struct sms_alarm_switch;

	U8 		loc_lang_type;
	U8		ignore_alarm;
	
	U8 		log_level;

	U8 		defence;
	
	gps_tracker_server_struct server;
	gps_tracker_apn_struct apn;

	S8 sms_center_num[MAX_GT_PHONE_NUM_LEN];//�������ĺ���

	S8 para_index;

	U8 pwr_oil_switch;
} gps_tracker_sms_cmd_para_struct;

//sms req info
typedef struct
{    
	gps_tracker_sms_content_struct content;
	gps_tracker_sms_number_struct number;
	
	GT_SMS_CMD_EN cmd_id;
	GT_LANG_EN cmd_lang_type;
	gps_tracker_sms_cmd_para_struct para;
	
	srv_sms_sim_enum sim_id;
} gps_tracker_sms_req_struct;

typedef struct 
{
	gps_tracker_msg_head_struct head;
	gps_tracker_msg_tail_struct tail;
	
	U8 packet_len;
	U8 packet_buf[64];
	
	union{
		gps_tracker_login_req_content_struct 	login;
		gps_tracker_gps_req_content_struct 		gps;
		gps_tracker_status_req_content_struct 	status;
		gps_tracker_alarm_req_content_struct 	alarm;

		gps_tracker_data_content_struct 		data;
	#ifdef __PROTOCOL_CONTROL__	
		gps_tracker_control_data_struct control;
	#endif	
	}content;	
}gps_tracker_dev_req_packet_struct;

typedef struct 
{
	kal_uint8 	is_updated;
	
	kal_uint16 	sn;
	kal_uint8 	prot_type;	
	kal_uint8 	is_responsed;

	gps_tracker_dev_req_packet_struct dev_req_packet;
}gps_tracker_dev_req_struct;

typedef struct 
{
	kal_uint8 datetime[6];
	kal_uint8 state;
	kal_uint32 latitude;
	kal_uint8 lat_ind;
	kal_uint32 longitude;
	kal_uint8 long_ind;
	kal_uint16 speed;
	kal_uint16 course;
	kal_uint16 magnetic_value;
	kal_uint8 magnetic_ind;
	kal_uint8 mode;//A ���� D ��� E���� N ������Ч

	kal_uint8 sat_uesed;
	kal_uint16 msl_altitude;
	kal_uint16 hdop;
}gps_tracker_gps_struct;

typedef struct 
{
	kal_uint8  sig_stren;
	kal_uint16 mcc;
	U8 mnc;
	kal_uint16 lac_sid;
	kal_uint16 cellid_nid;
	kal_uint16 bid;		
}gps_tracker_cell_info_struct;

typedef struct 
{
	U8 pwr_low_ind:1;
	U8 pwr_off_ind:1;
	U8 vibr_ind:1;
	U8 oil_pwr_ind:1;
	U8 sos_ind:1;
	U8 temp_ind:1;
	U8 speed_ind:1;
	U8 fence_ind:1;
	
	kal_uint8 pwr_level;
	kal_uint8 vibr_value;
	kal_int8 temp;
	kal_uint16 speed;
}gps_tracker_alarm_struct;


typedef struct 
{
	S8 x;
	S8 y;
	S8 z;
}gps_tracker_acce_struct;

typedef struct 
{
	U32 lat;
	U32 lng;
}gps_tracker_lat_lng_struct;

typedef struct 
{
	U8 number[MAX_GT_PHONE_NUM_LEN];
	U16 content[MAX_GT_SMS_CONTENT_LEN];
}sms_node_struct;

#ifdef __GPS_BAT_CONNECT__
typedef struct
{
	kal_uint16 total_voltage;
	kal_uint16 remain_capability;
	kal_uint16 percent_capability;
	kal_uint16 bunch_num;
	kal_uint16 cycle_count;
}battery_info_struct;
#endif

#ifdef __GPS_MCU_CONNECT__
/*
��������	��������		����ֵ
Ѱ��		0x01				NULL
����		0x02				0������ 1������
��������	0x03				0����1����
*/
typedef struct
{
	kal_uint8 type;		//0x01 Ѱ��,0x02 ������0x03 ��������
	kal_uint8 para[6];
}mcu_command_struct;
#endif

/*******************************************************
* ����ȫ�ֱ�������
*********************************************************/

/*******************************************************
* ���������ĺ���
*********************************************************/

#endif//__SRV_GPS_TRACKER_H__

