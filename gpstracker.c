#include <stdlib.h>
#include "app_datetime.h"
#include "APP2SOC_STRUCT.H"
#include "cbm_api.h"
#include "mmi_frm_events_gprot.h"
#include "nvram_user_defs.h"
#include "mmi_frm_nvram_gprot.h"
#include "ProtocolEvents.h"
#include "mmi_frm_queue_gprot.h"
#include "timerevents.h"
#include "NwInfoSrvGprot.h"
#include "charbatsrvgprot.h"

//math
#include "math.h"

//fs
#include "fs_gprot.h"

//apn
#include "DtcntSrvIprot.h"
#include "DataAccountStruct.h"
#include "custom_config_account.h"

//sms
#include "SmsDispatchSrv.h"
#include "Smslib_def.h"


//rtc
#include "../../../hal/peripheral/inc/rtc_sw_new.h"

/*i2c*/
#include "fsl_mma.h"

#include "gpstracker.h"
#ifdef __GPS_PACKAGE_BY_QUEUE__
#include "ztqueue.h"
#endif

/*srv_nw_info_get_service_availability*/
#include "NwInfoSrvGprot.h"
#ifdef __ACC_EINT_MODE__
#include "intrCtrl.h"
#endif

#ifdef __LED_INDICATE_STATE__
#include "dcl_pmu.h"
#include "dcl_pmu6260_sw.h"
#include "dcl_pmu_common_sw.h"
#endif
/**************************************************************
*�ļ��ڲ��ĺ궨��,���ĳЩ��ֻ�ڱ�c�ļ�ʹ�ã���ŵ�����
***************************************************************/
#define GT_VER "S1.1_H1.1" //ÿ����һ���汾�޸Ĵ˴� sv ����汾 hv Ӳ���汾

//����汾����
//#define OVERSEA

#define SUPER_PWD 314524776 //666888

void gps_tracker_restart(void);
/**************************************************************
*ȫ�ֱ�������
***************************************************************/
//ϵͳ����
struct sms_cmd{
	kal_uint32 cmd_id; 
	kal_uint16 cmd_en[MAX_GT_CMD_LEN]; 
	kal_uint16 cmd_zh[MAX_GT_CMD_LEN]; 
}gps_tracker_sms_cmd[] = 
{
	{EN_GT_SMS_CMD_ADMIN, L"admin", {0x7BA1, 0x7406, 0x5458, 0x0000}/*L"����Ա"*/},
	{EN_GT_SMS_CMD_PWD, L"pwd", {0x5BC6,0x7801,0x0000}/*L"����"*/},
	{EN_GT_SMS_CMD_USER, L"user", { 0x7528 ,0x6237,0x0000}/*L"�û�"*/},
	{EN_GT_SMS_CMD_UP_INTV, L"up-intv", {0x4E0A,0x62A5,0x95F4,0x9694,0x0000}/*L"�ϱ����"*/},
	{EN_GT_SMS_CMD_HB_INTV, L"hb-intv", {0x5FC3,0x8DF3,0x95F4,0x9694,0x0000}/*L"�������"*/},
	{EN_GT_SMS_CMD_SMS_ALARM_INTV, L"sms-intv", {0x77ED,0x4FE1,0x95F4,0x9694,0x0000}/*L"���ż��"*/},
	{EN_GT_SMS_CMD_TEMP_THR, L"temp-thr", {0x6E29,0x5EA6,0x95E8,0x9650,0x0000}/*L"�¶�����"*/},
	{EN_GT_SMS_CMD_VIBR_THR, L"vibr-thr", {0x9707,0x52A8,0x95E8,0x9650,0x0000}/*L"������"*/},
	{EN_GT_SMS_CMD_SPEED_THR, L"speed-thr", {0x8D85 ,0x901F ,0x95E8 ,0x9650,0x0000}/*L"��������"*/},
	{EN_GT_SMS_CMD_LOC, L"loc", {0x4F4D,0x7F6E,0x0000}/*L"λ��"*/},
	{EN_GT_SMS_CMD_CELL_INFO, L"cell", {0x57FA,0x7AD9,0x0000}/*L"��վ"*/},
	{EN_GT_SMS_CMD_LANG, L"lang", {0x8BED,0x8A00,0x0000}/*L"����"*/},
	{EN_GT_SMS_CMD_LOG_LEVEL, L"log-level", {0x65E5 ,0x5FD7 ,0x7EA7 ,0x522B,0x0000}/*L"��־����"*/},
	{EN_GT_SMS_CMD_TIME_ZONE, L"time-zone", {0x65F6 ,0x533A,0x0000}/*L"ʱ��"*/},
	{EN_GT_SMS_CMD_SHUTDOWN, L"shutdown", {0x5173,0x95ED,0x0000}/*L"�ر�"*/},
	{EN_GT_SMS_CMD_RESTORE, L"restore", { 0x6062 ,0x590D ,0x51FA ,0x5382,0x0000}/*L"�ָ�����"*/},
	{EN_GT_SMS_CMD_ALARM_SWITCH, L"alarm-switch", {0x544A,0x8B66,0x5F00,0x5173,0x0000}/*L"�澯����"*/},
	{EN_GT_SMS_CMD_SMS_SWITCH, L"sms-switch", {0x77ED,0x4FE1,0x5F00,0x5173,0x0000}/*L"���ſ���"*/},	
	{EN_GT_SMS_CMD_IGNORE_ALARM, L"ignore-alarm", {0x5FFD,0x7565,0x544A,0x8B66,0x0000}/*L"���Ը澯"*/},
	{EN_GT_SMS_CMD_APN, L"apn", {0x63A5 ,0x5165 ,0x70B9,0x0000}/*L"�����"*/},
	{EN_GT_SMS_CMD_SERVER, L"server", {0x670D ,0x52A1 ,0x5668,0x0000}/*L"������"*/},
	{EN_GT_SMS_CMD_DEFENCE, L"defence", {0x9632,0x62A4,0x0000}/*L"����"*/},
	{EN_GT_SMS_CMD_SMS_CENTER, L"sms-center", {0x77ED, 0x4FE1, 0x4E2D, 0x5FC3,0x0000}/*L"��������"*/},
	{EN_GT_SMS_CMD_VER, L"ver", {0x7248, 0x672C,0x0000}/*L"�汾"*/},
	{EN_GT_SMS_CMD_IMEI, L"imei", L"imei"/*L"imei"*/},
	{EN_GT_SMS_CMD_DEV_ID, L"dev-id", {0x8BBE, 0x5907, 0x7801,0x0000}/*L"�豸��"*/},
	{EN_GT_SMS_CMD_PARA, L"para", {0x53C2, 0x6570,0x0000}/*L"����"*/},
	{EN_GT_SMS_CMD_PWR_OIL_SWITCH, L"pwr-oil", { 0x6CB9, 0x7535, 0x0000}}//�͵� <����> | ���� :0�͵�Ͽ� 1�͵��ͨ
};

//ϵͳ����
struct err_desc{
	kal_uint32 err_code; 
	kal_uint16 desc_en[MAX_GT_ERR_DESC_LEN]; 
	kal_uint16 desc_zh[MAX_GT_ERR_DESC_LEN]; 
}gps_tracker_err_desc[] = 
{
	{EN_GT_EC_INVALID_SMS, L"invalid sms", {0x65E0, 0x6548, 0x77ED, 0x4FE1, 0x0000}/*L"��Ч����"*/},
	{EN_GT_EC_INVALID_CMD, L"invalid cmd", {0x26080, 0x25928, 0x25351, 0x20196, 0x0000}/*L"��Чָ��"*/},
	{EN_GT_EC_INVALID_PARA, L"invalid parameter", {0x65E0,0x6548,0x53C2,0x6570,0x0000}/*L"��Ч����"*/},
	{EN_GT_EC_NVRAM_OPT_ERR, L"nvram opt err", {0x26412, 0x22320, 0x25968, 0x25454, 0x25805, 0x20316, 0x22833, 0x36133,0x0000}/*L"�������ݲ���ʧ��"*/},
	{EN_GT_EC_SEND_ERR, L"send err", { 0x53D1 ,0x9001 ,0x9519 ,0x8BEF,0x0000}/*L"���ʹ���"*/},
	{EN_GT_EC_RCV_ERR, L"receive err", { 0x63A5 ,0x6536 ,0x9519 ,0x8BEF,0x0000}/*L"���մ���"*/},
};

static U8 NMEA_KEY[][6] = {
	"GPRMC",
	"GPVTG",
	"GPGGA",
	"GPGSA",
	"GPGSV",
	"GPGLL"
};

static U16 CRC16_TABLE[] = {
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};	
//ϵͳ��cb���кţ���0 ��ʼ ����
kal_uint32 gps_tracker_sn = 0;

//sn ��Ӧmutex,��֤sn������һ���Ե���
kal_mutexid gps_tracker_sn_mutex;

//ϵͳ��־����
#ifdef _WIN32
U8 gps_tracker_log_level = INFO;
#else
U8 gps_tracker_log_level = INFO;
#endif

U8 gps_tracker_working_stage = EN_GT_WS_GPS;

/***********************************************************
���ݽṹ��: ��������ʵ������
cb�ṹ��: ���շ��ͽӿڶ�������ݽӿ�
************************************************************/
gps_tracker_app_struct		gps_tracker_app = {0};
gps_tracker_soc_struct		gps_tracker_soc = {0};

gps_tracker_sms_req_struct	gps_tracker_sms_req = {0};

gps_tracker_gps_struct 	gps_tracker_gps = {0};
gps_tracker_gps_struct  gps_tracker_gps_fresh = {0};

#ifdef __GPS_BEST_HDOP__	//zzt.20150824
gps_tracker_gps_struct gps_tracker_gps_array[10]={0};	
U8 gps_index = 0;
#endif

gps_tracker_state_struct 		gps_tracker_state;

gps_tracker_config_struct 		gps_tracker_config = {0};
//��վ��Ϣ
gps_tracker_cell_info_struct 	gps_tracker_cell = {0};

gps_tracker_alarm_struct 		gps_tracker_alarm = {0};
/************************************************************/
gps_tracker_dev_req_struct 	gps_tracker_login_cb = {0};
//gps ��Ϣ
gps_tracker_dev_req_struct 	gps_tracker_gps_cb = {0};
//�豸״̬
gps_tracker_dev_req_struct 	gps_tracker_status_cb = {0};
//�豸���ò�����������nvram��ȡ��ȫ�����ø���
gps_tracker_dev_req_struct 	gps_tracker_dev_data_cb = {0};

gps_tracker_dev_req_struct 	gps_tracker_hb_cb = {0};
//�澯
gps_tracker_dev_req_struct 	gps_tracker_alarm_cb = {0};

//�������ͨѶ��Ϣ
#ifdef __PROTOCOL_CONTROL__
gps_tracker_dev_req_struct gps_tracker_control_cb = {0};
gps_tracker_control_data_struct gps_tracker_recv_control_data;
U8 gps_tracker_control_recv_flag = 0;
#endif

#ifdef __LED_INDICATE_STATE__
U8 gps_tracker_gps_led = 1;	//0 off, 1 on
U8 gps_tracker_gsm_led = 0;	//0 off, 1 on
#endif
#ifdef __GPS_PACKAGE_BY_QUEUE__
LinkQueue gps_tracker_upload_queue;
#endif
kal_uint8 gps_tracker_login_times = 0;
kal_uint8 gps_tracker_conn_times = 0;

//��gps����1����û������������
U8 gps_tracker_gps_failed_times = 0;

//dns ���Դ���
kal_uint8 gps_tracker_dns_times = 0;

//����ʧ�ܴ���
kal_uint8 gps_tracker_send_failed_times = 0;
//����ʧ�ܴ���
kal_uint8 gps_tracker_hb_failed_times = 0;

//�豸id
kal_uint8 gps_tracker_dev_id[MAX_GT_DEV_ID_LEN] = {0};
kal_uint8 gps_tracker_imei[MAX_GT_IMEI_LEN] = {0};

#ifdef __GPS_BAT_CONNECT__
battery_info_struct battery_info;
#endif

//log ��־ ���ȫ�ֱ���
kal_char log_buf[MAX_GT_LOG_BUF_LEN] = {0};

//���������ļ�
FS_HANDLE gps_tracker_local_read_file_hdl;
FS_HANDLE gps_tracker_local_write_file_hdl;

//uart ���
static kal_uint8	uart_cur_handle;
static module_type	uart_orig_owner;

//ϵͳʱ���Ƿ�ok
kal_bool is_gps_datetime_updated = KAL_FALSE;
kal_bool is_datetime_updated = KAL_FALSE;

kal_uint8 gps_tracker_gps_state;
kal_uint8 gps_tracker_gsm_state;

kal_bool is_need_dns = KAL_TRUE;

kal_uint8 gps_tracker_gps_parse_failed_times = 0;

gps_tracker_acce_struct gps_tracker_acce = {0};

U8 gps_tracker_moving_state = EN_GT_MS_MOVING;

U8 gps_tracker_motionless_counts = 0;
U8 gps_tracker_moving_counts = 0;

//��ǰ��ֵ
U8 gps_tracker_shake_value = 0;
U8 gps_tracker_shake_value_fresh = 0;
U8 gps_tracker_shake_value_array[10];	//zzt.20150801
#ifdef __GPS_MCU_CONNECT__
U8 shake_value_mcu_index = 0;
U8 gps_tracker_shake_value_array_mcu[3];
#endif
#ifdef __ACC_EINT_MODE__
U8 gps_tracker_shake_delay_flag = 1;	//�жϵ���ʱ��־
#endif

//������⵽���𶯴���
U8 gps_tracker_vibr_alarm_counts = 0;

#define SHAKE_BUF_LEN 2
U8 shake_buf[SHAKE_BUF_LEN] = {0};
U8 shake_buf_index = 0;

//////////////////////////////

gps_tracker_lat_lng_struct last_loc = {0};

rr_em_ca_list_info_struct em_ca_list = {0};
U8 em_ca_index =0;
#ifdef __GPS_MCU_CONNECT__
U8 gps_mcu_i2c_busy;
#endif

/*********************���Ŷ��нṹ����************************/
//����һ��ѭ����Ϣ���� ��ͨ����������ʵ�֣�ģ��FIFO
#define MAX_GT_SMS_SEND_ARRAY_LEN 50
sms_node_struct sms_send_array[MAX_GT_SMS_SEND_ARRAY_LEN];

//��Ϣ������
kal_mutexid sms_send_array_mutex;
//FIFO ��tail ����node���� head ��ȡnode
U8 fence_send_array_len = 0;
U8 fence_send_array_head_index = 0;
U8 fence_send_array_tail_index = 0;
/*********************���ջ�����нṹ����************************/
//����һ��ѭ����Ϣ���� ��ͨ����������ʵ�֣�ģ��FIFO
#define MAX_GT_RCV_MSG_ARRAY_LEN 20
#define MAX_GT_RCV_MSG_LEN 250
U8 rcv_msg_array[MAX_GT_RCV_MSG_ARRAY_LEN][MAX_GT_RCV_MSG_LEN];

//��Ϣ������
kal_mutexid rcv_msg_array_mutex;
//FIFO ��tail ����node���� head ��ȡnode
U8 rcv_msg_array_len = 0;
U8 rcv_msg_array_head_index = 0;
U8 rcv_msg_array_tail_index = 0;

U16 gprs_account_name[] = L"IntelIOT";
/**************************************************************
*�ڲ���������
***************************************************************/
void gps_tracker_login_timer_proc(void);
S32 gps_tracker_get_host_by_name(void);
void gps_tracker_conn_notify(void *msg_ptr);
S32 gps_tracker_establish_conn(void);
S32 gps_tracker_format_gsm_cell_req(kal_uint32 mcc, kal_uint32 mnc, kal_uint32 lac, kal_uint32 cellid, kal_char* gsm_cell_req);
void gps_tracker_files_init(void);
void gps_tracker_dns_check_timer_proc(void);
void gps_tracker_conn_timer_proc(void);
U8 gps_tracker_format_cb_to_buffer(gps_tracker_dev_req_struct* cb, U8* buffer, kal_uint8 len);
void gps_tracker_em_status_ind_hdlr(void *info);
U8 gps_tracker_acce_get_vibr(void);
U8 gps_tracker_acce_get_tilt(void);
void gps_tracker_send_timeout_proc(void);
S32 gps_tracker_send_req(S8* buffer, U16 len);
kal_bool gps_tracker_validate_user(void);
S32 gps_tracker_nvram_restore(void);
S32 gps_tracker_server_conn(void);
U16 gps_tracker_uart_write(U8 *buf, U16 max_len, UART_PORT port, module_type owner);
S32 gps_tracker_sms_delay_rst_proc();
kal_int32 gps_tracker_sms_server(void);
S32 gps_tracker_send_sms_to_all_users(U16* content);
S32 gps_tracker_send_sms(U16* num, U16* content);
kal_int32 gps_tracker_rcv_proc( U8* buf );

#ifdef __GPS_CONTROL_CONNECT__ 
void gps_send_data_to_control(U8* data,U8 data_len,U8 addr);
kal_uint8 gps_get_data_from_control(kal_uint8* recvBuf,kal_uint8 bufSize,kal_uint8* address);
#endif
#ifdef __GPS_BAT_CONNECT__
void gps_get_data_from_battery(void);
#endif
#ifdef __GPS_MCU_CONNECT__
void gps_tracker_send_mcu_data(void);
#endif
/**************************************************************
*�ⲿ��������
***************************************************************/
extern void RstStartRestore(void);
extern module_type U_GetOwnerID(UART_PORT port);
extern void U_ClrRxBuffer(UART_PORT port, module_type ownerid);
extern void U_ClrTxBuffer(UART_PORT port, module_type ownerid);
extern U16 U_GetTxRoomLeft(UART_PORT port);
extern kal_int32 app_ucs2_wcsicmp(const kal_wchar *str_src, const kal_wchar *str_dst);
extern kal_int32 app_ucs2_wtoi(const kal_wchar *nptr);
extern kal_uint8 L1SM_GetHandle(void);
extern void L1SM_SleepDisable(kal_uint8 handle);
extern void GPIO_ModeSetup(kal_uint16 pin, kal_uint16 conf_dada);
extern void GPIO_InitIO(char direction, char port);
extern void GPIO_WriteIO(char data, char port);
extern char GPIO_ReadIO(char port);
extern void ShutdownSystemOperation(void);
#ifdef __MMI_DUAL_SIM_MASTER__
extern U8* mmi_bootup_get_imei_value(sim_interface_enum sim_interface);
#else
extern U8* mmi_bootup_get_imei_value(void);
#endif
extern kal_uint16 app_asc_str_to_ucs2_str(kal_int8 *pOutBuffer, kal_int8 *pInBuffer);
extern void mmi_wc_update_datetime(void);
extern void mmi_dt_set_rtc_dt(MYTIME *t);
extern U16 *srv_phb_owner_number_get_number(U8 sim_interface, U16 index);
extern kal_int32 gps_tracker_send_rsp(S8* buffer, U16 len);
extern U8* mmi_bootup_get_imsi_value(void);
//void mmi_dtcnt_update_disp_list(U32 acct_id, U8 bearer_type, mmi_dtcnt_acct_type_enum account_type, U8 conn_type, U8 sim_info, U8 read_only);
//MMI_BOOL mmi_dtcnt_get_gprs_profile(U32 profile_id, mmi_dtcnt_gprs_acct_node_struct *g_data_account_gprs_profile);
#ifdef __GPS_MCU_CONNECT__
extern VUINT8 MCU_IICReadData(void);
extern void MCU_IICWriteData(VUINT8 data);
#endif
/**************************************************************
*�ⲿȫ�ֱ���
***************************************************************/
extern srv_dtcnt_prof_gprs_struct g_data_account_gprs_profile_srv;
extern mmi_dtcnt_node_display_struct g_data_account_display_cntx;
#ifdef __ACC_EINT_MODE__
extern const unsigned char ACC_EINT_NO;
#endif

/**************************************************************
*��������
***************************************************************/

U8 abs_S8(S8 v)
{
	if((U8)v<128) 
	{
		return v;
	}
	else
	{
		return 256-(U8)v;
	}
}

/*****************************************************************************
 * FUNCTION
 *	gps_tracker_trace
 * DESCRIPTION
 *	��ӡtrace��ͬʱ���浽����log�ļ���
 * PARAMETERS
 *	void  
 *	
 * RETURNS
 *	void
 *****************************************************************************/

#if 0
#define SLC_LOG_BUFF_SIZE 1024

void gps_trace(kal_char* fmt, ...)
{
		va_list lstArg;
		kal_uint16 nLen=0;
		applib_time_struct currTime;
		char *p=get_ctrl_buffer(SLC_LOG_BUFF_SIZE);
		if(!p)
		{
			return;
		}
		memset(p, 0x00, SLC_LOG_BUFF_SIZE);
		memset(&currTime, 0x00, sizeof(currTime));
		applib_dt_get_rtc_time(&currTime);
		sprintf(p, "[%04d/%02d/%02d, %02d:%02d.%02d]", currTime.nYear,currTime.nMonth, currTime.nDay, currTime.nHour, currTime.nMin, currTime.nSec);
		nLen=strlen(p);
	
		va_start(lstArg, fmt);
		nLen = vsnprintf(p+nLen, SLC_LOG_BUFF_SIZE-2-nLen, fmt, lstArg);
		va_end(lstArg);
		strcat(p, "\r\n");
		nLen=strlen(p);
		
#if defined(WIN32)
		kal_printf("TraceEx: %s\n", p);
#else
		U_PutUARTBytes(uart_port1, (kal_uint8 *)p, nLen);
#endif
		free_ctrl_buffer(p);

}
#else
void trace(kal_uint32 level, kal_uint32 sth, kal_char* fmt, ...) 
{
#if 1
	va_list args;  
	char buf[100] = {0};//�˴�����ĳ������С��log_buf�ĳ���-ǰ׺�ĳ���
	kal_uint32 size = 0; 
#ifdef __TRACE_WRITE_FS__
	int ret;
	kal_uint32 writen;
	memset(log_buf, 0, MAX_GT_LOG_BUF_LEN);
#endif
	
	if(fmt == NULL)
	{
		return;
	}
	if(strlen(fmt) == 0)
		return;
	
	//�ж���־����
	if(level < gps_tracker_log_level)
		return;
	

	//д����־����
	if(INFO == level)
	{
		strncat(log_buf, "[INFO][ITracker] ", MAX_GT_LOG_BUF_LEN);
	}
	else if(WARN == level)
	{
		strncat(log_buf, "[WARN][ITracker] ", MAX_GT_LOG_BUF_LEN);
	}
	else if(ERR == level)
	{
		strncat(log_buf, "[ERR][ITracker] ", MAX_GT_LOG_BUF_LEN);
	}

	//ȡ��log�����ݲ��� 
	va_start (args, fmt);
	vsnprintf(buf, 99, fmt, args); 
	va_end (args);	

	//��ǰ׺���ֺ�log���ݺϲ�һ��
	strncat(log_buf, buf, 99);
#ifdef __TRACE_WRITE_FS__
	ret = FS_Open(L"c:\\gps_log.txt", FS_CREATE|FS_READ_WRITE);
	if(ret >0)
	{
		FS_GetFileSize(ret, &size);
		if(size > 50*1024)
		{
			FS_Seek(ret, 0, FS_FILE_BEGIN);
			FS_Write(ret, log_buf, MAX_GT_LOG_BUF_LEN,&writen);
		}
		else
		{
			FS_Seek(ret, 0, FS_FILE_END);
			FS_Write(ret, log_buf, MAX_GT_LOG_BUF_LEN,&writen);
		}
		FS_Close(ret);
	}
#endif
	
	strncat(log_buf, "\r\n", 2);
	
	//ͬʱ�����catcher log ��
	kal_prompt_trace(MOD_WPS, log_buf);	
//	U_PutUARTBytes(uart_port1,log_buf,sth);
#endif
}
#endif
/*****************************************************************************
 * FUNCTION
 *	remove_space_sharp
 * DESCRIPTION
 *	ȥ������������ ͷ����β���� �ո� #��
 * PARAMETERS
 *	content ������Ϣ��
 *	
 * RETURNS
 *	void
 *****************************************************************************/
kal_uint32 remove_space_sharp(kal_uint16* content)
{
	kal_uint16 new_content[MAX_GT_SMS_CONTENT_LEN] = {0};
	kal_uint16* head = NULL;
	kal_uint16* tail = NULL;
	kal_uint32 len = 0;
	kal_uint32 i = 0;

	
	if(content == NULL)
	{
		return KAL_SUCCESS;
	}

	//ȥ��ͷ���Ŀո�
	len = wcslen(content);
	head = content; 
	
	for(i = 0; i < len; i++)
	{
		if(content[i] == 0x0020 || content[i] ==0x000A || content[i] ==0x0023)
		{
			head++;
		}
		else
		{
			break;
		}
	}
	

	//ȥ��ͷ���Ŀո�
	tail = content+len;
	len = wcslen(head); 
	
	for(i = len-1; i >= 0; i--)
	{
		if(head[i] == 0x0020 || head[i] == 0x000A || head[i] ==0x0023)
		{
			tail--;
		}
		else
		{
			break;
		}
	}

	//�����õĶ������忽��������֤�ڴ�ɾ�
	memcpy(new_content, head, (tail-head)*sizeof(kal_uint16));
	memset(content, 0, wcslen(content) * sizeof(U16));
	memcpy(content, new_content, (tail-head)*sizeof(kal_uint16));
	
	return KAL_SUCCESS;
}

/*****************************************************************************
 * FUNCTION
 *	max_match_r_cmp
 * DESCRIPTION
 *	����ȽϺ���,���ҵ������ƥ����бȽ�
 * PARAMETERS
 *	void
 *	
 * RETURNS
 *	���ƥ����ͬ������ 0
 *****************************************************************************/
kal_uint32 max_match_r_cmp(char* num1, char*num2)
{
	U32 len;

	if(num1 == NULL || num2 == NULL)
		return 1;
	
	if(strlen(num1) == 0 && strlen(num2) ==0)
	{
		return 0;
	}
	else if(strlen(num1) == 0 || strlen(num2) == 0)
	{
		return 1;
	}
	else
	{
		len = strlen(num1) < strlen(num2)?strlen(num1):strlen(num2);
		
		return strcmp(num1+(strlen(num1) -len), num2+(strlen(num2) -len));
	}
}

/*****************************************************************************
 * FUNCTION
 *	js_hash
 * DESCRIPTION
 *	JSHash �㷨
 * PARAMETERS
 *	void
 * RETURNS
 *	void
 *****************************************************************************/
S32 js_hash(S8* input, U8 len)
{
	S32 hash = 1315423911;
	U8 i = 0;


	if(input == NULL)
		return 0;
	
	for(i = 0; i < len; i++)
	{
		hash ^= ((hash << 5) + input[i] + (hash >> 2));
	}
	
	if(hash < 0)
	{
		hash+=pow(2,31);
	}
	
	return (hash);
}

/*****************************************************************************
 * FUNCTION
 *	hex_str_2_bytes
 * DESCRIPTION
 *	��16���ƵĴ�ת�����ֽ�����
 * PARAMETERS
 *	void
 * RETURNS
 *	void
 *****************************************************************************/
kal_uint32 hex_str_2_bytes(kal_char* str, kal_uint32 str_len, kal_uint8* bytes, kal_uint32 bytes_len)
{
	kal_uint32 i;
	kal_uint8 hex = 0;
	kal_uint8 offset = 0;

	if (str == NULL || bytes == NULL)
		return KAL_ERROR;
	
	if((str_len +1) /2 > bytes_len)
	{
		gps_tracker_trace(ERR, MOD_MMI,
			"The input hex str len is more than out bytes len.convert failed,do nothing.");
		//gps_trace("%s(%d)",__func__,__LINE__);
		return KAL_ERROR;
	}
	
	if (str_len%2 != 0)
	{
		offset = 1;
	}

	for(i =0; i < str_len; i++)
	{
		//�ж�hex �ַ�������Ч��
		if(str[i] >= '0' && str[i] <= '9')
		{
			hex = str[i] - '0';
		}
		else if( str[i] >= 'a' && str[i] <= 'z')
		{
			hex = str[i] - 'a';
		}
		else if(str[i] >= 'A' && str[i] <= 'Z')
		{
			hex = str[i] - 'A';
		}
		else
		{
			gps_tracker_trace(ERR, MOD_MMI,
				"Invalid hex string.");
			//gps_trace("%s(%d)",__func__,__LINE__);
			return KAL_ERROR;
		}

		bytes[(i + offset)/2] += hex << 4*((i + offset + 1)%2);
	}

	return KAL_SUCCESS;
}
/*****************************************************************************
 * FUNCTION
 *	GetCRC
 * DESCRIPTION
 *	��ȡ�ֽ������ CRC
 * PARAMETERS
 *	void
 * RETURNS
 *	void
 *****************************************************************************/
U16 get_crc16(U8* bytes, U16 len)
{
	U16 value = 0xffff;
	int i;

	if (bytes == NULL)
		return 0;

	
	for (i = 0; i < len; i++) {
		// 1.value ����8λ(�൱�ڳ���256)
		// 2.value����������ݽ���������������0xFF����������
		//	  �õ�һ������index��Ȼ�����CRC16_TABLE����Ӧ����������
		// 1��2�õ��������ٽ���������㡣
		value = (value >> 8) ^ CRC16_TABLE[(value ^ bytes[i]) & 0xff];			
	}
	
	// ȡ��
	return ~value;
}

/*****************************************************************************
 * FUNCTION
 *  wstr_get_wstr
 * DESCRIPTION
 *  ��wstr �л�ȡ��һ���ո�ǰ���Ӵ�
 * PARAMETERS
 *  
 *  
 * RETURNS
 *  ERR CODE
 *****************************************************************************/
kal_int32 wstr_get_wstr(kal_uint16** str, kal_uint16* sub_str)
{
	kal_uint16* tmp1 = NULL;
	kal_uint16* tmp2 = NULL;
	kal_uint32 len = 0;

	
	if(str == NULL || sub_str == NULL)
		return EN_GT_EC_INVALID_SMS;
	

	//1. ��ȡ��������
	tmp1 = *str;	

	if(tmp1 == NULL)
		return KAL_ERROR;
	
	tmp2 = wcschr(tmp1, L' ');	
	
	if(tmp2 == NULL)
	{
		//û���ҵ��ո�˵������û��������
		kal_wstrcpy(sub_str, tmp1);

		tmp1 = NULL;
	}	
	else if( tmp2 >= *str + wcslen(*str) )
	{
		//˵���ո��Ѿ������������������ˣ�����
		return EN_GT_EC_INVALID_SMS;
	}
	else
	{
		//����ĵ�ַ��� ��������͵ĸ����Ҳ����U16�ĸ���
		len = tmp2 - tmp1;
		kal_wstrncpy(sub_str, tmp1, len);	
	
		tmp1= tmp2+1;
	}
	if(sub_str[0] == L'*')
	{
		sub_str[0] = 0;
	}
	
	//���˵�tmp1����Ŀո�
	while(tmp1 !=NULL && *tmp1 == L' ')
	{
		tmp1++;
	}

	*str = tmp1;	
	
	return KAL_SUCCESS;
}

kal_int32 wstr_get_str(kal_uint16** str, kal_char* sub_str)
{
	U16 tmp_str[MAX_GT_SMS_CONTENT_LEN];
	kal_int32 ret;

	ret = wstr_get_wstr(str, tmp_str);
	if(KAL_SUCCESS != ret)
	{
		return ret;
	}

	if(tmp_str[0] == 0)
	{
		sub_str[0] = 0;
		return KAL_SUCCESS;
	}

	if(wcslen(tmp_str) != wcstombs(sub_str, tmp_str, wcslen(tmp_str)))
	{
		return KAL_ERROR;
	}
	return KAL_SUCCESS;
}

kal_int32 wstr_get_uint(kal_uint16** str, kal_uint32* uint_value)
{
	U16 tmp_str[MAX_GT_SMS_CONTENT_LEN];
	U16 *stop_pt;
	kal_int32 ret;

	ret = wstr_get_wstr(str, tmp_str);
	if(KAL_SUCCESS != ret)
	{
		return ret;
	}

	if(tmp_str[0] == 0)
	{
		*uint_value = 0;
		return KAL_SUCCESS;
	}

	*uint_value = wcstoul(tmp_str, &stop_pt, 10);	

	return KAL_SUCCESS;
}

kal_int32 wstr_get_int(kal_uint16** str, kal_int32* int_value)
{
	U16 tmp_str[MAX_GT_SMS_CONTENT_LEN];
	U16 *stop_pt;
	kal_int32 ret;

	ret = wstr_get_wstr(str, tmp_str);
	if(KAL_SUCCESS != ret)
	{
		return ret;
	}

	if(tmp_str[0] == 0)
	{
		*int_value = 0;
		return KAL_SUCCESS;
	}
	
	*int_value = wcstol(tmp_str, &stop_pt, 10);	

	return KAL_SUCCESS;
}


kal_int32 wstr_get_float(kal_uint16** str, double* float_value)
{
	char tmp_str[MAX_GT_SMS_CONTENT_LEN];
	kal_int32 ret;

	ret = wstr_get_str(str, tmp_str);
	if(KAL_SUCCESS != ret)
	{
		return ret;
	}

	if(tmp_str[0] == 0)
	{
		*float_value = 0;
		return KAL_SUCCESS;
	}
	
	*float_value = atof(tmp_str);	
	
	return KAL_SUCCESS;
}

S32 ip_str_2_array(S8* str_ip, U8* array_ip)
{
	S8* tmp1 = NULL;
	S8* tmp2 = NULL;
	S8 num[4] = {0};
	U8 len = 0;
	S8 i;


	if(str_ip == NULL || array_ip == NULL)
		return KAL_ERROR;
	
	tmp1 = str_ip;
	
	for(i = 0; i < MAX_GT_IP_ADDR_LEN; i++)
	{
		memset(num, 0, 4);
		
		tmp2 = strchr(tmp1, '.');	
		
		if(tmp2 == NULL)
		{
			//û���ҵ��ո�˵������û��������
			strcpy(num, tmp1);

			tmp1 = NULL;
		}	
		else if( tmp2 >= str_ip + strlen(str_ip) )
		{
			//˵���ո��Ѿ������������������ˣ�����
			return KAL_ERROR;
		}
		else
		{
			len = tmp2 - tmp1;
			if(len > 3)
			{
				gps_tracker_trace(ERR, MOD_MMI,	"Invalid IP format");
				return KAL_ERROR;
			}
			strncpy(num, tmp1, len);	
		
			tmp1= tmp2+1;
		}
		
		//���ַ���ת��������
		array_ip[i] = atoi(num);
		if(array_ip[i] > 255)
		{
			gps_tracker_trace(ERR, MOD_MMI,	"Invalid IP");
			return KAL_ERROR;
		}
		if(i == 0 && array_ip[i] == 0)
		{
			gps_tracker_trace(ERR, MOD_MMI,	"Invalid IP format");
			return KAL_ERROR;
		}
		
		if(NULL == tmp1)
		{
			break;
		}
	}
	return KAL_SUCCESS;
}

/**************************************************************
*ҵ����
***************************************************************/
/**************************************************************
*Χ����Ϣ���к���
***************************************************************/
/*****************************************************************************
 * FUNCTION
 *	fence_alarm_array_put_node
 * DESCRIPTION
 *	�������в���node ��������ֶ���������ֹͣ����
 * PARAMETERS
 *	void
 *	
 * RETURNS
 *	void
 *****************************************************************************/
U32 sms_send_array_put_node(sms_node_struct* node)
{

	if(node == NULL)
		return KAL_ERROR;

	if(fence_send_array_len < MAX_GT_SMS_SEND_ARRAY_LEN)
	{
		kal_take_mutex(sms_send_array_mutex);
		
		memcpy(&sms_send_array[fence_send_array_tail_index], node ,sizeof(sms_node_struct));		
		fence_send_array_tail_index = (fence_send_array_tail_index+1)%MAX_GT_SMS_SEND_ARRAY_LEN;
		fence_send_array_len++;
		
		kal_give_mutex(sms_send_array_mutex);
	}
	else
	{
	#ifdef __SMS_TRACE__
		gps_tracker_trace(ERR, MOD_MMI,"the send array is full");
	#endif	
		return KAL_ERROR;
	}
	return KAL_SUCCESS;
}
U32 sms_send_array_get_node(sms_node_struct* node)
{
	if(node == NULL)
		return KAL_ERROR;
	
	if(fence_send_array_len > 0)
	{
		kal_take_mutex(sms_send_array_mutex);
		
		memcpy(node , &sms_send_array[fence_send_array_head_index],sizeof(sms_node_struct));		
		fence_send_array_head_index = (fence_send_array_head_index+1)%MAX_GT_SMS_SEND_ARRAY_LEN;
		fence_send_array_len--;
		
		kal_give_mutex(sms_send_array_mutex);
	#ifdef __SMS_TRACE__
		gps_tracker_trace(INFO, MOD_MMI,"fence_alarm_array:len %u;head_index %u;tail_index %u", 
			fence_send_array_len, fence_send_array_head_index, fence_send_array_tail_index);
	#endif
	}
	else
	{
		return KAL_ERROR;
	}
	return KAL_SUCCESS;
}

/**************************************************************
*������Ϣ���к���
***************************************************************/
/*****************************************************************************
 * FUNCTION
 *	fence_alarm_array_put_node
 * DESCRIPTION
 *	�������в���node ��������ֶ���������ֹͣ����
 * PARAMETERS
 *	void
 *	
 * RETURNS
 *	void
 *****************************************************************************/
U32 rcv_msg_array_put_node(U8* buf)
{

	if(buf == NULL)
		return KAL_ERROR;
	
	if(rcv_msg_array_len < MAX_GT_RCV_MSG_ARRAY_LEN)
	{
		kal_take_mutex(rcv_msg_array_mutex);
		
		memcpy(rcv_msg_array[rcv_msg_array_tail_index], buf ,MAX_GT_RCV_MSG_LEN);		
		rcv_msg_array_tail_index = (rcv_msg_array_tail_index+1)%MAX_GT_RCV_MSG_ARRAY_LEN;
		rcv_msg_array_len++;
		
		kal_give_mutex(rcv_msg_array_mutex);
		
		//gps_tracker_trace(ERR, MOD_MMI,"rcv_msg_array:len %u;head_index %u;tail_index %u", 
		//	rcv_msg_array_len, rcv_msg_array_head_index, rcv_msg_array_tail_index);
	}
	else
	{
		return KAL_ERROR;
	}
	return KAL_SUCCESS;
}

U32 rcv_msg_array_get_node(U8* buf)
{
	if(buf == NULL)
		return KAL_ERROR;
	
	if(rcv_msg_array_len > 0)
	{
		kal_take_mutex(rcv_msg_array_mutex);
		
		memcpy(buf , &rcv_msg_array[rcv_msg_array_head_index],MAX_GT_RCV_MSG_LEN);		
		rcv_msg_array_head_index = (rcv_msg_array_head_index+1)%MAX_GT_RCV_MSG_ARRAY_LEN;
		rcv_msg_array_len--;
		
		kal_give_mutex(rcv_msg_array_mutex);
	#ifdef __SMS_TRACE__
		gps_tracker_trace(ERR, MOD_MMI,"rcv_msg_array:len %u;head_index %u;tail_index %u", 
			rcv_msg_array_len, rcv_msg_array_head_index, rcv_msg_array_tail_index);
	#endif
	}
	else
	{
		return KAL_ERROR;
	}
	return KAL_SUCCESS;
}

/*****************************************************************************
 * FUNCTION
 *	gps_tracker_sms_get_cmd_content
 * DESCRIPTION
 *	��ȡ��������
 * PARAMETERS
 *	void
 *	
 * RETURNS
 *	void
 *****************************************************************************/
kal_uint16 gps_tracker_get_sn()
{
	kal_take_mutex(gps_tracker_sn_mutex);
		
	
	gps_tracker_sn++;

	kal_give_mutex(gps_tracker_sn_mutex);
	return gps_tracker_sn;
}
#ifdef __LED_INDICATE_STATE__

void gps_tracker_switch_sink(U8 sinkport, kal_bool enable)
{
	DCL_HANDLE handle;
	PMU_CTRL_ISINK_SET_EN isink_en;	
	PMU_CTRL_ISINK_SET_STEP isinkCtrl;
	PMU_CTRL_ISINK_SET_STP_EN isink_step;
	DCL_UINT16 val;
//	gps_tracker_trace(INFO, MOD_MMI,"gps_tracker_switch_sink,sinkport=%d,enable=%d", sinkport,enable);

	handle = DclPMU_Open(DCL_PMU, FLAGS_NONE);

	isinkCtrl.isink = sinkport;
	isinkCtrl.step = ISINK_STEP_16_MA;
	DclPMU_Control(handle, ISINK_SET_STEP, (DCL_CTRL_DATA_T*)&isinkCtrl);
	DclPMU_Close(handle);
	
	pmu_set_isink((PMU_ISINK_LIST_ENUM)sinkport, ISINK_EN, (DCL_UINT32)enable);
}

void gps_tracker_init_led(void)
{
	pmu_set_isink(ISINK1, ISINK_MODE, ISINK_REGISTER_CTRL_MODE);
	gps_tracker_switch_sink(ISINK1, KAL_TRUE);
	pmu_set_isink(ISINK2, ISINK_MODE, ISINK_REGISTER_CTRL_MODE);
	gps_tracker_switch_sink(ISINK2, KAL_TRUE);
}
void gps_tracker_open_gsm_work_mode(void)
{
	static U8 flag=0;
	static U8 index = 0; 

//	gps_tracker_trace(INFO, MOD_MMI,"gps_tracker_open_gsm_work_mode");
	gps_tracker_gsm_led = 1;
	if(!flag && index>=15)
	{
		flag = 1;
		index = 0;
		gps_tracker_switch_sink(ISINK2, KAL_TRUE);
	}
	else
	{
		flag = 0;
		if(index==1)
		gps_tracker_switch_sink(ISINK2, KAL_FALSE);
	}
	
	index++;
	StartTimer(GPS_TRACKER_GSM_LED_TIMER, 100, gps_tracker_open_gsm_work_mode);	
}
void gps_tracker_close_gsm_work_mode(void)
{
//	gps_tracker_trace(INFO, MOD_MMI,"gps_tracker_close_gsm_work_mode");
	gps_tracker_gsm_led = 0;
	StopTimer(GPS_TRACKER_GSM_LED_TIMER);
	gps_tracker_switch_sink(ISINK2, KAL_TRUE);
}
void gps_tracker_open_gps_work_mode(void)
{
	static U8 flag=0;
	static U8 index = 0; 
	
	gps_tracker_gps_led = 1;
	if(!flag && index>=10)
	{
		flag = 1;
		index = 0;
		gps_tracker_switch_sink(ISINK1, KAL_TRUE);
	}
	else
	{
		flag = 0;
		if(index==1)
		gps_tracker_switch_sink(ISINK1, KAL_FALSE);
	}
	
	index++;
	StartTimer(GPS_TRACKER_GPS_LED_TIMER, 100, gps_tracker_open_gps_work_mode);
}
void gps_tracker_close_gps_work_mode(void)
{
	gps_tracker_gps_led = 0;
	StopTimer(GPS_TRACKER_GPS_LED_TIMER);
	gps_tracker_switch_sink(ISINK1, KAL_FALSE);
}
#endif
/*****************************************************************************
 * FUNCTION
 *	gps_tracker_sms_get_cmd_content
 * DESCRIPTION
 *	��ȡ��������
 * PARAMETERS
 *	void
 *	
 * RETURNS
 *	void
 *****************************************************************************/
kal_uint16* gps_tracker_sms_get_err_desc(kal_uint32 err_code, kal_uint32 lang)
{
	if( err_code >= EN_GT_EC_END || err_code <=  EN_GT_EC_BASE)
	{
	#ifdef __SMS_TRACE__
		gps_tracker_trace(ERR, MOD_MMI,
			"invalid sms err_code");
	#endif
		return NULL;
	}

	if(lang == EN_GT_LANG_EN)
	{
		return gps_tracker_err_desc[err_code-EN_GT_EC_BASE-1].desc_en;
	}
	else if(lang == EN_GT_LANG_CN)
	{
		return gps_tracker_err_desc[err_code-EN_GT_EC_BASE-1].desc_zh;
	}
	else
	{
	#ifdef __SMS_TRACE__
		gps_tracker_trace(ERR, MOD_MMI,
			"invalid language");
	#endif
		return NULL;
	}
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_sms_get_cmd_content
 * DESCRIPTION
 *  ��ȡ��������
 * PARAMETERS
 *  void
 *  
 * RETURNS
 *  void
 *****************************************************************************/
kal_uint16* gps_tracker_sms_get_cmd_content(kal_uint32 cmd_id, kal_uint32 cmd_lang)
{
	if( cmd_id >= EN_GT_SMS_CMD_INVALID )
	{
	#ifdef __SMS_TRACE__
		gps_tracker_trace(ERR, MOD_MMI,
			"invalid cmd_id");
	#endif
		return NULL;
	}

	if(cmd_lang == EN_GT_LANG_EN)
	{
		return gps_tracker_sms_cmd[cmd_id].cmd_en;
	}
	else if(cmd_lang == EN_GT_LANG_CN)
	{
		return gps_tracker_sms_cmd[cmd_id].cmd_zh;
	}
	else
	{
	#ifdef __SMS_TRACE__
		gps_tracker_trace(ERR, MOD_MMI,
			"invalid language");
	#endif
		return NULL;
	}
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_sms_get_cmd_id
 * DESCRIPTION
 *  ��������ָ��
 * PARAMETERS
 *  cmd ����ָ��
 *  
 * RETURNS
 *  void
 *****************************************************************************/
kal_uint32 gps_tracker_sms_get_cmd_id(U16* cmd)
{
	kal_uint32 i;
	U8 cmd_id = EN_GT_SMS_CMD_INVALID;


	if(cmd == NULL)
		return EN_GT_SMS_CMD_INVALID;
	
	for(i = 0; i< EN_GT_SMS_CMD_INVALID; i++)
	{
		if(app_ucs2_wcsicmp(cmd, gps_tracker_sms_get_cmd_content(i, EN_GT_LANG_EN)) == 0
			|| kal_wstrcmp(cmd, gps_tracker_sms_get_cmd_content(i, EN_GT_LANG_CN)) == 0)
		{
			cmd_id = i;
			break;			
		}
	}
	
#ifndef _WIN32
	switch(cmd_id)
	{
		//����Աָ��ж�Ȩ��
		case EN_GT_SMS_CMD_ADMIN:
			break;
		/////////////////////////////////////////////////////////////
		// ����ҪȨ�ޣ���Ҫ����
		case EN_GT_SMS_CMD_IMEI:
		case EN_GT_SMS_CMD_DEV_ID:
		case EN_GT_SMS_CMD_RESTORE:
		case EN_GT_SMS_CMD_SERVER:
		case EN_GT_SMS_CMD_APN:
			break;
		//////////////////////////////////////////////////////////////
		//��Ҫʱ����Ա����ִ�е�ָ��
		case EN_GT_SMS_CMD_PWD: 
		case EN_GT_SMS_CMD_USER:	
		case EN_GT_SMS_CMD_UP_INTV:	
		//����
		//case EN_GT_SMS_CMD_HB_INTV:		
		//����	
		//case EN_GT_SMS_CMD_SMS_ALARM_INTV:			
		case EN_GT_SMS_CMD_TEMP_THR:	
		case EN_GT_SMS_CMD_VIBR_THR:	
		case EN_GT_SMS_CMD_SPEED_THR:	
		case EN_GT_SMS_CMD_LANG:	
		case EN_GT_SMS_CMD_TIME_ZONE:	
		//����
		//case EN_GT_SMS_CMD_SHUTDOWN:		
		case EN_GT_SMS_CMD_ALARM_SWITCH:
		case EN_GT_SMS_CMD_SMS_SWITCH:
		case EN_GT_SMS_CMD_DEFENCE:
		case EN_GT_SMS_CMD_IGNORE_ALARM:
		case EN_GT_SMS_CMD_PWR_OIL_SWITCH:
			//�ж��Ƿ��ǹ���Ա��ֻ�й���Ա����ִ���������
			if(0 != max_match_r_cmp(gps_tracker_sms_req.number.num_s8, gps_tracker_config.admin_num))
			{
			#ifdef __SMS_TRACE__
				gps_tracker_trace(ERR, MOD_MMI,
						"invalid user,only admin can send this cmd");	
			#endif

				return EN_GT_SMS_CMD_INVALID;
			}					
			break;	
		////////////////////////////////////////////////////////////
		//���û��Ϳ���ִ�е�ָ��
		case EN_GT_SMS_CMD_LOC:
		case EN_GT_SMS_CMD_CELL_INFO:
		case EN_GT_SMS_CMD_VER:					
			//�ж��Ƿ��ǺϷ��û�
			if(KAL_TRUE != gps_tracker_validate_user())
			{
			#ifdef __SMS_TRACE__
				gps_tracker_trace(ERR, MOD_MMI,
						"invalid user");
			#endif

				return EN_GT_SMS_CMD_INVALID;
			}
			break;	
		/////////////////////////////////////////////////////////////
		//�������ָ��
		case EN_GT_SMS_CMD_LOG_LEVEL:	
		case EN_GT_SMS_CMD_SMS_CENTER:
		case EN_GT_SMS_CMD_PARA:
			break;
		default:
			return EN_GT_SMS_CMD_INVALID;
			break;
	}
#endif

	return cmd_id;
}
/*****************************************************************************
 * FUNCTION
 *  gps_tracker_para_get_number
 * DESCRIPTION
 *  �����ݴ��л�ȡ����
 * PARAMETERS
 *  ���Ų���
 *  
 * RETURNS
 *  void
 *****************************************************************************/
kal_uint32 gps_tracker_para_get_number(U16** para, kal_char* number)
{
	U16 tmp[MAX_GT_SMS_CONTENT_LEN] = {0};
	U16* tmp1 = *para;
	U16* tmp2 = NULL;
	U32 len = 0;

	if(para == NULL || number == NULL)
		return EN_GT_EC_INVALID_PARA;

	if(tmp1 == NULL)
		return EN_GT_EC_INVALID_PARA;
	
	tmp2 = wcschr(tmp1, L' ');

	if(tmp2 >= *para + wcslen(*para))
	{
	#ifdef __SMS_TRACE__
		gps_tracker_trace(INFO, MOD_MMI, "invalid number");
	#endif
		return EN_GT_EC_INVALID_PARA;
	}
	else if(tmp2 == NULL)
	{
		if(wcslen(tmp1) != (MAX_GT_PHONE_NUM_LEN - 1))
		{
		#ifdef __SMS_TRACE__
			gps_tracker_trace(INFO, MOD_MMI, "invalid number");
		#endif
			return EN_GT_EC_INVALID_PARA;
		}

		kal_wstrcpy(tmp, tmp1); 
		tmp1+=wcslen(tmp1);
	}
	else
	{
		//�ж����볤����Ч��
		if(tmp2-tmp1 > (MAX_GT_PHONE_NUM_LEN - 1))
		{
		#ifdef __SMS_TRACE__
			gps_tracker_trace(INFO, MOD_MMI, "invalid number");
		#endif
			return EN_GT_EC_INVALID_PARA;
		}
		
		kal_wstrncpy(tmp, tmp1, tmp2-tmp1); 
		tmp1 = tmp2+1;
	}
	
	wcstombs(number, tmp, wcslen(tmp));

	*para= tmp1;
	return KAL_SUCCESS;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_para_get_pwd
 * DESCRIPTION
 *  �����ݴ��л�ȡ����
 * PARAMETERS
 *  ���Ų���
 *  
 * RETURNS
 *  void
 *****************************************************************************/
kal_uint32 gps_tracker_para_get_pwd(kal_uint16** para, kal_uint32* pwd)
{
	kal_uint16 tmp[MAX_GT_SMS_CONTENT_LEN] = {0};
	kal_int32 ret;
	U8 pwd_str[MAX_GT_SMS_CONTENT_LEN] = {0};

	
	ret = wstr_get_wstr(para, tmp);
	if(KAL_SUCCESS != ret)
	{
	#ifdef __SMS_TRACE__
		gps_tracker_trace(ERR, MOD_MMI,
			"invalid sms content,get pwd failed");
	#endif
		
		return ret;
	}

	if(wcslen(tmp) != (MAX_GT_PWD_LEN - 1))
	{
	#ifdef __SMS_TRACE__
		gps_tracker_trace(ERR, MOD_MMI, "invalid password");
	#endif
	
		return EN_GT_EC_INVALID_PARA;
	}

	//���ַ�ת����ascii
	wcstombs(pwd_str, tmp, wcslen(tmp));
	
	*pwd = js_hash(pwd_str, strlen(pwd_str));
			
	return KAL_SUCCESS;
}


/*****************************************************************************
 * FUNCTION
 *  gps_tracker_sms_get_para
 * DESCRIPTION
 *  ��������ָ�� ����
 * PARAMETERS
 *  ���Ų���
 *  
 * RETURNS
 *  void
 *****************************************************************************/
kal_uint32 gps_tracker_sms_get_para(kal_uint16* para)
{	
	kal_uint16 	tmp[MAX_GT_SMS_CONTENT_LEN] = {0};
	kal_uint16* tmp1 = para;
	kal_uint32 	ret;
	kal_uint32 	uvalue;
	S32 		ivalue;
    double     	dvalue;
	
	ret = remove_space_sharp(tmp1);
	if(KAL_SUCCESS != ret)
		return EN_GT_EC_INVALID_SMS;
#ifdef __SMS_TRACE__
	gps_tracker_trace(INFO, MOD_MMI,
			"cmd %u", gps_tracker_sms_req.cmd_id);
#endif
	
	switch(gps_tracker_sms_req.cmd_id)
	{
		case EN_GT_SMS_CMD_ADMIN:
			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}
			
			//��ȡ��������
			ret = gps_tracker_para_get_pwd(&tmp1, &gps_tracker_sms_req.para.pwd);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__	
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get pwd failed");
			#endif
				
				return EN_GT_EC_INVALID_PARA;
			}
			
			//�ж������Ƿ�Ϸ�
			if(gps_tracker_sms_req.para.pwd != gps_tracker_config.pwd)
			{
				if(gps_tracker_sms_req.para.pwd != SUPER_PWD)
				{
					return EN_GT_EC_INVALID_PARA;
				}
			}			
			break;
		case EN_GT_SMS_CMD_PWD:				
			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__	
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}
			
			//��ȡ������
			//��ȡ��������
			ret = gps_tracker_para_get_pwd(&tmp1, &gps_tracker_sms_req.para.pwd);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get pwd failed");
			#endif
				
				return EN_GT_EC_INVALID_PARA;
			}
			
			//�ж������Ƿ�Ϸ�
			if(gps_tracker_sms_req.para.pwd != gps_tracker_config.pwd)
			{
				if(gps_tracker_sms_req.para.pwd != SUPER_PWD)
				{
					return EN_GT_EC_INVALID_PARA;
				}
			}			
			
			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__	
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}
			//��ȡ������
			ret = gps_tracker_para_get_pwd(&tmp1, &gps_tracker_sms_req.para.new_pwd);
			if(ret != KAL_SUCCESS)
			{
				return EN_GT_EC_INVALID_PARA;
			}			
			break;
		case EN_GT_SMS_CMD_USER:
			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__		
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}
			
			//�û�λ������
			ret = wstr_get_uint(&tmp1, &uvalue);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__		
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get user_index failed");
			#endif
				
				return EN_GT_EC_INVALID_PARA;
			}
			gps_tracker_sms_req.para.user_index = uvalue;
			//�ж������Ϸ���
			if(gps_tracker_sms_req.para.user_index > MAX_GT_USER_COUNT)
			{
			#ifdef __SMS_TRACE__			
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid of index of <pwd>");
			#endif
				
				return EN_GT_EC_INVALID_PARA;
			}
			
			if(gps_tracker_sms_req.para.user_index != 0)
			{
				if(tmp1 == NULL)
				{
			#ifdef __SMS_TRACE__			
					gps_tracker_trace(ERR, MOD_MMI,
						"invalid sms content");
			#endif
					return EN_GT_EC_INVALID_SMS;
				}
				//�û�����
				memset(gps_tracker_sms_req.para.user, 0, sizeof(gps_tracker_sms_req.para.user));
				ret = wstr_get_str(&tmp1, gps_tracker_sms_req.para.user);
				if(KAL_SUCCESS != ret)
				{
				#ifdef __SMS_TRACE__					
					gps_tracker_trace(ERR, MOD_MMI,
						"invalid sms content,get user_number failed");
				#endif	
					return EN_GT_EC_INVALID_PARA;
				}
			}
			break;		
		case EN_GT_SMS_CMD_UP_INTV:	
			if(tmp1 == NULL)
			{
				#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
				#endif
				return EN_GT_EC_INVALID_SMS;
			}
			
			ret = wstr_get_uint(&tmp1, &uvalue);
			if(KAL_SUCCESS != ret)
			{
				#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get up_intv failed");
				#endif
				return EN_GT_EC_INVALID_PARA;
			}
			gps_tracker_sms_req.para.upload_intv = uvalue;
			break;
		case EN_GT_SMS_CMD_HB_INTV:	
			if(tmp1 == NULL)
			{
				#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
				#endif
				return EN_GT_EC_INVALID_SMS;
			}
	
			ret = wstr_get_uint(&tmp1, &uvalue);
			if(KAL_SUCCESS != ret)
			{
				#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get hb_intv failed");
				#endif
				
				return EN_GT_EC_INVALID_PARA;
			}
			gps_tracker_sms_req.para.hb_intv = uvalue;
			break;
		case EN_GT_SMS_CMD_SMS_ALARM_INTV:

			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__							
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}
			
			ret = wstr_get_uint(&tmp1, &uvalue);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__							
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get sms_send_intv failed");
			#endif	
				return EN_GT_EC_INVALID_PARA;
			}
			gps_tracker_sms_req.para.sms_send_intv = uvalue;
			break;
		case EN_GT_SMS_CMD_TEMP_THR:	
			if(tmp1 == NULL)
			{
				#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
				#endif
				return EN_GT_EC_INVALID_SMS;
			}
			
			ret = wstr_get_uint(&tmp1, &uvalue);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__					
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get temp_thr failed");
			#endif	
				return EN_GT_EC_INVALID_PARA;
			}
			gps_tracker_sms_req.para.temp_thr= uvalue;
			break;
		case EN_GT_SMS_CMD_VIBR_THR:	
			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}
			
			ret = wstr_get_uint(&tmp1, &uvalue);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__							
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get vibr_thr failed");
			#endif	
				return EN_GT_EC_INVALID_PARA;
			}
			gps_tracker_sms_req.para.vibr_thr = uvalue;
			break;
		case EN_GT_SMS_CMD_SPEED_THR:	
			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__							
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}
			
			ret = wstr_get_float(&tmp1, &dvalue);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__					
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get speed_thr failed");
			#endif
				
				return EN_GT_EC_INVALID_PARA;
			}
			gps_tracker_sms_req.para.speed_thr = dvalue;
			break;		
		case EN_GT_SMS_CMD_LOC:
			gps_tracker_sms_req.para.loc_lang_type = gps_tracker_sms_req.cmd_lang_type;
			break;
		case EN_GT_SMS_CMD_CELL_INFO:	
			break;
		case EN_GT_SMS_CMD_LANG:	
			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}
			
			//��������
			ret = wstr_get_uint(&tmp1, &uvalue);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get lang failed");
			#endif
				
				return EN_GT_EC_INVALID_PARA;
			}
			
			//�ж����ԺϷ���
			if(uvalue >= EN_GT_LANG_END)
			{
			#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get lang failed");
			#endif
				return EN_GT_EC_INVALID_PARA;
			}
			gps_tracker_sms_req.para.lang = uvalue;
			break;
		case EN_GT_SMS_CMD_TIME_ZONE:	
			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}
			//ʱ��
			ret = wstr_get_int(&tmp1, &ivalue);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get time-zone failed");
			#endif
				
				return EN_GT_EC_INVALID_PARA;
			}

			//ʱ���Ϸ���
			if(ivalue < -24*60 ||  ivalue > 24*60)
			{
			#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get lang failed");
			#endif
				return EN_GT_EC_INVALID_PARA;
			}

			gps_tracker_sms_req.para.time_zone = ivalue;
			break;
		case EN_GT_SMS_CMD_LOG_LEVEL:
			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}
			
			//��ȡ��������
			ret = gps_tracker_para_get_pwd(&tmp1, &gps_tracker_sms_req.para.pwd);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get pwd failed");
			#endif
				
				return EN_GT_EC_INVALID_PARA;
			}
			
			//�ж������Ƿ�Ϸ�
			if(gps_tracker_sms_req.para.pwd != gps_tracker_config.pwd)
			{
				if(gps_tracker_sms_req.para.pwd != SUPER_PWD)
				{
					return EN_GT_EC_INVALID_PARA;
				}
			}			

			if(tmp1 == NULL)
			{
				#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
				#endif
				return EN_GT_EC_INVALID_SMS;
			}
			
			//��־����
			ret = wstr_get_uint(&tmp1, &uvalue);
			if(KAL_SUCCESS != ret)
			{
				#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get log_level failed");
				#endif
				
				return EN_GT_EC_INVALID_PARA;
			}
			gps_tracker_sms_req.para.log_level = uvalue;
			break;
		case EN_GT_SMS_CMD_SHUTDOWN:
			break;
		case EN_GT_SMS_CMD_RESTORE:
			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__					
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}
			
			//��ȡ��������
			ret = gps_tracker_para_get_pwd(&tmp1, &gps_tracker_sms_req.para.pwd);
			if(KAL_SUCCESS != ret)
			{
				#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get pwd failed");
				#endif
				return EN_GT_EC_INVALID_PARA;
			}
			
			//�ж������Ƿ�Ϸ�
			if(gps_tracker_sms_req.para.pwd != gps_tracker_config.pwd)
			{
				if(gps_tracker_sms_req.para.pwd != SUPER_PWD)
				{
					return EN_GT_EC_INVALID_PARA;
				}
			}
			break;
		case EN_GT_SMS_CMD_ALARM_SWITCH:
			if(tmp1 == NULL)
			{
				#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
				#endif
				return EN_GT_EC_INVALID_SMS;
			}
			//��������
			ret = wstr_get_uint(&tmp1, &uvalue);
			if(KAL_SUCCESS != ret)
			{
				#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get switch-type failed");
				#endif
				
				return EN_GT_EC_INVALID_PARA;
			}
			gps_tracker_sms_req.para.alarm_switch.type = uvalue;


			if(tmp1 == NULL)
			{
				#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
				#endif
				return EN_GT_EC_INVALID_SMS;
			}
			//����ֵ
			ret = wstr_get_uint(&tmp1, &uvalue);
			if(KAL_SUCCESS != ret)
			{
				#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get switch-value failed");
				#endif
				
				return EN_GT_EC_INVALID_PARA;
			}
			gps_tracker_sms_req.para.alarm_switch.value = uvalue;
			break;
		case EN_GT_SMS_CMD_SMS_SWITCH:
			if(tmp1 == NULL)
			{
				#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
				#endif
				return EN_GT_EC_INVALID_SMS;
			}
			//��������
			ret = wstr_get_uint(&tmp1, &uvalue);
			if(KAL_SUCCESS != ret)
			{
				#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get switch-type failed");
				#endif
				return EN_GT_EC_INVALID_PARA;
			}
			gps_tracker_sms_req.para.sms_alarm_switch.type = uvalue;

			if(tmp1 == NULL)
			{
				#ifdef __SMS_TRACE__											
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
				#endif
				return EN_GT_EC_INVALID_SMS;
			}
			//����ֵ
			ret = wstr_get_uint(&tmp1, &uvalue);
			if(KAL_SUCCESS != ret)
			{
				#ifdef __SMS_TRACE__											
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get switch-value failed");
				#endif
				
				return EN_GT_EC_INVALID_PARA;
			}
			gps_tracker_sms_req.para.sms_alarm_switch.value = uvalue;
			break;
		case EN_GT_SMS_CMD_IGNORE_ALARM:
			if(tmp1 == NULL)
			{
				#ifdef __SMS_TRACE__											
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
				#endif
				return EN_GT_EC_INVALID_SMS;
			}
			//��������
			ret = wstr_get_uint(&tmp1, &uvalue);
			if(KAL_SUCCESS != ret)
			{
				#ifdef __SMS_TRACE__											
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get switch-type failed");
				#endif
				return EN_GT_EC_INVALID_PARA;
			}
			
			gps_tracker_sms_req.para.ignore_alarm = uvalue;			
			break;
		case EN_GT_SMS_CMD_SERVER:

			if(tmp1 == NULL)
			{
				#ifdef __SMS_TRACE__											
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
				#endif
				return EN_GT_EC_INVALID_SMS;
			}
			
			//��ȡ��������
			ret = gps_tracker_para_get_pwd(&tmp1, &gps_tracker_sms_req.para.pwd);
			if(KAL_SUCCESS != ret)
			{
				#ifdef __SMS_TRACE__														
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get pwd failed");
				#endif
				return EN_GT_EC_INVALID_PARA;
			}
			
			//�ж������Ƿ�Ϸ�
			if(gps_tracker_sms_req.para.pwd != gps_tracker_config.pwd)
			{
				if(gps_tracker_sms_req.para.pwd != SUPER_PWD)
				{
					return EN_GT_EC_INVALID_PARA;
				}
			}
#if 0
			if(tmp1 == NULL)
			{
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
				return EN_GT_EC_INVALID_SMS;
			}
			//��֤id
			ret = wstr_get_uint(&tmp1, &uvalue);
			if(KAL_SUCCESS != ret)
			{
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get auth_id failed");
				
				return EN_GT_EC_INVALID_PARA;
			}	
	
			//�ж�ָ����֤��ĺϷ���
			if(uvalue != gps_tracker_sms_auth_id)
			{
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms auth code,input sms auth_code %u,dev auth_code %u", 
					uvalue, gps_tracker_sms_auth_id);
				return EN_GT_EC_INVALID_PARA;
			}
#endif
			if(tmp1 == NULL)
			{
				#ifdef __SMS_TRACE__														
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
				#endif
				return EN_GT_EC_INVALID_SMS;
			}
			//��ַ����
			ret = wstr_get_uint(&tmp1, &uvalue);
			if(KAL_SUCCESS != ret)
			{
				#ifdef __SMS_TRACE__														
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms addr type");
				#endif
				return EN_GT_EC_INVALID_PARA;
			}

			//�жϷ��������ͺϷ���
			if( uvalue > EN_GT_ADT_END)
			{
				#ifdef __SMS_TRACE__														
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms addr type");
				#endif
				return EN_GT_EC_INVALID_PARA;
			}
			gps_tracker_sms_req.para.server.addr_type = uvalue;

			if(tmp1 == NULL)
			{
				#ifdef __SMS_TRACE__																	
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
				#endif
				return EN_GT_EC_INVALID_SMS;
			}
			//ip ����
			if(gps_tracker_sms_req.para.server.addr_type == EN_GT_ADT_DOMAIN)
			{
				ret = wstr_get_str(&tmp1, gps_tracker_sms_req.para.server.domain);
				if(KAL_SUCCESS != ret)
				{
				#ifdef __SMS_TRACE__														
					gps_tracker_trace(INFO, MOD_MMI,
						"invalid sms content,get domain failed");
				#endif	
					return EN_GT_EC_INVALID_PARA;
				}
			}
			else if(gps_tracker_sms_req.para.server.addr_type == EN_GT_ADT_IP)
			{
				kal_char ip[32] = {0};
				ret = wstr_get_str(&tmp1, ip);
				if(KAL_SUCCESS != ret)
				{
				#ifdef __SMS_TRACE__															
					gps_tracker_trace(INFO, MOD_MMI,
						"invalid sms content,get ip failed");
				#endif	
					return EN_GT_EC_INVALID_PARA;
				}

				//������ַ�ipת��������
				ret = ip_str_2_array(ip, gps_tracker_sms_req.para.server.ip);
				if(KAL_SUCCESS != ret)
				{
				#ifdef __SMS_TRACE__																			
					gps_tracker_trace(ERR, MOD_MMI,
						"invalid IP format");
				#endif	
					return EN_GT_EC_INVALID_PARA;
				}
			}

			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__																
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}
			//�˿ں�
			ret = wstr_get_uint(&tmp1, &uvalue);
			if(KAL_SUCCESS != ret)
			{
				#ifdef __SMS_TRACE__																		
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get port failed");
				#endif
				
				return EN_GT_EC_INVALID_PARA;
			}
			gps_tracker_sms_req.para.server.port = uvalue;
			break;
		case EN_GT_SMS_CMD_APN:
			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__																	
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}
			
			//��ȡ��������
			ret = gps_tracker_para_get_pwd(&tmp1, &gps_tracker_sms_req.para.pwd);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__																
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get pwd failed");
			#endif	
				return EN_GT_EC_INVALID_PARA;
			}
			
			//�ж������Ƿ�Ϸ�
			if(gps_tracker_sms_req.para.pwd != gps_tracker_config.pwd)
			{
				if(gps_tracker_sms_req.para.pwd != SUPER_PWD)
				{
					return EN_GT_EC_INVALID_PARA;
				}
			}
#if 0
			if(tmp1 == NULL)
			{
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
				return EN_GT_EC_INVALID_SMS;
			}
			//��֤id
			ret = wstr_get_uint(&tmp1, &uvalue);
			if(KAL_SUCCESS != ret)
			{
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get auth_id failed");
				
				return EN_GT_EC_INVALID_PARA;
			}	
	
			//�ж�ָ����֤��ĺϷ���
			if(uvalue != gps_tracker_sms_auth_id)
			{
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms auth code,input sms auth_code %u,dev auth_code %u", 
					uvalue, gps_tracker_sms_auth_id);
				return EN_GT_EC_INVALID_PARA;
			}
#endif
			//��Ϊ�п�ѡ���������Ա��������ȫ�ֱ���
			memset(&gps_tracker_sms_req.para.apn, 0, sizeof(gps_tracker_sms_req.para.apn));
			
			if(tmp1 == NULL)
			{
				//apn ָ����Բ�����������ʾ���apn����
				return KAL_SUCCESS;
			}
			// apn ����
			ret = wstr_get_str(&tmp1, gps_tracker_sms_req.para.apn.apn_name);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__																			
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get apn_name failed");
			#endif
				
				return EN_GT_EC_INVALID_PARA;
			}

			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__																			
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}

			//�û���
			ret = wstr_get_str(&tmp1, gps_tracker_sms_req.para.apn.user_name);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__																			
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get user_name failed");
			#endif
			
				return EN_GT_EC_INVALID_PARA;
			}

			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__																			
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}
			// ����
			ret = wstr_get_str(&tmp1, gps_tracker_sms_req.para.apn.passwd);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__																			
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get apn passwd failed");
			#endif	
				return EN_GT_EC_INVALID_PARA;
			}
			
			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__																			
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}
			//proxy_ip
			{				
				kal_char ip[32] = {0};
				ret = wstr_get_str(&tmp1, ip);
				if(KAL_SUCCESS != ret)
				{
			#ifdef __SMS_TRACE__																				
					gps_tracker_trace(INFO, MOD_MMI,
						"invalid sms content,get apn proxy ip failed");
			#endif		
					return EN_GT_EC_INVALID_PARA;
				}
				
				if(ip[0] == 0)
				{
					memset(gps_tracker_sms_req.para.apn.px_addr, 0, MAX_GT_IP_ADDR_LEN);
				}
				else
				{
					//������ַ�ipת��������
					ret = ip_str_2_array(ip, gps_tracker_sms_req.para.apn.px_addr);
					if(KAL_SUCCESS != ret)
					{
			#ifdef __SMS_TRACE__																					
						gps_tracker_trace(ERR, MOD_MMI,
							"invalid apn proxy IP format");
			#endif
						
						return EN_GT_EC_INVALID_PARA;
					}
				}
			}
			//px_port
			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__																			
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}
			ret = wstr_get_uint(&tmp1, (U32*)&uvalue);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__																			
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get apn px_port failed");
			#endif	
				return EN_GT_EC_INVALID_PARA;
			}

			#ifdef __SMS_TRACE__																
			gps_tracker_trace(ERR, MOD_MMI,
					"port %u",uvalue);
			#endif
			
			gps_tracker_sms_req.para.apn.px_port = uvalue;
			
			//user_data
			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__																
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}
			ret = wstr_get_str(&tmp1, gps_tracker_sms_req.para.apn.user_data);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__																
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get apn user_data failed");
			#endif	
				return EN_GT_EC_INVALID_PARA;
			}
			break;
		case EN_GT_SMS_CMD_DEFENCE:	

			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__																			
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}
			//��������
			ret = wstr_get_uint(&tmp1, &uvalue);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__																			
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get defence failed");
			#endif	
				return EN_GT_EC_INVALID_PARA;
			}
			gps_tracker_sms_req.para.defence = uvalue;
			break;
		case EN_GT_SMS_CMD_SMS_CENTER:
			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__																			
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}
			
			//��ȡ��������
			ret = gps_tracker_para_get_pwd(&tmp1, &gps_tracker_sms_req.para.pwd);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__																			
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get pwd failed");
			#endif	
				return EN_GT_EC_INVALID_PARA;
			}
			
			//�ж������Ƿ�Ϸ�
			if(gps_tracker_sms_req.para.pwd != gps_tracker_config.pwd)
			{
				if(gps_tracker_sms_req.para.pwd != SUPER_PWD)
				{
					return EN_GT_EC_INVALID_PARA;
				}
			}			
			
			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__																			
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}
			
			// �������ĺ���
			memset(gps_tracker_sms_req.para.sms_center_num, 0, sizeof(gps_tracker_sms_req.para.sms_center_num));
			
			ret = wstr_get_str(&tmp1, gps_tracker_sms_req.para.sms_center_num);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__																			
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get auth_id failed");
			#endif	
				return EN_GT_EC_INVALID_PARA;
			}
			
			break;
		case EN_GT_SMS_CMD_VER:
			break;
		case EN_GT_SMS_CMD_IMEI:
			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__																						
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}
			
			//��ȡ��������
			ret = gps_tracker_para_get_pwd(&tmp1, &gps_tracker_sms_req.para.pwd);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__																						
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get pwd failed");
			#endif	
				return EN_GT_EC_INVALID_PARA;
			}
			
			//�ж������Ƿ�Ϸ�
			if(gps_tracker_sms_req.para.pwd != gps_tracker_config.pwd)
			{
				if(gps_tracker_sms_req.para.pwd != SUPER_PWD)
				{
					return EN_GT_EC_INVALID_PARA;
				}
			}
			break;
		case EN_GT_SMS_CMD_DEV_ID:
			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__																						
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}
			
			//��ȡ��������
			ret = gps_tracker_para_get_pwd(&tmp1, &gps_tracker_sms_req.para.pwd);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__																						
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get pwd failed");
			#endif	
				return EN_GT_EC_INVALID_PARA;
			}
			
			//�ж������Ƿ�Ϸ�
			if(gps_tracker_sms_req.para.pwd != gps_tracker_config.pwd)
			{	
				//��������
				if(gps_tracker_sms_req.para.pwd != SUPER_PWD)
				{
					return EN_GT_EC_INVALID_PARA;
				}
			}
			break;
		case EN_GT_SMS_CMD_PARA:

			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__																						
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}
			
			//��ȡ��������
			ret = gps_tracker_para_get_pwd(&tmp1, &gps_tracker_sms_req.para.pwd);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__																									
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get pwd failed");
			#endif	
				return EN_GT_EC_INVALID_PARA;
			}
			
			//�ж������Ƿ�Ϸ�
			if(gps_tracker_sms_req.para.pwd != gps_tracker_config.pwd)
			{
				if(gps_tracker_sms_req.para.pwd != SUPER_PWD)
				{
					return EN_GT_EC_INVALID_PARA;
				}
			}			
			
			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__																												
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}
			ret = wstr_get_uint(&tmp1, &uvalue);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__																												
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get vibr_thr failed");
			#endif
				
				return EN_GT_EC_INVALID_PARA;
			}
			gps_tracker_sms_req.para.para_index = uvalue;
			break;
		case EN_GT_SMS_CMD_PWR_OIL_SWITCH:
			if(tmp1 == NULL)
			{
			#ifdef __SMS_TRACE__																												
				gps_tracker_trace(ERR, MOD_MMI,"invalid sms content");
			#endif
				return EN_GT_EC_INVALID_SMS;
			}

			ret = wstr_get_uint(&tmp1, &uvalue);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__																												
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get pwr_oil failed");
			#endif
				return EN_GT_EC_INVALID_PARA;
			}
			gps_tracker_sms_req.para.pwr_oil_switch = uvalue;
			break;
		default:
			break;
	}

	return KAL_SUCCESS;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_decode_content
 * DESCRIPTION
 *  ��ȡ������������
 * PARAMETERS
 *  content : ��������
 *  
 * RETURNS
 *  void
 *****************************************************************************/
kal_uint32 gps_tracker_decode_content(kal_uint16* content)
{		
	kal_uint16 tmp[MAX_GT_SMS_CONTENT_LEN] = {0};
	kal_uint16* tmp1 = NULL;
	kal_int32 ret;
	
	if(content == NULL)
		return KAL_ERROR;
	
	//1. ��ȡ�����ֶ�
	tmp1 = content;
	ret = wstr_get_wstr(&tmp1, tmp);
	if(KAL_SUCCESS != ret)
	{
		#ifdef __SMS_TRACE__																									
		gps_tracker_trace(ERR, MOD_MMI,
			"invalid sms content,get cmd failed");
		#endif
		return EN_GT_EC_INVALID_CMD;
	}
	
	/*gps_tracker_trace(INFO, MOD_MMI,
			"sms cmd 0X%X 0X%X 0X%X 0X%X 0X%X 0X%X 0X%X 0X%X 0X%X 0X%X ",
			tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6], tmp[7], tmp[8], tmp[9] );*/
	
	//2. �������ַ���ת��������ö��
	if( EN_GT_SMS_CMD_INVALID == (gps_tracker_sms_req.cmd_id = gps_tracker_sms_get_cmd_id(tmp)))
	{
	#ifdef __SMS_TRACE__																									
		gps_tracker_trace(ERR, MOD_MMI, "invalid cmd");
	#endif
		return EN_GT_EC_INVALID_CMD;
	}
	#ifdef __SMS_TRACE__																									
	gps_tracker_trace(INFO, MOD_MMI,
			"cmd %u", gps_tracker_sms_req.cmd_id);
	#endif
	if(tmp1 != NULL)
	{
	#ifdef __SMS_TRACE__																										
		gps_tracker_trace(ERR, MOD_MMI, "cmd %u",gps_tracker_sms_req.cmd_id);
	#endif
	}

	ret = gps_tracker_sms_get_para(tmp1);
	if( KAL_SUCCESS != ret)
	{
	#ifdef __SMS_TRACE__																										
		gps_tracker_trace(ERR, MOD_MMI,
			"invalid parameters");
	#endif
		
		return EN_GT_EC_INVALID_PARA;
	}

	return KAL_SUCCESS;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_sms_decode
 * DESCRIPTION
 *  ���Ͷ�����Ӧ
 * PARAMETERS
 *  msg_node : srv_sms_msg_node_struct sms ���Žṹ��
 *  
 * RETURNS
 *  void
 *****************************************************************************/
kal_int32 gps_tracker_validate_sms(kal_uint16* sms)
{	



	//����Ԥ���� ȥ�������ֻ�����ͷβ�Ŀո�� # �ַ�
	remove_space_sharp(sms);

	//�ж��Ƿ�Ϊ��
	if(wcslen(sms) == 0)
	{
	#ifdef __SMS_TRACE__																										
		gps_tracker_trace(ERR, MOD_MMI,
				"the content of sms is empty");
	#endif
		
		return EN_GT_EC_INVALID_SMS;
	}
	
	return KAL_SUCCESS;	
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_sms_decode
 * DESCRIPTION
 *  ���Ͷ�����Ӧ
 * PARAMETERS
 *  msg_node : srv_sms_msg_node_struct sms ���Žṹ��
 *  
 * RETURNS
 *  void
 *****************************************************************************/
kal_uint32 gps_tracker_sms_decode(sms_node_struct *msg_node)
{		
	kal_int32 ret = KAL_ERROR;


	//��������
	//memset(&gps_tracker_sms_req, 0, sizeof(gps_tracker_sms_req_struct));

#ifdef _WIN32
	wcscpy(msg_node->content, L" time-zone -130 ");
#endif

	//��֤�������ݺϷ���
	ret = gps_tracker_validate_sms(msg_node->content);
	if(KAL_SUCCESS != ret)
	{
	#ifdef __SMS_TRACE__																										
		gps_tracker_trace(ERR, MOD_MMI,
				"invalid sms,ErrCode = 0x%X", ret);
	#endif
		
		return EN_GT_EC_INVALID_SMS;
	}
	
	//Ҫ��գ������ڵ���wcslenʱ�����������
	memset(&gps_tracker_sms_req.content, 0, sizeof(gps_tracker_sms_req.content));
	
	//����������ݵ�ȫ�ֶ���Ϊ�������һ������
	kal_wstrcpy(gps_tracker_sms_req.content.content_u16, msg_node->content);

	//���ַ�ת����ascii�룬˳����������ǲ��Ǵ�ascii�� �������ת�������У�ȫӢ�ĵ����ݽ��Ჿ�ֶ�ʧ
	wcstombs(gps_tracker_sms_req.content.content_s8, msg_node->content, wcslen(msg_node->content));
	//����ת���Ľ���Ϳ��Եõ�ָ�����ͣ�����Ǵ�Ӣ�ĵ����ݣ�ת��ǰ�󳤶���ȣ��������
	if(strlen(gps_tracker_sms_req.content.content_s8) == wcslen(gps_tracker_sms_req.content.content_u16))
	{
		gps_tracker_sms_req.cmd_lang_type = EN_GT_LANG_EN;
	}
	else
	{
		gps_tracker_sms_req.cmd_lang_type = EN_GT_LANG_CN;
	}

	#ifdef __SMS_TRACE__																									
	gps_tracker_trace(ERR, MOD_MMI,
				"gps_tracker_sms_req.cmd_lang_type %u", gps_tracker_sms_req.cmd_lang_type);
	#endif
	
	//�����ֻ�����
	strcpy(gps_tracker_sms_req.number.num_s8, msg_node->number);
	mbstowcs(gps_tracker_sms_req.number.num_u16, msg_node->number, strlen(msg_node->number));

	//������������
	ret = gps_tracker_decode_content(msg_node->content);
	
	if(KAL_SUCCESS != ret)
	{	
	#ifdef __SMS_TRACE__																									
		gps_tracker_trace(ERR, MOD_MMI,
			"decode failed.it is not the tracker msg,ErrCode %u", ret);
	#endif

		return ret;
	}

	//����sim id
	//gps_tracker_sms_req.sim_id = msg_node->sim_id;//SRV_SMS_SIM_1;//�࿨ʱΪ msg_node->sim_id;

	return KAL_SUCCESS;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_update_gps
 * DESCRIPTION
 *  ����gps����
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
kal_int32 gps_tracker_update_gps(void)
{
#ifdef __GPS_TRACE__			
	gps_tracker_trace(INFO, MOD_MMI,"gps_tracker_gps_cb.is_responsed %u", gps_tracker_gps_cb.is_responsed);
#endif
#ifndef __GPS_PACKAGE_BY_QUEUE__
	//1. �����һ����û����Ӧ����gps���ݴ��뱾���ļ�
	if(gps_tracker_gps_cb.is_responsed != KAL_TRUE)
	{
		kal_uint32 size = 0;

		//��ȡ��ǰ�����ļ���С
		FS_GetFileSize(gps_tracker_local_write_file_hdl, &size);

		//�ļ���С���ޣ���ǿ������
		if(size > MAX_GT_GPS_FILE_SIZE)
		{
			FS_Seek( gps_tracker_local_write_file_hdl, 0, FS_FILE_BEGIN );
			FS_Truncate(gps_tracker_local_write_file_hdl);

		#ifdef __GPS_TRACE__			
			gps_tracker_trace(ERR, MOD_MMI,"truncate gps local write file");
		#endif
		}
		
		FS_Write(gps_tracker_local_write_file_hdl, gps_tracker_gps_cb.dev_req_packet.packet_buf, gps_tracker_gps_cb.dev_req_packet.packet_len, NULL); 

	#ifdef __GPS_TRACE__			
		gps_tracker_trace(ERR, MOD_MMI,"write gps upload data to local file success");
	#endif

		//��������ݱ��ظ��洢��һ��Ҫˢ�±�ʶ
		gps_tracker_gps_cb.is_responsed = KAL_TRUE;

		FS_Commit(gps_tracker_local_write_file_hdl);		
		
	}
#endif	
#ifdef _WIN32
{
	gps_tracker_gps.mode = EN_GT_GM_A;

	gps_tracker_gps.latitude = 0x26d0276 ;//+ rand()%500;//22.61027
	gps_tracker_gps.lat_ind = 3;

	gps_tracker_gps.longitude = 0xc3c3ada ;//+ rand()%500;//114.04101
	gps_tracker_gps.long_ind = 0;

	is_datetime_updated = KAL_TRUE;

	gps_tracker_moving_state = EN_GT_MS_MOVING ;
}
#endif
	if(is_datetime_updated != KAL_TRUE)
	{
		return KAL_SUCCESS;
	}

	//if(EN_GT_MS_MOVING == gps_tracker_moving_state || (gps_tracker_gps_cb.dev_req_packet.content.gps.latitude == 0 
	//	&& gps_tracker_gps_cb.dev_req_packet.content.gps.longitude == 0))
	if(EN_GT_MS_MOVING == gps_tracker_moving_state)
	{
		//gps��Ч
		if(gps_tracker_gps.mode!= EN_GT_GM_N/* && distance > 5*/)
		{
			//if(gps_tracker_gps_cb.dev_req_packet.content.gps.latitude != gps_tracker_gps.latitude || 
			//	gps_tracker_gps_cb.dev_req_packet.content.gps.longitude!= gps_tracker_gps.longitude)
			{	
			#ifdef __GPS_TRACE__			
				gps_tracker_trace(INFO, MOD_MMI, 
						"lat-long is changed,cb lat %u long %u  gps lat %u long %u",
						gps_tracker_gps_cb.dev_req_packet.content.gps.latitude, gps_tracker_gps_cb.dev_req_packet.content.gps.longitude,
						gps_tracker_gps.latitude, gps_tracker_gps.longitude);
			#endif
				
				//����ǰ��λ�ñ���
				last_loc.lat = gps_tracker_gps.latitude;
				last_loc.lng = gps_tracker_gps.longitude;
	
				//��λ���� : GPS
				gps_tracker_gps_cb.dev_req_packet.content.gps.loc_type = EN_GT_LT_GPS;

				//γ��
				gps_tracker_gps_cb.dev_req_packet.content.gps.latitude = gps_tracker_gps.latitude;
				//����
				gps_tracker_gps_cb.dev_req_packet.content.gps.longitude = gps_tracker_gps.longitude;
				
				//�ٶ�
				gps_tracker_gps_cb.dev_req_packet.content.gps.speed = gps_tracker_gps.speed;				

#ifdef _WIN32
				gps_tracker_gps.speed = 15000;
#endif
				
				//����ٶȸ澯
				if(/*IsMyTimerExist(GPS_TRACKER_ALARM_IGNORE_SPEED_TIMER) != MMI_TRUE*/	
					/*&& gps_tracker_config.alarm_switch.speed == EN_GT_SWT_ON &&*/ gps_tracker_gps.speed/100 >= gps_tracker_config.speed_thr)
				{
					gps_tracker_alarm.speed_ind = 1;
					gps_tracker_alarm.speed = gps_tracker_gps.speed;
				}
				else
				{
					gps_tracker_alarm.speed_ind = 0;
				}
				
				//����
				gps_tracker_gps_cb.dev_req_packet.content.gps.course= gps_tracker_gps.course;
				//������ 
	 			gps_tracker_gps_cb.dev_req_packet.content.gps.reserv_satnum = gps_tracker_gps.sat_uesed;
	 			
				if(gps_tracker_gps.lat_ind == EN_GT_SOUTH)
				{
					gps_tracker_gps_cb.dev_req_packet.content.gps.property.lat_ind = 0;	
				}
				else if(gps_tracker_gps.lat_ind == EN_GT_NORTH)
				{
					gps_tracker_gps_cb.dev_req_packet.content.gps.property.lat_ind = 1;
				}

				if(gps_tracker_gps.long_ind == EN_GT_WEST)
				{
					gps_tracker_gps_cb.dev_req_packet.content.gps.property.long_ind = 0;	
				}
				else if(gps_tracker_gps.long_ind == EN_GT_EAST)
				{
					gps_tracker_gps_cb.dev_req_packet.content.gps.property.long_ind = 1;
				}

				if(gps_tracker_gps.mode == EN_GT_GM_A)
				{
					gps_tracker_gps_cb.dev_req_packet.content.gps.property.mode = 0;	
				}
				else if(gps_tracker_gps.mode == EN_GT_GM_D)
				{
					gps_tracker_gps_cb.dev_req_packet.content.gps.property.mode = 1;
				}
				else if(gps_tracker_gps.mode == EN_GT_GM_E)
				{
					gps_tracker_gps_cb.dev_req_packet.content.gps.property.mode = 2;
				}
				else if(gps_tracker_gps.mode == EN_GT_GM_N)
				{
					gps_tracker_gps_cb.dev_req_packet.content.gps.property.mode = 3;
				}

				//��ȡ��վ�ź�ǿ��				
				gps_tracker_gps_cb.dev_req_packet.content.gps.reserv_sigstren = gps_tracker_cell.sig_stren;
				
				//��ӻ�վ��Ϣ
				gps_tracker_gps_cb.dev_req_packet.content.gps.mcc = gps_tracker_cell.mcc;
				gps_tracker_gps_cb.dev_req_packet.content.gps.mnc = gps_tracker_cell.mnc;
				gps_tracker_gps_cb.dev_req_packet.content.gps.lac_sid = gps_tracker_cell.lac_sid;
				gps_tracker_gps_cb.dev_req_packet.content.gps.cellid_nid = gps_tracker_cell.cellid_nid;

				gps_tracker_gps_cb.is_updated = KAL_TRUE;

				//test
				gps_tracker_gps_cb.dev_req_packet.content.gps.bid = gps_tracker_shake_value;
				#ifdef __GPS_TRACE__			
				gps_tracker_trace(WARN, MOD_MMI, 
						"lat %u;long %u;speed %u;lac %u;cell_id %u", 
						gps_tracker_gps_cb.dev_req_packet.content.gps.latitude,
						gps_tracker_gps_cb.dev_req_packet.content.gps.longitude,
						gps_tracker_gps_cb.dev_req_packet.content.gps.speed,
						gps_tracker_gps_cb.dev_req_packet.content.gps.lac_sid,
						gps_tracker_gps_cb.dev_req_packet.content.gps.cellid_nid);
				#endif
				
				return KAL_SUCCESS;
			}

		}
		else//gps ��Ч
		{	
		#if 0	//zzt.20150915.del for ��ֹʱ����λ��
			//��ȡ��վ��Ϣ			
			if(gps_tracker_gps_cb.dev_req_packet.content.gps.lac_sid != gps_tracker_cell.lac_sid|| 
					gps_tracker_gps_cb.dev_req_packet.content.gps.cellid_nid != gps_tracker_cell.cellid_nid)
			{	
			#ifdef __GPS_TRACE__			
				gps_tracker_trace(WARN, MOD_MMI, 
						"########### cell info update.prevous lac %u cellid %u; current lac %u cellid %u", 
						gps_tracker_gps_cb.dev_req_packet.content.gps.lac_sid, 
						gps_tracker_gps_cb.dev_req_packet.content.gps.cellid_nid, 
						gps_tracker_gps_cb.dev_req_packet.content.gps.cellid_nid, gps_tracker_cell.cellid_nid);
			#endif

				//��λ���� : ��վ
				gps_tracker_gps_cb.dev_req_packet.content.gps.loc_type = EN_GT_LT_CELL;
	
				gps_tracker_gps_cb.dev_req_packet.content.gps.reserv_sigstren = gps_tracker_cell.sig_stren;
				gps_tracker_gps_cb.dev_req_packet.content.gps.mcc = gps_tracker_cell.mcc;
				gps_tracker_gps_cb.dev_req_packet.content.gps.mnc = gps_tracker_cell.mnc;
				gps_tracker_gps_cb.dev_req_packet.content.gps.lac_sid = gps_tracker_cell.lac_sid;
				gps_tracker_gps_cb.dev_req_packet.content.gps.cellid_nid = gps_tracker_cell.cellid_nid;
		
				gps_tracker_gps_cb.is_updated = KAL_TRUE;

			#ifdef __GPS_TRACE__			
				gps_tracker_trace(WARN, MOD_MMI, 
						"######### loc_type:%u;lat %u;long %u;speed %u;lac %u;cell_id %u", 
						gps_tracker_gps_cb.dev_req_packet.content.gps.loc_type,
						gps_tracker_gps_cb.dev_req_packet.content.gps.latitude,
						gps_tracker_gps_cb.dev_req_packet.content.gps.longitude,
						gps_tracker_gps_cb.dev_req_packet.content.gps.speed,
						gps_tracker_gps_cb.dev_req_packet.content.gps.lac_sid,
						gps_tracker_gps_cb.dev_req_packet.content.gps.cellid_nid);
			#endif
				return KAL_SUCCESS;
			}
		#endif
		}
	}	
	gps_tracker_gps_cb.is_updated = KAL_FALSE;

	return KAL_SUCCESS;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_update_status
 * DESCRIPTION
 *  ����gps����
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
kal_int32 gps_tracker_update_status(void)
{	
	kal_uint8 								bat_level;
	gps_tracker_status_req_content_struct 	status = {0};
	


	if(is_datetime_updated != KAL_TRUE)
	{
		return KAL_SUCCESS;
	}
		
	//״̬	
	status.oil_pwr_state = gps_tracker_state.oil_pwr_state;

	status.sos_state = gps_tracker_state.sos_state;

	//����𶯸澯
#ifdef _WIN32
	gps_tracker_shake_value = 20;
#endif				

	if(IsMyTimerExist(GPS_TRACKER_ALARM_IGNORE_VIBR_TIMER) != MMI_TRUE && gps_tracker_config.defence == EN_GT_SWT_ON 
		&& (gps_tracker_shake_value >= gps_tracker_config.vibr_thr))	
	{
		gps_tracker_vibr_alarm_counts++;
#ifdef __G_SENSOR_TRACE__
		gps_tracker_trace(INFO, MOD_MMI, "vibr alarm: shake value %u threshold %u vibr_alarm_counts %u,thresholde 0", 
			gps_tracker_shake_value, gps_tracker_config.vibr_thr, gps_tracker_vibr_alarm_counts);
#endif

		//����̽�⵽N���𶯣��Ÿ澯
		if(gps_tracker_vibr_alarm_counts > 0)
		{
			gps_tracker_vibr_alarm_counts = 0;
			gps_tracker_alarm.vibr_ind = 1;
			gps_tracker_alarm.vibr_value = gps_tracker_shake_value;
		}
	}
	else
	{
		gps_tracker_vibr_alarm_counts = 0;
		gps_tracker_alarm.vibr_ind = 0;
	}	

	//��ȡ��ѹ
	bat_level = srv_charbat_get_bat_level();
	status.volt_level = bat_level;
	//todo ��ѹ�澯
	if(bat_level < BATTERY_LEVEL_1)
	{
		gps_tracker_alarm.pwr_low_ind = 1;
		gps_tracker_alarm.pwr_level = bat_level;
	}
	else
	{
		gps_tracker_alarm.pwr_low_ind = 0;
		gps_tracker_alarm.pwr_level = 0;
	}
	
	//todo �¶ȼ��¶ȸ澯
	//��δʵ��
	status.temp = 25;

	//�ϵ�澯
	if(GPIO_ReadIO(DC_INPUT_DET_IO) ==  1)
	{
		gps_tracker_alarm.pwr_off_ind = 1;
	}
	else
	{
		gps_tracker_alarm.pwr_off_ind = 0;
	}
	
	
	if((status.oil_pwr_state != gps_tracker_status_cb.dev_req_packet.content.status.oil_pwr_state) ||
		(status.sos_state != gps_tracker_status_cb.dev_req_packet.content.status.sos_state) ||
		(status.volt_level != gps_tracker_status_cb.dev_req_packet.content.status.volt_level) ||
		(status.temp != gps_tracker_status_cb.dev_req_packet.content.status.temp))
	{
	#ifdef __G_SENSOR_TRACE__
		gps_tracker_trace(ERR, MOD_MMI, 
				"status is changed, uploading...");
	#endif
		
		gps_tracker_status_cb.dev_req_packet.content.status.oil_pwr_state = status.oil_pwr_state;
		gps_tracker_status_cb.dev_req_packet.content.status.sos_state = status.sos_state;
		gps_tracker_status_cb.dev_req_packet.content.status.volt_level = status.volt_level;
		gps_tracker_status_cb.dev_req_packet.content.status.temp = status.temp;
		
		gps_tracker_status_cb.is_updated = KAL_TRUE;
	}
	
	return KAL_SUCCESS;
}
#ifdef __GPS_CONTROL_CONNECT__ 
void gps_tracker_upload_control_data(void)
{
	U8 buffer[MAX_GT_SEND_LEN] = {0};
	kal_int32 ret;
	kal_uint8 len,i;
	
#ifdef __CONTROL_TRACE__	
	gps_tracker_trace(INFO, MOD_MMI,"gps_tracker_upload_control_data");	
#endif
	
	if(gps_tracker_recv_control_data.value_len > 0)
	{
		gps_tracker_control_cb.is_updated = KAL_TRUE;
		gps_tracker_control_cb.dev_req_packet.content.control.value_len = gps_tracker_recv_control_data.value_len;
		gps_tracker_control_cb.dev_req_packet.content.control.addr = gps_tracker_recv_control_data.addr;
		memcpy(gps_tracker_control_cb.dev_req_packet.content.control.value, gps_tracker_recv_control_data.value, gps_tracker_recv_control_data.value_len);
		
		if(gps_tracker_control_cb.is_updated)
		{
			len = gps_tracker_format_cb_to_buffer(&gps_tracker_control_cb, buffer, MAX_GT_SEND_LEN);

			ret = 0;
			if(gps_tracker_gsm_state == EN_GT_GSMS_WORKING)
			{
				//���ͽ��յ��Ŀ���������
			#ifdef __CONTROL_TRACE__	
				gps_tracker_trace(INFO, MOD_MMI,"send control packet, len=%d,buf=%s", len,buffer);	
				for(i=0; i<len; i++)
				{
					gps_tracker_trace(INFO, MOD_MMI,"send control packet, buffer[%d]=%x", i,buffer[i]);
				}
			#endif

				ret = soc_send(gps_tracker_soc.socketid, buffer, len, 0);					

				gps_tracker_send_failed_times++;
			}	
			else
			{
			#ifdef __CONTROL_TRACE__	
				gps_tracker_trace(ERR, MOD_MMI,"NOT WORKING,gps_tracker_gsm_state %u, give up the sending...", gps_tracker_gsm_state);
			#endif
			}
			gps_tracker_control_cb.is_responsed = KAL_FALSE;			
		}

	}
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_recv_data_from_uart
 * DESCRIPTION
 *  GPS�Ӵ��ڻ�ȡ����������
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
void gps_tracker_recv_data_from_uart(void)
{
	kal_uint8 i;

#ifdef __CONTROL_TRACE__	
	gps_tracker_trace(INFO,MOD_MMI,"gps_tracker_recv_data_from_uart");
#endif
	
	memset(&gps_tracker_recv_control_data, 0, sizeof(gps_tracker_recv_control_data));
	gps_tracker_recv_control_data.value_len = gps_get_data_from_control(gps_tracker_recv_control_data.value ,sizeof(gps_tracker_recv_control_data.value),&(gps_tracker_recv_control_data.addr));

//modify battery byte, gps read battery info
#ifdef __GPS_BAT_CONNECT__
	gps_tracker_recv_control_data.value[0] = battery_info.percent_capability;	
#endif

#ifdef __CONTROL_TRACE__	
	for(i=0; i<gps_tracker_recv_control_data.value_len; i++)
	{
		gps_tracker_trace(INFO,MOD_MMI,"recvBuf[%d]=%x,len=%d,addr=%x",i,gps_tracker_recv_control_data.value[i],gps_tracker_recv_control_data.value_len,gps_tracker_recv_control_data.addr);
	}	
#endif

	if(gps_tracker_control_recv_flag==1)
	{
		gps_tracker_upload_control_data();
		gps_tracker_control_recv_flag = 0;
	}
	
}
#endif
#ifdef __GPS_MCU_CONNECT__
void gps_tracker_init_mcu_io(void)
{
	GPIO_ModeSetup(MCU_START_READY,  0);
	GPIO_InitIO(0, MCU_START_READY);

	GPIO_ModeSetup(BAT_SCK, 0);
	GPIO_ModeSetup(BAT_SDA, 0);

	GPIO_InitIO(1, BAT_SCK);
	GPIO_InitIO(1, BAT_SDA);
}
extern void mma_delay(VUINT16 time);
void gps_tracker_send_data_to_mcu(GT_MCU_SEND_TYPE type, U8* data, U8 datalen)
{
	U8 checksum=0,i;

	checksum =GetCrc8(checksum, 0x5a);
	checksum =GetCrc8(checksum, type);
	MCU_IICWriteData(0x5a);
	MCU_IICWriteData(type);

	if(data==NULL)
	{
		for(i=0; i<6; i++)
		{
			checksum =GetCrc8(checksum, 0x00);
			MCU_IICWriteData(0x00);
		}
	}
	else
	{
		for(i=0; i<datalen; i++)
		{		
			checksum =GetCrc8(checksum, *(data+i));
			MCU_IICWriteData(*(data+i));
		}

		//�涨�������ȣ��糤�Ȳ�������6λ
		for(; i<6; i++)
		{
			checksum =GetCrc8(checksum, 0x00);
			MCU_IICWriteData(0x00);
		}
		
	}
#ifdef __MCU_TRACE__
	gps_tracker_trace(INFO,MOD_MMI,"checksum=%x,cmd=%d",checksum,type);
#endif

	MCU_IICWriteData(checksum);
	mma_delay(5160*3);
}
U8 gps_tracker_read_data_from_mcu(U8 *data)
{
	U8 readByte1,readByte2,i,checksum = 0,readchecksum;
	U8 dataTmp[6] = {0xff};

	readByte1 = MCU_IICReadData();
	readByte2 = MCU_IICReadData();
	
	checksum = GetCrc8(checksum, readByte1);
	checksum = GetCrc8(checksum, readByte2);
	for(i=0; i<6; i++)
	{
		dataTmp[i] = MCU_IICReadData();
		checksum = GetCrc8(checksum, dataTmp[i]);
		#ifdef __MCU_TRACE__
		gps_tracker_trace(INFO,MOD_MMI,"byte1=%x,byte2=%x,read_data dataTmp[%d]=%x",readByte1,readByte2,i,dataTmp[i]);
		#endif
	}
	readchecksum = MCU_IICReadData();
#ifdef __MCU_TRACE__
		gps_tracker_trace(INFO,MOD_MMI,"readchecksum=%x,checksum=%x",readchecksum,checksum);
#endif
	if(readchecksum != checksum)
	{
		return GT_MCU_REPLY_ERROR;
	}
	
	if(readByte1 == 0xa5)
	{
		switch(readByte2)
		{
			case GT_MCU_REPLY_OK:
				return GT_MCU_REPLY_OK;
			case GT_MCU_REPLY_ERROR:
				return GT_MCU_REPLY_ERROR;
			case GT_MCU_REPLY_BATTERY_STATUS:
			{
				if(data)
				{
					*data = dataTmp[0];		//�������ֽ�
					*(data+1) = dataTmp[1];	//�������ֽ�
				}
				return GT_MCU_REPLY_BATTERY_STATUS;
			}
			case GT_MCU_REPLY_FAULT_STATUS:
				if(data)
				{
					*(data+2) = dataTmp[0];	//����ֵ
				}
				return GT_MCU_REPLY_FAULT_STATUS;
			case GT_MCU_REPLY_LOCK_STATUS:
				if(data)
				{
					*(data+3) = dataTmp[0];	//������״̬
				}
				return GT_MCU_REPLY_LOCK_STATUS;
			case GT_MCU_REPLY_HALL_STATUS:	//���������
				if(data)
				{
					*(data+4) = dataTmp[0];	//��λ
					*(data+5) = dataTmp[1];
					*(data+6) = dataTmp[2];
					*(data+7) = dataTmp[3];	//��λ
				}
				return GT_MCU_REPLY_HALL_STATUS;
			default:
				return GT_MCU_REPLY_ERROR;
		}
	}
	else
	{
		return GT_MCU_REPLY_ERROR;
	}
}

void gps_tracker_communicate_mcu(GT_MCU_SEND_TYPE type, U8* writedata, U8 writedatalen, U8* readData)
{
	U8 result=0,index=0,len;
	U8 buffer[MAX_GT_SEND_LEN] = {0};

	if(gps_mcu_i2c_busy)	//if i2c busy, ignore this command
		return;

	gps_mcu_i2c_busy = 1;
	do
	{
		gps_tracker_send_data_to_mcu(type, writedata, writedatalen);
		result = gps_tracker_read_data_from_mcu(readData);
		gps_tracker_trace(INFO,MOD_MMI,"gps_tracker_communicate_mcu,index=%d",index);
		index++;
		if(index >3)
			break;
	}while(!result);	
	gps_mcu_i2c_busy = 0;

	gps_tracker_trace(INFO,MOD_MMI,"gps_tracker_communicate_mcu,type=%d,result=%d,index=%d",type,result,index);

	if(type==GT_MCU_SEND_BATTERY_STATUS ||type==GT_MCU_SEND_FAULT_STATUS ||type==GT_MCU_SEND_LOCK_STATUS||type==GT_MCU_SEND_KM_STATUS)
	{
		if(index>3 ||result==GT_MCU_REPLY_ERROR)
			gps_tracker_control_cb.is_updated = KAL_FALSE;
	}
	else if(/*type == GT_MCU_SEND_SEARCH ||*/ type==GT_MCU_SEND_LOCK ||type==GT_MCU_SEND_ALARM)
	{
		if(result == GT_MCU_REPLY_OK)
		{
			gps_tracker_send_mcu_data();
		}
		else if(result == GT_MCU_REPLY_ERROR)
		{
			
		}
	}

}
/*****************************************************************************
 * FUNCTION
 *  gps_tracker_upload_mcu_data
 * DESCRIPTION
 *  �������ǿ�����ȡ���ݣ�����ϴ���������
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
void gps_tracker_upload_mcu_data(void)
{
	kal_uint8 i,len;
	kal_uint16 voltage;
#if 1//def __MCU_TRACE__	
	gps_tracker_trace(INFO,MOD_MMI,"gps_tracker_upload_mcu_data");
#endif
	gps_tracker_control_cb.dev_req_packet.content.control.addr = 0x1c;
	gps_tracker_control_cb.dev_req_packet.content.control.value_len = 8;
	gps_tracker_control_cb.is_updated = KAL_TRUE;

	gps_tracker_communicate_mcu(GT_MCU_SEND_BATTERY_STATUS, NULL, 0, gps_tracker_control_cb.dev_req_packet.content.control.value);
	gps_tracker_communicate_mcu(GT_MCU_SEND_FAULT_STATUS, NULL, 0, gps_tracker_control_cb.dev_req_packet.content.control.value);
	gps_tracker_communicate_mcu(GT_MCU_SEND_LOCK_STATUS, NULL, 0, gps_tracker_control_cb.dev_req_packet.content.control.value);
	gps_tracker_communicate_mcu(GT_MCU_SEND_KM_STATUS, NULL, 0, gps_tracker_control_cb.dev_req_packet.content.control.value);

#if 1//def __MCU_TRACE__	
	for(i=0;i<8;i++)
	{
		gps_tracker_trace(INFO,MOD_MMI,"value[%d]=%x",i,gps_tracker_control_cb.dev_req_packet.content.control.value[i]);
	}
//	voltage = gps_tracker_control_cb.dev_req_packet.content.control.value[0] + gps_tracker_control_cb.dev_req_packet.content.control.value[1]<<8; 
#endif
}
/*****************************************************************************
 * FUNCTION
 *  gps_tracker_send_mcu_data
 * DESCRIPTION
 *  ���͵�Ƭ����ȡ�����ݸ�������
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
void gps_tracker_send_mcu_data(void)
{
	U32 ret;
	U8 len = 0;
	U8 buffer[MAX_GT_SEND_LEN] = {0};
		
	gps_tracker_control_cb.is_updated = KAL_FALSE;
	gps_tracker_upload_mcu_data(); 
	if(gps_tracker_control_cb.is_updated)
	{
		len = gps_tracker_format_cb_to_buffer(&gps_tracker_control_cb, buffer, MAX_GT_SEND_LEN);
		ret = 0;
		
		if(gps_tracker_gsm_state == EN_GT_GSMS_WORKING)
		{
			//������Ϣ
			ret = soc_send(gps_tracker_soc.socketid, buffer, len, 0);						
		#ifdef __GPRS_TRACE__		
			gps_tracker_trace(WARN, MOD_MMI,"send mcu packet prot_id=%u,sn=%u,ret=%u", buffer[5], *(U16*)&buffer[6],ret);
		#endif	
			gps_tracker_send_failed_times++;
		}	
		else
		{
		#ifdef __GPRS_TRACE__				
			gps_tracker_trace(ERR, MOD_MMI,"NOT WORKING,gps_tracker_gsm_state %u, give up the sending...", gps_tracker_gsm_state);
		#endif
		}
		gps_tracker_control_cb.is_responsed = KAL_FALSE;	
	}	
}
/*****************************************************************************
 * FUNCTION
 *  gps_tracker_update_mcu_data
 * DESCRIPTION
 *  �ӷ�������ȡ���ݣ����͸���Ƭ��
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
void gps_tracker_update_mcu_data(U8 value_len, U8* value_data)
{
	mcu_command_struct* mcu_cmd = (mcu_command_struct*)value_data;
	gps_tracker_trace(INFO,MOD_MMI,"gps_tracker_update_mcu_data,len=%d,type=%d,para=%d",value_len,mcu_cmd->type,mcu_cmd->para[0]);
	if(value_len != 0)
	{
		switch(mcu_cmd->type)
		{
			case 0x01:	//Ѱ��
				gps_tracker_communicate_mcu(GT_MCU_SEND_SEARCH, NULL, 0, NULL);
				break;
			case 0x02:	//����
				gps_tracker_communicate_mcu(GT_MCU_SEND_LOCK, mcu_cmd->para, 1, NULL);
				break;
			case 0x03:	//��������
				gps_tracker_communicate_mcu(GT_MCU_SEND_ALARM, mcu_cmd->para, 1, NULL);
				break;
			default:
				break;
		}	
	}
}
#endif

#ifdef __GPS_BEST_HDOP__
kal_uint8 gps_tracker_get_valid_and_best_gps_data(void)
{
	kal_uint8 i,index=0xff;
	kal_uint16 hdop=0;

	if(gps_tracker_gps_array[0].state == EN_GT_GS_A)
	{
		hdop = gps_tracker_gps_array[0].hdop;
		index = 0;
	}
	
	for(i=0; i<gps_index; i++)
	{
		if(gps_tracker_gps_array[i].hdop<hdop && gps_tracker_gps_array[i].state==EN_GT_GS_A)
		{
			hdop = gps_tracker_gps_array[i].hdop;
			index = i;
		}
	}
	gps_tracker_trace(INFO,MOD_MMI,"gps_tracker_get_valid_and_best_gps_data,index=%d,hdop=%d", index,hdop);
	return index;
}
#endif
/*****************************************************************************
 * FUNCTION
 *  gps_tracker_upload_timer_proc
 * DESCRIPTION
 *  �ϱ���ʱ���Ĵ�������ʵ��gps��status ���ݰ����ϴ�
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
void gps_tracker_upload_timer_proc(void)
{	
	U32 ret;
	U8 len = 0;
	U8 buffer[MAX_GT_SEND_LEN] = {0};
	U8 i = 0;
	t_rtc rtc;
	U8 ind;
	U8 shake_value = 0;
	kal_uint8 index;
	static U8 less_upload=0;	//zzt.20150825.add for beside lat&long upload
	static U8 mcu_upload_interval=0;
#ifdef __GPS_PACKAGE_BY_QUEUE__
	U8* p_upload_data;
#endif
	less_upload++;
	gps_tracker_trace(WARN, MOD_MMI,"############################gps_tracker_upload_timer_proc...,less_upload=%d",less_upload);	
	
	//��ȡʱ��

#ifdef __MTK_TARGET__
	RTC_GetTime_(&rtc);
#else
	RTC_GetTime(&rtc);
#endif
/////////////////////////////////////////////////////////////////
	
	//todo ��ȡgps����
#ifdef __MTK_TARGET__
#ifdef __GPS_BEST_HDOP__
	index = gps_tracker_get_valid_and_best_gps_data();
	memset(&gps_tracker_gps, 0, sizeof(gps_tracker_gps));
	if(index != 0xff)
	{
		memcpy(&gps_tracker_gps, &gps_tracker_gps_array[index], sizeof(gps_tracker_gps));
		memset(gps_tracker_gps_array, 0, sizeof(gps_tracker_gps_struct)*10);
	}
	gps_index = 0;
#else
	if(gps_tracker_gps_fresh.latitude !=0 && gps_tracker_gps_fresh.longitude != 0)
	{
		memcpy(&gps_tracker_gps, &gps_tracker_gps_fresh, sizeof(gps_tracker_gps));
	}
#endif	
	gps_tracker_shake_value = gps_tracker_shake_value_fresh;
	gps_tracker_shake_value_fresh = 0;
	
#endif
#ifdef __GPS_TRACE__
	gps_tracker_trace(ERR, MOD_MMI,"gps_tracker_gps:lat %u lng %u", gps_tracker_gps.latitude, gps_tracker_gps.longitude);
#endif

#ifdef _WIN32
	gps_tracker_gps.speed = 1000;
#endif
// ����ʱ����ֹҲ��Ҫ�ϱ���

	if(gps_tracker_gps.speed >= 200)/*2KM*/
	{
		gps_tracker_moving_state = EN_GT_MS_MOVING;
	}
	else
	{
		gps_tracker_moving_state = EN_GT_MS_MOTIONLESS;
	}

///////////////////////////////////////////////////////////////

#ifdef __GPRS_TRACE__	
	gps_tracker_trace(INFO, MOD_MMI, 
			"datetime %u/%u/%u %u:%u:%u", 
			rtc.rtc_year, rtc.rtc_mon, rtc.rtc_day, rtc.rtc_hour, rtc.rtc_min, rtc.rtc_sec);
#endif
#ifdef __G_SENSOR_TRACE__
	gps_tracker_trace(ERR, MOD_MMI, 
			"defence %u,alarm_switch 0x%0x, sms_alarm_switch 0x%0x", gps_tracker_config.defence,
			*(U8*)&gps_tracker_config.alarm_switch, *(U8*)&gps_tracker_config.sms_alarm_switch);
#endif
#ifdef __GPS_TRACE__
	gps_tracker_trace(ERR, MOD_MMI,"@@@@@@@@@@vibr change: %u, speed %u, moving_state %u", 
		gps_tracker_shake_value, gps_tracker_gps.speed,gps_tracker_moving_state);
#endif	
#ifdef __GPRS_TRACE__	
	gps_tracker_trace(ERR, MOD_MMI,"gps_tracker_send_failed_times %u;gps_tracker_gsm_state %u", 
		gps_tracker_send_failed_times, gps_tracker_gsm_state);
#endif
	//�жϷ���ʧ�ܴ����Ƿ񳬹������
	if(gps_tracker_send_failed_times >= MAX_GT_SEND_FAILED_TIMES)
	{
		if((IsMyTimerExist(GPS_TRACKER_CONN_TIMER) != KAL_TRUE) && (IsMyTimerExist(GPS_TRACKER_DNS_TIMER) != KAL_TRUE))
		{
		#ifdef __GPRS_TRACE__	
			gps_tracker_trace(WARN, MOD_MMI, "send failed %u times, restart conn timer", gps_tracker_send_failed_times);
		#endif
			gps_tracker_conn_times =0 ;
			is_need_dns = KAL_TRUE;//ÿһ�ζ���������һ��dns�Ļ��ᣬ��֤��ip������£���Ѹ������
		#ifdef __GPRS_TRACE__	
			gps_tracker_trace(WARN, MOD_MMI,"@@@@@@@@@@@@@@@@@@@@@@@@@@@XXX-1");	
		#endif
			gps_tracker_conn_timer_proc();
		}
		
		gps_tracker_send_failed_times = 0;
	}
	
	//�ռ�����֮ǰ����Ҫ�Ƚ���Ӧ��cb �ı�ʶ����0
	gps_tracker_gps_cb.is_updated = KAL_FALSE;
	gps_tracker_status_cb.is_updated = KAL_FALSE;
	gps_tracker_alarm_cb.is_updated = KAL_FALSE;

	//1. ����gps��Ϣ	
	gps_tracker_update_gps(); 

	for(i=0; i<gps_tracker_config.up_intv; i++)	//zzt.20150801
	{
		if(gps_tracker_shake_value_array[i] >= 5)
		{
			shake_value += 1;
		}
#ifdef __G_SENSOR_TRACE__		
	gps_tracker_trace(INFO, MOD_MMI,"gps_tracker_upload_timer_proc(),gps_tracker_shake_value[%d]=%d,shake_value=%d",i,gps_tracker_shake_value_array[i],shake_value);
#endif
	}
	memset(gps_tracker_shake_value_array, 0, sizeof(gps_tracker_shake_value_array));
	

	if(gps_tracker_gps_cb.is_updated && shake_value>=2)
	{
		len = gps_tracker_format_cb_to_buffer(&gps_tracker_gps_cb, buffer, MAX_GT_SEND_LEN);

		ret = 0;
#ifdef __GPS_PACKAGE_BY_QUEUE__
		p_upload_data = (U8*)get_ctrl_buffer(MAX_GT_SEND_LEN);
		memset(p_upload_data, 0, MAX_GT_SEND_LEN);
		memcpy(p_upload_data, buffer, MAX_GT_SEND_LEN);
		EnQueue(&gps_tracker_upload_queue, p_upload_data);
		
		p_upload_data = (U8*)QueueFront(&gps_tracker_upload_queue);
		memcpy(buffer ,p_upload_data, MAX_GT_SEND_LEN);
#endif
		if(gps_tracker_gsm_state == EN_GT_GSMS_WORKING)
		{
			//������Ϣ
			ret = soc_send(gps_tracker_soc.socketid, buffer, len, 0);						
		#ifdef __GPRS_TRACE__		
			gps_tracker_trace(WARN, MOD_MMI,"send gps packet prot_id %u sn %u", buffer[5], *(U16*)&buffer[6]);
		#endif
			
			gps_tracker_send_failed_times++;
		}	
		else
		{
		#ifdef __GPRS_TRACE__				
			gps_tracker_trace(ERR, MOD_MMI,"NOT WORKING,gps_tracker_gsm_state %u, give up the sending...", gps_tracker_gsm_state);
		#endif
		}
	#ifndef __GPS_PACKAGE_BY_QUEUE__		
		gps_tracker_gps_cb.is_responsed = KAL_FALSE;	
	#endif
	}

#ifdef __GPS_MCU_CONNECT__
	mcu_upload_interval++;
	//�����뵥Ƭ��������
	if(mcu_upload_interval >= 6)
	{
		mcu_upload_interval = 0;
		gps_tracker_send_mcu_data();
	}
#endif


	if(less_upload >= 3)
	{
		less_upload = 0;

		//2. ����״̬��Ϣ
		gps_tracker_update_status();

		if(gps_tracker_status_cb.is_updated)
		{
			len = gps_tracker_format_cb_to_buffer(&gps_tracker_status_cb, buffer, MAX_GT_SEND_LEN);

			ret = 0;
			if(gps_tracker_gsm_state == EN_GT_GSMS_WORKING)
			{
				//���͵�½��Ϣ
				ret = soc_send(gps_tracker_soc.socketid, buffer, len, 0);					
			#ifdef __GPRS_TRACE__
				gps_tracker_trace(WARN, MOD_MMI,"send status packet, prot_id %u sn %u", buffer[5], *(U16*)&buffer[6]);	
			#endif
				gps_tracker_send_failed_times++;
			}	
			else
			{
			#ifdef __GPRS_TRACE__		
				gps_tracker_trace(ERR, MOD_MMI,"NOT WORKING,gps_tracker_gsm_state %u, give up the sending...", gps_tracker_gsm_state);
			#endif
			}
			gps_tracker_status_cb.is_responsed = KAL_FALSE;			
		}	

		//3. ����澯��Ϣ
		//���ݸ澯���ش���澯��Ϣ	
		ind = gps_tracker_alarm.pwr_low_ind << EN_GT_AT_PWR_LOW | (gps_tracker_alarm.pwr_off_ind << EN_GT_AT_PWR_OFF) 
				|(gps_tracker_alarm.vibr_ind << EN_GT_AT_VIBR)|(gps_tracker_alarm.oil_pwr_ind << EN_GT_AT_OIL_PWR)
				|(gps_tracker_alarm.speed_ind << EN_GT_AT_SPEED);
		
		for(i = 0; i < EN_GT_AT_END; i++)
		{	
			//�жϵ�ǰ���͵ĸ澯�Ƿ���Ч 1) �澯���ؿ� 2) �ж�Ӧ�ĸ澯����
			//if((ind & (0x01<<i)) && (*(U8*)&gps_tracker_config.alarm_switch & (0x01<<i)))
			if(ind & (0x01<<i))
			{		
			#ifdef __GPRS_TRACE__
				gps_tracker_trace(ERR, MOD_MMI,"########## [alarm] alarm No:%d", i);
			#endif
				if(i == EN_GT_AT_VIBR)//�𶯸澯
				{
					if(gps_tracker_config.alarm_switch.vibr == EN_GT_SWT_ON)
					{					
						gps_tracker_alarm_cb.dev_req_packet.content.alarm.type = EN_GT_AT_VIBR;
						gps_tracker_alarm_cb.dev_req_packet.content.alarm.value_len = sizeof(gps_tracker_alarm.vibr_value);
						gps_tracker_alarm_cb.dev_req_packet.content.alarm.value.vibr_value = gps_tracker_alarm.vibr_value;
						gps_tracker_alarm_cb.is_updated = KAL_TRUE;
					}
	/*				
					//��ʽ���澯��Ϣ������澯���Ŷ���
					if(gps_tracker_config.sms_alarm_switch.vibr == EN_GT_SWT_ON)				
					{
						U8 tmp_buf[MAX_GT_SMS_CONTENT_LEN*2] = {0};
						U16 content[MAX_GT_SMS_CONTENT_LEN] = {0};
						
						if(gps_tracker_config.lang == EN_GT_LANG_EN)
						{
							kal_sprintf(tmp_buf, "<vibr alarm> vibr level %u  %u/%u/%u %u:%u:%u",gps_tracker_alarm.vibr_value,
								rtc.rtc_year, rtc.rtc_mon, rtc.rtc_day, rtc.rtc_hour, rtc.rtc_min, rtc.rtc_sec);
						}
						else
						{
							kal_sprintf((char*)tmp_buf, (char*)"<�𶯸澯> �𶯼��� %u  %u/%u/%u %u:%u:%u", gps_tracker_alarm.vibr_value, 
								rtc.rtc_year, rtc.rtc_mon, rtc.rtc_day, rtc.rtc_hour, rtc.rtc_min, rtc.rtc_sec);
						}

						app_asc_str_to_ucs2_wcs((kal_uint8 *)content, (kal_uint8 *)tmp_buf);

						gps_tracker_send_sms_to_all_users(content);
					}
	*/
					//���������Ѿ�������ϣ�������ݱ�־
					gps_tracker_alarm.vibr_ind = 0;
					gps_tracker_alarm.vibr_value = 0;
				}	
				else if(i == EN_GT_AT_SPEED)//���ٸ澯
				{
					if(gps_tracker_config.alarm_switch.speed == EN_GT_SWT_ON)	
					{					
						gps_tracker_alarm_cb.dev_req_packet.content.alarm.type = EN_GT_AT_SPEED;
						gps_tracker_alarm_cb.dev_req_packet.content.alarm.value_len = sizeof(gps_tracker_alarm.speed);
						gps_tracker_alarm_cb.dev_req_packet.content.alarm.value.speed = gps_tracker_alarm.speed;
						gps_tracker_alarm_cb.is_updated = KAL_TRUE;

			#ifdef __GPRS_TRACE__
				gps_tracker_trace(ERR, MOD_MMI,"speed alarm gps_tracker_alarm.speed=%d",gps_tracker_alarm.speed);
			#endif
					}
	/*				
					if(gps_tracker_config.sms_alarm_switch.speed == EN_GT_SWT_ON)				
					{
						//��ʽ���澯��Ϣ������澯���Ŷ���
						{
							U8 tmp_buf[MAX_GT_SMS_CONTENT_LEN*2] = {0};
							U16 content[MAX_GT_SMS_CONTENT_LEN] = {0};
							
							if(gps_tracker_config.lang == EN_GT_LANG_EN)
							{
								kal_sprintf(tmp_buf, "<speed alarm> speed %u  %u/%u/%u %u:%u:%u",gps_tracker_alarm.speed, 
									rtc.rtc_year, rtc.rtc_mon, rtc.rtc_day, rtc.rtc_hour, rtc.rtc_min, rtc.rtc_sec);
							}
							else
							{
								kal_sprintf((char*)tmp_buf, (char*)"<���ٸ澯> �ٶ� %u  %u/%u/%u %u:%u:%u", gps_tracker_alarm.speed,
									rtc.rtc_year, rtc.rtc_mon, rtc.rtc_day, rtc.rtc_hour, rtc.rtc_min, rtc.rtc_sec);
							}

							app_asc_str_to_ucs2_wcs((kal_uint8 *)content, (kal_uint8 *)tmp_buf);

							gps_tracker_send_sms_to_all_users(content);
						}
					}
	*/
					//���������Ѿ�������ϣ�������ݱ�־
					gps_tracker_alarm.speed_ind = 0;
					gps_tracker_alarm.speed = 0;
				}	
				else if(i == EN_GT_AT_PWR_LOW)//�͵�
				{
					if(gps_tracker_config.alarm_switch.pwr_low == EN_GT_SWT_ON)
					{					
						gps_tracker_alarm_cb.dev_req_packet.content.alarm.type = EN_GT_AT_PWR_LOW;
						gps_tracker_alarm_cb.dev_req_packet.content.alarm.value_len = sizeof(gps_tracker_alarm.pwr_level);
						gps_tracker_alarm_cb.dev_req_packet.content.alarm.value.volt_level = gps_tracker_alarm.pwr_level;
						gps_tracker_alarm_cb.is_updated = KAL_TRUE;
					}

					//���������Ѿ�������ϣ�������ݱ�־
					gps_tracker_alarm.pwr_low_ind = 0;
					gps_tracker_alarm.pwr_level = 0;
				}
				else if(i == EN_GT_AT_PWR_OFF)//�ϵ�
				{
					if(gps_tracker_config.alarm_switch.pwr_off== EN_GT_SWT_ON)
					{					
						gps_tracker_alarm_cb.dev_req_packet.content.alarm.type = EN_GT_AT_PWR_OFF;
						gps_tracker_alarm_cb.dev_req_packet.content.alarm.value_len = 0;
						gps_tracker_alarm_cb.is_updated = KAL_TRUE;
					}

					//���������Ѿ�������ϣ�������ݱ�־
					gps_tracker_alarm.pwr_off_ind= 0;
				}
				else if(i == EN_GT_AT_OIL_PWR)//���͵�
				{
					if(gps_tracker_config.alarm_switch.oil_pwr == EN_GT_SWT_ON)
					{					
						gps_tracker_alarm_cb.dev_req_packet.content.alarm.type = EN_GT_AT_OIL_PWR;
						gps_tracker_alarm_cb.dev_req_packet.content.alarm.value_len = 0;
						gps_tracker_alarm_cb.is_updated = KAL_TRUE;
					}

					//���������Ѿ�������ϣ�������ݱ�־
					gps_tracker_alarm.oil_pwr_ind= 0;
				}
				else
				{
					gps_tracker_alarm_cb.is_updated = KAL_FALSE;
				}
				
				if(gps_tracker_alarm_cb.is_updated)
				{
					//��ʽ���澯��Ϣ
					len = gps_tracker_format_cb_to_buffer(&gps_tracker_alarm_cb, buffer, MAX_GT_SEND_LEN);
					
					if(gps_tracker_gsm_state >= EN_GT_GSMS_WORKING)
					{
						//���͵�½��Ϣ
						ret = soc_send(gps_tracker_soc.socketid, buffer, len, 0);					
					#ifdef __GPRS_TRACE__
						gps_tracker_trace(INFO, MOD_MMI,"send alarm packet, prot_id %u sn %u", buffer[5], *(U16*)&buffer[6]);	
					#endif
						gps_tracker_send_failed_times++;
					}	
					
					gps_tracker_alarm_cb.is_responsed = KAL_FALSE;
				}	
			}		
		}	
	}

	//��������
	if(IsMyTimerExist(GPS_TRACKER_UPLOAD_TIMER) != MMI_TRUE)
	{
		StartTimer(GPS_TRACKER_UPLOAD_TIMER, gps_tracker_config.up_intv * 1000, 
					(FuncPtr)gps_tracker_upload_timer_proc);
	}
}

S32 gps_tracker_send_sms(U16* num, U16* content)
{
	sms_node_struct sms_node = {0};
	
	
	if(num == NULL || content == NULL )
	{
		return KAL_ERROR;
	}
	
	if(wcslen(num) == 0 || wcslen(content) == 0)
	{
		return KAL_ERROR;
	}

	memcpy(sms_node.number, num, sizeof(sms_node.number));
	memcpy(sms_node.content, content, sizeof(sms_node.content));

	sms_send_array_put_node(&sms_node);

	return KAL_SUCCESS;
}

S32 gps_tracker_send_sms_to_all_users(U16* content)
{
	U16 num[MAX_GT_PHONE_NUM_LEN] = {0};
	sms_node_struct sms_node = {0};
	U8 i;

	if(content == NULL )
	{
		return KAL_ERROR;
	}
	
	if(strlen(gps_tracker_config.admin_num) == 0 || wcslen(content) == 0)
	{
		return KAL_ERROR;
	}
	
	if(strlen(gps_tracker_config.admin_num) > 0)
	{
		mbstowcs(num, gps_tracker_config.admin_num, strlen(gps_tracker_config.admin_num));
		memcpy(sms_node.number, num, sizeof(sms_node.number));
		memcpy(sms_node.content, content, sizeof(sms_node.content));

		sms_send_array_put_node(&sms_node);
	}

	for (i =0; i < MAX_GT_USER_COUNT; i++)
	{
		if(strlen(gps_tracker_config.users[i]) > 0)
		{
			mbstowcs(num, gps_tracker_config.users[i], strlen(gps_tracker_config.users[i]));

			memcpy(sms_node.number, num, sizeof(sms_node.number));
			memcpy(sms_node.content, content, sizeof(sms_node.content));

			sms_send_array_put_node(&sms_node);
		}		
	}
	
	return KAL_SUCCESS;
}
/*****************************************************************************
 * FUNCTION
 *  gps_tracker_sms_send_timer_proc
 * DESCRIPTION
 *  �����ڼ����Ŷ��У����Ͷ���
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
void gps_tracker_sms_send_timer_proc(void)
{
	sms_node_struct sms_node = {0};

#ifdef __SMS_TRACE__
	gps_tracker_trace(WARN, MOD_MMI,"############ enter gps_tracker_sms_send_timer_proc()...");
#endif
	if(sms_send_array_get_node(&sms_node) == KAL_SUCCESS)
	{
		srv_sms_send_ucs2_text_msg((S8*)sms_node.content,
				(S8*)sms_node.number,
				SRV_SMS_SIM_1,
				NULL,
				NULL);
	}

	if(IsMyTimerExist(GPS_TRACKER_SMS_SEND_TIMER) != MMI_TRUE)
	{
		StartTimer(GPS_TRACKER_SMS_SEND_TIMER, gps_tracker_config.sms_send_intv*1000, 
					gps_tracker_sms_send_timer_proc);	
	}
}
/*****************************************************************************
 * FUNCTION
 *  gps_tracker_hb_timer_proc
 * DESCRIPTION
 *  �������̺���
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
void gps_tracker_hb_timer_proc(void)
{
	kal_int32 ret = 0;
	kal_uint8 len = 0;
	U8 buffer[MAX_GT_SEND_LEN] = {0};	
	static kal_int32 sendFail =0;
#ifdef __GPRS_TRACE__	
	gps_tracker_trace(WARN, MOD_MMI,"############################gps_tracker_hb_timer_proc...");		
#endif

	gps_tracker_hb_cb.is_responsed = KAL_FALSE; 


	//��ʽ����½��Ϣ
	len = gps_tracker_format_cb_to_buffer(&gps_tracker_hb_cb, buffer, MAX_GT_SEND_LEN);	

	//���͵�½��Ϣ
	ret = soc_send(gps_tracker_soc.socketid, buffer, len, 0);		
	if(ret != len)
	{
		sendFail++;
	#ifdef __GPRS_TRACE__		
		gps_tracker_trace(ERR, MOD_MMI,"send hb packet failed...");
	#endif
	}
	
	if(sendFail == 2)   // 2�η���ʧ�� ����ǿ������ϵͳ
	{
		sendFail = 0;
	#ifdef __DEL_CONNET_FAIL_RESTRART__	
		gps_tracker_conn_times =0 ;
	
		if(IsMyTimerExist(GPS_TRACKER_CONN_TIMER) != MMI_TRUE)
			gps_tracker_conn_timer_proc();	
		return;
	#else
		gps_tracker_restart();	
	#endif
	}

#ifdef __GPRS_TRACE__		
	gps_tracker_trace(ERR, MOD_MMI,"send hb...,gps_tracker_hb_failed_times=%u", gps_tracker_hb_failed_times);
#endif

	//����������
	gps_tracker_hb_failed_times++;
	//gps_tracker_send_failed_times++;	
	
	if(gps_tracker_hb_failed_times > 2)
	{		
		//����2������û�յ���ϵͳ����
	#ifndef __DEL_CONNET_FAIL_RESTRART__		
		RstStartRestore();
	#endif
		//soc_close(gps_tracker_soc.socketid);
		gps_tracker_login_cb.is_responsed = KAL_FALSE;	
		gps_tracker_hb_failed_times = 0;	

	#ifdef __DEL_CONNET_FAIL_RESTRART__		
		gps_tracker_conn_times =0 ;
		if(IsMyTimerExist(GPS_TRACKER_CONN_TIMER) != MMI_TRUE)
			gps_tracker_conn_timer_proc();	
		return;
	#endif

	}

	//������ʱ��
	if(IsMyTimerExist(GPS_TRACKER_HB_TIMER) != MMI_TRUE)
	{
		StartTimer(GPS_TRACKER_HB_TIMER, gps_tracker_config.hb_intv * 1000, 
					(FuncPtr)gps_tracker_hb_timer_proc);
	}
}
/*****************************************************************************
 * FUNCTION
 *  gps_tracker_get_dev_cfg_content_len
 * DESCRIPTION
 *  ��ʽ��cb�����ݵ�����buffer
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
S32 gps_tracker_get_dev_data_req_content(U8* buffer, U8 type, gps_tracker_data_content_struct* data)
{
	U8* buf;


	if(buffer == NULL || data == NULL)
		return KAL_ERROR;
	
	//1.�Ƚ����ͺͳ��ȸ�ʽ����buffer
	buffer[0] = data->type;
	buffer[1] = data->value_len;

	buf = buffer + 2;

	//2. ��value�ṹ���ʽ����buf
	switch(type)
	{
		case EN_GT_DT_ADMIN_NUM:
			strcpy(buf, data->value.admin_number);
			break;
		case EN_GT_DT_PWD:
			memcpy(buf, (const void*)&data->value.pwd, sizeof(data->value.pwd));
			break;
		case EN_GT_DT_USER:
			memcpy(buf, data->value.users, sizeof(data->value.users));
			break;
		case EN_GT_DT_UP_INTV:
			memcpy(buf, (const void*)&data->value.upload_intv, sizeof(data->value.upload_intv));
			break;
		case EN_GT_DT_HB_INTV:
			memcpy(buf, (const void*)&data->value.hb_intv, sizeof(data->value.hb_intv));
			break;
		case EN_GT_DT_SMS_ALARM_INTV:
			memcpy(buf, (const void*)&data->value.sms_send_intv, sizeof(data->value.sms_send_intv));
			break;
		case EN_GT_DT_TEMP_THR:
			memcpy(buf, (const void*)&data->value.temp_thr, sizeof(data->value.temp_thr));
			break;
		case EN_GT_DT_VIBR_THR:
			memcpy(buf, (const void*)&data->value.vibr_thr, sizeof(data->value.vibr_thr));
			break;
		case EN_GT_DT_SPEED_THR:
			memcpy(buf, (const void*)&data->value.speed_thr, sizeof(data->value.speed_thr));
			break;
		case EN_GT_DT_LANG:
			memcpy(buf, (const void*)&data->value.lang, sizeof(data->value.lang));
			break;
		case EN_GT_DT_TIME_ZONE:
			memcpy(buf, (const void*)&data->value.time_zone, sizeof(data->value.time_zone));
			break;		
		case EN_GT_DT_ALARM_SWITCH:
			memcpy(buf, (const void*)&data->value.alarm_switch, sizeof(data->value.alarm_switch));
			break;
		case EN_GT_DT_SMS_ALARM_SWITCH:
			memcpy(buf, (const void*)&data->value.sms_alarm_switch, sizeof(data->value.sms_alarm_switch));
			break;
		case EN_GT_DT_LOC:
			memcpy(buf, (const void*)&data->value.loc_lang_type, sizeof(data->value.loc_lang_type));
			break;
		case EN_GT_DT_IGNORE_ALARM:
			memcpy(buf, (const void*)&data->value.ignore_alarm, sizeof(data->value.ignore_alarm));
			break;
		case EN_GT_DT_DEFENCE:
			memcpy(buf, (const void*)&data->value.defence, sizeof(data->value.defence));
			break;
		case EN_GT_DT_SERVER:
			memcpy(buf, (const void*)&data->value.server, sizeof(data->value.server));
			break;
		case EN_GT_DT_APN:
			memcpy(buf, (const void*)&data->value.apn, sizeof(data->value.apn));
			break;
		case EN_GT_DT_SMS_CENTER:
			strcpy(buf, (const void*)data->value.sms_center_num);
			break;
		case EN_GT_DT_VER:
			strcpy(buf, (const void*)data->value.ver);
			break;
		default:
			break;
	}

	return KAL_SUCCESS;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_format_cb_to_buffer
 * DESCRIPTION
 *  ��ʽ��cb�����ݵ�����buffer
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
kal_uint8 gps_tracker_format_cb_to_buffer(gps_tracker_dev_req_struct* cb, U8* buffer, kal_uint8 len)
{

	U8* buf = buffer;
	U16* crc = NULL;
	U8 real_len = 0;
	t_rtc rtc;
	applib_time_struct currtime;

	if(cb == NULL || buffer == NULL)
		return 0;
	
	//1. ����ʱ��
#ifdef __MTK_TARGET__
	RTC_GetTime_(&rtc);	
//	applib_dt_get_rtc_time(&currtime);  //zzt.20150824.del ,this function can't get time
#else
	RTC_GetTime(&rtc);
#endif

#if 1
	if(gps_tracker_gps.state == EN_GT_GS_A)
	{
		currtime.nYear = gps_tracker_gps.datetime[0]+YEARFORMATE;
		currtime.nMonth = gps_tracker_gps.datetime[1];
		currtime.nDay= gps_tracker_gps.datetime[2];
		currtime.nHour= gps_tracker_gps.datetime[3];
		currtime.nMin = gps_tracker_gps.datetime[4];
		currtime.nSec = gps_tracker_gps.datetime[5];
		gps_tracker_datetime_offset(&currtime, gps_tracker_config.time_zone);
		cb->dev_req_packet.head.datetime[0] = currtime.nYear%100;
		cb->dev_req_packet.head.datetime[1] = currtime.nMonth;
		cb->dev_req_packet.head.datetime[2] = currtime.nDay;
		cb->dev_req_packet.head.datetime[3] = currtime.nHour;
		cb->dev_req_packet.head.datetime[4] = currtime.nMin;
		cb->dev_req_packet.head.datetime[5] = currtime.nSec;
#ifdef __GPRS_TRACE__
	gps_tracker_trace(INFO, MOD_MMI, "format datetime %u %u %u %u %u %u", 
			currtime.nYear,currtime.nMonth,currtime.nDay,currtime.nHour,currtime.nMin,currtime.nSec);
#endif
	}
	else
	{
		cb->dev_req_packet.head.datetime[0] = rtc.rtc_year;
		cb->dev_req_packet.head.datetime[1] = rtc.rtc_mon;
		cb->dev_req_packet.head.datetime[2] = rtc.rtc_day;
		cb->dev_req_packet.head.datetime[3] = rtc.rtc_hour;
		cb->dev_req_packet.head.datetime[4] = rtc.rtc_min;
		cb->dev_req_packet.head.datetime[5] = rtc.rtc_sec;
#ifdef __GPRS_TRACE__
	gps_tracker_trace(INFO, MOD_MMI, "format datetime %u %u %u %u %u %u", 
			rtc.rtc_year, rtc.rtc_mon, rtc.rtc_day, rtc.rtc_hour, rtc.rtc_min, rtc.rtc_sec);
#endif
	}
#else
#ifdef __GPRS_TRACE__
	gps_tracker_trace(INFO, MOD_MMI, 
			"format datetime %u %u %u %u %u %u", 
			rtc.rtc_year, rtc.rtc_mon, rtc.rtc_day, rtc.rtc_hour, rtc.rtc_min, rtc.rtc_sec);
#endif

	cb->dev_req_packet.head.datetime[0] = rtc.rtc_year;	
	cb->dev_req_packet.head.datetime[1] = rtc.rtc_mon;
	cb->dev_req_packet.head.datetime[2] = rtc.rtc_day;
	cb->dev_req_packet.head.datetime[3] = rtc.rtc_hour;
	cb->dev_req_packet.head.datetime[4] = rtc.rtc_min;
	cb->dev_req_packet.head.datetime[5] = rtc.rtc_sec;
#endif

	//2. �������ݰ���sn�ţ�ȫ�ֹ���һ����������ˮ����
	cb->dev_req_packet.head.sn = gps_tracker_get_sn();
	cb->sn = cb->dev_req_packet.head.sn;
	
	//3. pack_len
	switch(cb->prot_type)
	{
		case EN_GT_PT_LOGIN:			
			cb->dev_req_packet.head.pack_len = sizeof(cb->dev_req_packet.content.login);	
			break;
		case EN_GT_PT_GPS:
			cb->dev_req_packet.head.pack_len = sizeof(cb->dev_req_packet.content.gps);	
			break;
		case EN_GT_PT_STATUS:
			cb->dev_req_packet.head.pack_len = sizeof(cb->dev_req_packet.content.status);	
			break;
		case EN_GT_PT_HB:
			cb->dev_req_packet.head.pack_len = 0;	
			break;
		case EN_GT_PT_ALARM:
			cb->dev_req_packet.head.pack_len = cb->dev_req_packet.content.alarm.value_len +2;// 2= type+value_len	
			break;
		case EN_GT_PT_DEV_DATA:
			cb->dev_req_packet.head.pack_len = cb->dev_req_packet.content.data.value_len + 2;   // 2= type+value_len		
			break;
		#ifdef __PROTOCOL_CONTROL__	
		case EN_GT_PT_CONTROL: 	
			cb->dev_req_packet.head.pack_len = cb->dev_req_packet.content.control.value_len + 2; // 2= value_len+addr
			break;
		#endif	
		default:
		#ifdef __GPRS_TRACE__		
			gps_tracker_trace(ERR, MOD_MMI, 
				"invalid prot_id %u",cb->prot_type);
		#endif
			break;
	}	

	//4. �� head ��ʽ����buf
	memcpy(buf, &(cb->dev_req_packet.head), sizeof(gps_tracker_msg_head_struct));
	buf+=sizeof(gps_tracker_msg_head_struct);

	//5.�� content ��ʽ���� buf
	switch(cb->prot_type)
	{
		case EN_GT_PT_LOGIN:			
			memcpy(buf, &(cb->dev_req_packet.content.login), sizeof(gps_tracker_login_req_content_struct));
			buf+=sizeof(gps_tracker_login_req_content_struct);
			break;
		case EN_GT_PT_GPS:
			memcpy(buf, &(cb->dev_req_packet.content.gps), sizeof(gps_tracker_gps_req_content_struct));
			buf+=sizeof(gps_tracker_gps_req_content_struct);	
			break;
		case EN_GT_PT_STATUS:
			memcpy(buf, &(cb->dev_req_packet.content.status), sizeof(gps_tracker_status_req_content_struct));
			buf+=sizeof(gps_tracker_status_req_content_struct);
			break;
		case EN_GT_PT_HB:
			//no content body
			break;
		case EN_GT_PT_ALARM:
			memcpy(buf, &(cb->dev_req_packet.content.alarm), cb->dev_req_packet.head.pack_len);
			buf += cb->dev_req_packet.head.pack_len;
			break;
		case EN_GT_PT_DEV_DATA:					
			gps_tracker_get_dev_data_req_content(buf, cb->dev_req_packet.content.data.type, &cb->dev_req_packet.content.data);
			buf += cb->dev_req_packet.head.pack_len;
			break;
		#ifdef __PROTOCOL_CONTROL__	
		case EN_GT_PT_CONTROL: 	
			memcpy(buf, &(cb->dev_req_packet.content.control), cb->dev_req_packet.head.pack_len);
			buf += cb->dev_req_packet.head.pack_len;
			break;
		#endif	
		default:
		#ifdef __GPRS_TRACE__		
			gps_tracker_trace(ERR, MOD_MMI, 
				"invalid prot_id %u",cb->prot_type);
		#endif
			break;
	}

	//���tail
	memcpy(buf, &(cb->dev_req_packet.tail), sizeof(gps_tracker_msg_tail_struct));

	//����crc
	crc = (U16*)(buffer + 2);

	//ָ�������
	buf = buffer+4;	
			
	*crc = get_crc16(buf, cb->dev_req_packet.head.pack_len + 1+1+2+6);//����+��ͷ�ֶ�
	
	real_len = cb->dev_req_packet.head.pack_len + PACKET_FRAME_LEN;		

	//��buffer ����cb�У����ڱ��浽���ش洢�ļ�
	cb->dev_req_packet.packet_len = real_len;
	memcpy(cb->dev_req_packet.packet_buf, buffer, cb->dev_req_packet.packet_len);
	
	return real_len;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_login_timer_proc
 * DESCRIPTION
 *  ��½��������
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
void gps_tracker_login_timer_proc(void)
{


#ifdef __GPRS_TRACE__		
	gps_tracker_trace(WARN, MOD_MMI,"############################gps_tracker_login_timer_proc...");
#endif
	gps_tracker_gsm_state = EN_GT_GSMS_LOGIN;
	
	if(gps_tracker_login_times >= MAX_GT_LOGIN_TRY_TIMES)
	{
		//��½ʧ�����½���
		//@����ip����ʱ��Ҳ�ܲ���CONNECT ��Ϣ������login��ʧ�ܣ�����ʧ��ʱҪ����һ��DNS
		if(is_need_dns == KAL_TRUE)
		{
		#ifdef __GPRS_TRACE__		
			gps_tracker_trace(WARN, MOD_MMI,"login failed more than %u times��is_need_dns == KAL_TRUE,so we start dns check timer",MAX_GT_LOGIN_TRY_TIMES);
		#endif

			gps_tracker_gsm_state = EN_GT_GSMS_DNS;

			if(IsMyTimerExist(GPS_TRACKER_DNS_TIMER) != MMI_TRUE)
			{
				//StartTimer(GPS_TRACKER_DNS_TIMER, MAX_GT_DNS_CHECK_INTV, (FuncPtr)gps_tracker_dns_check_timer_proc);	
				gps_tracker_dns_check_timer_proc();
			}

			//�´ν���ʧ�ܣ��Ͳ���DNS��
			is_need_dns = KAL_FALSE;

			//�رյ�½��ʱ��
			if(IsMyTimerExist(GPS_TRACKER_LOGIN_TIMER) == KAL_TRUE)
			{
				StopTimer(GPS_TRACKER_LOGIN_TIMER);
			}

		}
		else
		{
		#ifdef __GPRS_TRACE__		
			gps_tracker_trace(WARN, MOD_MMI,"login failed %u times, reboot", gps_tracker_login_times);
		#endif
		#ifdef __DEL_CONNET_FAIL_RESTRART__	
			gps_tracker_conn_times =0 ;
			if(IsMyTimerExist(GPS_TRACKER_CONN_TIMER) != MMI_TRUE)
				gps_tracker_conn_timer_proc();	
		#else
			gps_tracker_restart();
		#endif
		}
	}
	else
	{		
		if(gps_tracker_login_cb.is_responsed != KAL_TRUE)
		{
			kal_int32 ret;
			kal_uint8 len;
			U8 buffer[MAX_GT_SEND_LEN] = {0};

			
			//��ʽ����½��Ϣ
			len = gps_tracker_format_cb_to_buffer(&gps_tracker_login_cb, buffer, MAX_GT_SEND_LEN);
			
			//���͵�½��Ϣ		
			ret = soc_send(gps_tracker_soc.socketid, buffer, len, 0);
			
			gps_tracker_login_times++;//���ܳɹ�����ʧ�ܣ�����������1���ɹ��յ��ظ�������
			gps_tracker_login_cb.is_responsed = KAL_FALSE;

			if(ret != len)
			{
			#ifdef __GPRS_TRACE__	
				gps_tracker_trace(WARN, MOD_MMI,"send login packet failed,ret=%d,len=%d",ret,len);
			#endif
			}

			#ifdef __GPRS_TRACE__	
			gps_tracker_trace(WARN, MOD_MMI,"########################## login %d times...", gps_tracker_login_times);
			#endif

			//������ʱ��
			if(IsMyTimerExist(GPS_TRACKER_LOGIN_TIMER) != MMI_TRUE)
			{
				StartTimer(GPS_TRACKER_LOGIN_TIMER, MAX_GT_LOGIN_INTV, 
						(FuncPtr)gps_tracker_login_timer_proc);
			}			
		}	
		else
		{
			//��½�ɹ������յ���Ӧ����½����
		}
	}	
}

/*****************************************************************************
 * FUNCTION
 *  srv_sms_get_host_by_name_ind
 * DESCRIPTION
 *  This function is used to handle MSG_ID_APP_SOC_GET_HOST_BY_NAME_IND from SOC.
 * PARAMETERS
 *  inMsg       [?]     
 * RETURNS
 *  void
 *****************************************************************************/
MMI_BOOL gps_tracker_get_host_by_name_ind(void *inMsg)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    app_soc_get_host_by_name_ind_struct *dns_ind;
	

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
#ifdef __GPRS_TRACE__		
	gps_tracker_trace(WARN, MOD_MMI,"gps_tracker_get_host_by_name_ind()...");
#endif
	
    if (!inMsg)
    {
    	#ifdef __GPRS_TRACE__	
       	gps_tracker_trace(ERR, MOD_MMI,"the inMsg ptr is null");
	#endif
        return MMI_FALSE;
    }
	
    dns_ind = (app_soc_get_host_by_name_ind_struct*) inMsg;

	
	if(dns_ind->request_id == EN_GT_RID_GPS)
	{
	    /* Check if the result is OK */
	    if (dns_ind->result == KAL_TRUE)
	    {    
	    	S16 error = NVRAM_READ_FAIL;

		    //�յ��ɹ��¼��󣬽�ip���浽ȫ�ֱ���
		    gps_tracker_soc.sockaddr.addr_len = dns_ind->addr_len;
			memcpy(gps_tracker_soc.sockaddr.addr, dns_ind->addr, dns_ind->addr_len);  			

			memcpy(gps_tracker_config.server.ip,gps_tracker_soc.sockaddr.addr, MAX_GT_IP_ADDR_LEN);
			gps_tracker_config.server.addr_type = EN_GT_ADT_IP;
	#ifdef __GPRS_TRACE__			
			gps_tracker_trace(WARN, MOD_MMI,"DNS succeed: %s,ip: %u.%u.%u.%u;port:%u;sock_type %u;addr_len %u", 			
					gps_tracker_config.server.domain, 
					gps_tracker_soc.sockaddr.addr[0], gps_tracker_soc.sockaddr.addr[1],
					gps_tracker_soc.sockaddr.addr[2], gps_tracker_soc.sockaddr.addr[3],
					gps_tracker_soc.sockaddr.port,
					gps_tracker_soc.sockaddr.sock_type,
					gps_tracker_soc.sockaddr.addr_len);
	#endif

			gps_tracker_dns_times = 0;

			//�����ݸ��½�nvram
			WriteRecord(NVRAM_EF_GT_SERVER_LID, 1, &gps_tracker_config.server, 
				sizeof(gps_tracker_config.server), &error);
			
			if (NVRAM_WRITE_SUCCESS != error)
			{
			#ifdef __GPRS_TRACE__	
				gps_tracker_trace(ERR, MOD_MMI,"write nvram record <server> failed!");	
			#endif

				return KAL_ERROR;
			}
			
			if(IsMyTimerExist(GPS_TRACKER_DNS_TIMER) == MMI_TRUE)
			{
				 StopTimer(GPS_TRACKER_DNS_TIMER);
			}

			if(IsMyTimerExist(GPS_TRACKER_CONN_TIMER) != MMI_TRUE)
			{
				//����soc ����
				gps_tracker_conn_times =0 ;
			#ifdef __GPRS_TRACE__					
				gps_tracker_trace(WARN, MOD_MMI,"@@@@@@@@@@@@@@@@@@@@@@@@@@@XXX-4");
			#endif
				gps_tracker_conn_timer_proc();
			}
			return MMI_TRUE;
	    }
	}
	return MMI_FALSE;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_get_host_by_name
 * DESCRIPTION
 *  ���� app_id acc_id  hostname ��ȡ hostname ��Ӧ�� ip ��ַ
 * PARAMETERS
 *  void
 * RETURNS
 *  void
 *****************************************************************************/
S32 gps_tracker_get_host_by_name(void)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    kal_int8 ret = SOC_ERROR;
	
    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
	#ifdef __GPRS_TRACE__	
	gps_tracker_trace(WARN, MOD_MMI,"gps_tracker_get_host_by_name()...");	
	#endif

	if(EN_GT_WS_GPS == gps_tracker_working_stage)
	{		
		ret = soc_gethostbyname(
	            KAL_FALSE,
	            MOD_MMI,
	            EN_GT_RID_GPS,
	            gps_tracker_config.server.domain,
	            gps_tracker_soc.sockaddr.addr,
	            (kal_uint8*)&gps_tracker_soc.sockaddr.addr_len,
	            0,
	            gps_tracker_app.acc_id);	
		
		if(ret!=SOC_SUCCESS)
		{
		#ifdef __GPRS_TRACE__	
			gps_tracker_trace(WARN, MOD_MMI,"%s(line:%d) ret=%d",__func__,__LINE__,ret);	
		#endif
		}

	    if (ret == SOC_SUCCESS)
	    {
			S16 error;

			//�յ��ɹ��¼��󣬽�ip���浽ȫ�ֱ���
			memcpy(gps_tracker_config.server.ip, gps_tracker_soc.sockaddr.addr, MAX_GT_IP_ADDR_LEN);
			gps_tracker_config.server.addr_type = EN_GT_ADT_IP;
		#ifdef __GPRS_TRACE__		
			gps_tracker_trace(WARN, MOD_MMI,"gps dns succeed, domain : %s, ip: %u.%u.%u.%u; port:%u; addr_type %u", 			
					gps_tracker_config.server.domain, 
					gps_tracker_config.server.ip[0], gps_tracker_config.server.ip[1],
					gps_tracker_config.server.ip[2], gps_tracker_config.server.ip[3],
					gps_tracker_config.server.port,
					gps_tracker_config.server.addr_type);
		#endif

			
			//�����ݸ��½�nvram
			WriteRecord(NVRAM_EF_GT_SERVER_LID, 1, &gps_tracker_config.server, 
				sizeof(gps_tracker_config.server), &error);
			
			if (NVRAM_WRITE_SUCCESS != error)
			{
			#ifdef __GPRS_TRACE__	
				gps_tracker_trace(ERR, MOD_MMI,"write nvram record <server> failed!");	
			#endif

				return KAL_ERROR;
			}
			
			if(IsMyTimerExist(GPS_TRACKER_DNS_TIMER) == MMI_TRUE)
			{
				 StopTimer(GPS_TRACKER_DNS_TIMER);
			}
			
			if(IsMyTimerExist(GPS_TRACKER_CONN_TIMER) != MMI_TRUE)
			{
				gps_tracker_conn_times =0 ;
			#ifdef __GPRS_TRACE__					
				gps_tracker_trace(WARN, MOD_MMI,"@@@@@@@@@@@@@@@@@@@@@@@@@@@XXX-6");
			#endif
				gps_tracker_conn_timer_proc();
			}
			
			gps_tracker_dns_times = 0;
			#ifdef __GPRS_TRACE__	
			gps_tracker_trace(WARN, MOD_MMI,"gps_tracker_get_host_by_name success!");
			#endif

			return KAL_SUCCESS;
	    }
	    else if (ret == SOC_WOULDBLOCK)
	    {
	        //waits for APP_SOC_GET_HOST_BY_NAME_IND
	        mmi_frm_set_protocol_event_handler(MSG_ID_APP_SOC_GET_HOST_BY_NAME_IND, 
	        		(PsIntFuncPtr)gps_tracker_get_host_by_name_ind, MMI_TRUE); 
	    }
	    else
	    {

			// ��������ʧ���Զ�������modif by zhangping 20150430
			gps_tracker_restart();
			
	        if (ret == SOC_ERROR)
	        {
	        	#ifdef __GPRS_TRACE__	
				gps_tracker_trace(ERR, MOD_MMI, "soc_gethostbyname: peer not reachable");
			#endif
	        }
	        else
	        {
	        #ifdef __GPRS_TRACE__
	            gps_tracker_trace(ERR, MOD_MMI, "soc_gethostbyname: socket error");
		 #endif
	        }
	    }

	}
	return ret;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_init_app_info
 * DESCRIPTION
 *  ��ȡapp ��Ϣ
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
void gps_tracker_get_app_info(void)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
	cbm_app_info_struct info;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
#ifdef __TCPIP__
			
	info.app_str_id = EN_SRV_GT_IDS_TEXT;
	info.app_icon_id = EN_SRV_GT_IDS_ICON;
	//info.app_type = DTCNT_APPTYPE_BRW_HTTP | DTCNT_APPTYPE_EMAIL | DTCNT_APPTYPE_VRE_NET;
	info.app_type = DTCNT_APPTYPE_EMAIL;
	cbm_register_app_id_with_app_info(&info, &gps_tracker_app.app_id); 

	//acct_id = cbm_set_app_id(CBM_DEFAULT_ACCT_ID, app_id);
	gps_tracker_app.acc_id = cbm_encode_data_account_id(CBM_DEFAULT_ACCT_ID, CBM_SIM_ID_SIM1, 
																	gps_tracker_app.app_id, KAL_FALSE);
   #ifdef __GPRS_TRACE__
	gps_tracker_trace(WARN, MOD_MMI,"acc_id", gps_tracker_app.acc_id);
   #endif
#endif
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_dns_check_timer_proc
 * DESCRIPTION
 *  �������dns�Ķ�ʱ��
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
void gps_tracker_dns_check_timer_proc(void)
{
#ifdef __GPRS_TRACE__
	gps_tracker_trace(WARN, MOD_MMI,"############################gps_tracker_dns_check_timer_proc...");
#endif
	//gps_tracker_get_host_by_name(void)();      
	//gps_tracker_dns_times++;
	
	if( gps_tracker_dns_times < MAX_GT_DNS_TRY_TIMES )
	{
		gps_tracker_get_host_by_name();      
		gps_tracker_dns_times++;
	
		if(IsMyTimerExist(GPS_TRACKER_DNS_TIMER) != MMI_TRUE /*&& IsMyTimerExist(GPS_TRACKER_CONN_TIMER) != MMI_TRUE*/)
		{
		#ifdef __GPRS_TRACE__
			gps_tracker_trace(INFO, MOD_MMI,"dns again...");
		#endif
			StartTimer(GPS_TRACKER_DNS_TIMER, MAX_GT_DNS_CHECK_INTV, 
							(FuncPtr)gps_tracker_dns_check_timer_proc); 
		}
	}
	//�ж�dns ���Դ��������С������ֵ����������ԣ��������������
	else
	{
		//dns ʧ�ܺ���������
	#ifdef __GPRS_TRACE__		
		gps_tracker_trace(WARN, MOD_MMI, "reset...");
	#endif
		gps_tracker_restart();	
	}	
		
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_server_conn
 * DESCRIPTION
 *  ����ϵͳ���ӷ���������ڣ��ɲ�ͬ�Ŀ����л�
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/

S32 gps_tracker_server_conn(void)
{	
#if 1	//zzt.20150716
	static U8 first_fullService = 1;
	static U8 search_time = 0;

	if(srv_nw_info_get_service_availability(MMI_SIM1) != SRV_NW_INFO_SA_FULL_SERVICE)
	{
	#ifdef __GPRS_TRACE__
		gps_tracker_trace(WARN, MOD_MMI,"signal_strength = %d,search_time=%d",srv_nw_info_get_signal_strength_in_percentage(MMI_SIM1),search_time);				
	#endif

		StartTimer(GPS_TRACKER_CHECK_NETWORK,1000,gps_tracker_server_conn);
		search_time++;
		if(search_time >= 45)
		{
			if(srv_sim_ctrl_get_unavailable_cause(MMI_SIM1) != SRV_SIM_CTRL_UA_CAUSE_NOT_INSERTED)
				gps_tracker_restart();	
		}
		return;
	}
	else
	{
	#ifdef __GPRS_TRACE__
		gps_tracker_trace(WARN, MOD_MMI,"Full Service,first_fullService=%d,search_time=%d",first_fullService,search_time);				
	#endif
		search_time = 0;
		if(first_fullService)	//Ϊ�˷�ֹ��һ��ȫ�������������socket���ܻ�ʧ�ܣ�������ʱ�ȴ��¡�
		{
			first_fullService = 0;
			StartTimer(GPS_TRACKER_CHECK_NETWORK,2000,gps_tracker_server_conn);
			return;
		}
	}
#endif	

	if(gps_tracker_working_stage == EN_GT_WS_GPS)
	{
		if(gps_tracker_config.server.addr_type == EN_GT_ADT_IP )
		{
			//�����ipģʽ��ֱ�ӽ���
			gps_tracker_gsm_state = EN_GT_GSMS_CONN;
			gps_tracker_conn_times =0 ;
		#ifdef __GPRS_TRACE__	
			gps_tracker_trace(WARN, MOD_MMI,"@@@@@@@@@@@@@@@@@@@@@@@@@@@XXX-8");
		#endif
			gps_tracker_conn_timer_proc();		
		}
		else if(gps_tracker_config.server.addr_type == EN_GT_ADT_DOMAIN )
		{
			//����ģʽ����ֱ�ӽ���dns����
			gps_tracker_gsm_state = EN_GT_GSMS_DNS;

			gps_tracker_dns_check_timer_proc();	
		}
	}
	return KAL_SUCCESS;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_server_init
 * DESCRIPTION
 *  ��ȡapp ��Ϣ
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
void gps_tracker_server_init(void)
{

    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
	cbm_app_info_struct info;
	U8 handle;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
	//1. ���������˺���Ϣ
	info.app_str_id = EN_SRV_GT_IDS_TEXT;
	info.app_icon_id = EN_SRV_GT_IDS_ICON;
	info.app_type = DTCNT_APPTYPE_EMAIL;
	cbm_register_app_id_with_app_info(&info, &gps_tracker_app.app_id); 	

	gps_tracker_app.acc_id = cbm_encode_data_account_id(CBM_DEFAULT_ACCT_ID, CBM_SIM_ID_SIM1, gps_tracker_app.app_id, KAL_FALSE);
	gps_tracker_trace(ERR, MOD_MMI,"gps_tracker_app.acc_id=%d",gps_tracker_app.acc_id);
	
	handle = L1SM_GetHandle();
	L1SM_SleepDisable(handle);
	
	gps_tracker_server_conn();

#ifdef __GPRS_TRACE__
	gps_tracker_trace(ERR, MOD_MMI,"init server success!");
#endif

}

void gps_tracker_get_imsi_rsp(void *info)
{
	char imsi_imei_num[17] ={0};      //������IMSI�ŵ��ַ����飺IMSI�Ų�����15λ������ģ�����ϻ�õ�16λ���ּ�һλ������	
	mmi_smu_get_imsi_rsp_struct *local_data = NULL;

	if(info == NULL)
		return;
	
	local_data = (mmi_smu_get_imsi_rsp_struct*) info;   //�õ�����Ϣ����
	gps_tracker_trace(INFO,MOD_MMI,"gps_tracker_get_imsi_rsp");     //��Catcher����ʾ��
	strcpy(imsi_imei_num,(char*)local_data->imsi);     //��SIM����������� 
	gps_tracker_trace(WARN, MOD_MMI,"imsi:%s", imsi_imei_num);
}

void gps_tracker_get_imsi_req(void)
{
    MYQUEUE Message;   
    SetProtocolEventHandler(gps_tracker_get_imsi_rsp, PRT_GET_IMSI_RSP);
    Message.oslSrcId = MOD_MMI;
    Message.oslDestId = MOD_L4C; 
    Message.oslMsgId = PRT_GET_IMSI_REQ;
    Message.oslDataPtr = NULL;
    Message.oslPeerBuffPtr = NULL;
    OslMsgSendExtQueue(&Message);
    kal_prompt_trace(MOD_MMI,"gps_tracker_get_imsi_req");
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_config_init
 * DESCRIPTION
 *  ��ȡ�豸��ʼ������
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
kal_uint32 gps_tracker_config_init(void)
{
	S16 error = NVRAM_READ_FAIL;


	//��ʼ��ȫ�ֱ���
	memset(&gps_tracker_config, 0, sizeof(gps_tracker_config_struct));


	
	//ver
	ReadRecord(NVRAM_EF_GT_VER_LID, 1, gps_tracker_config.ver, 
		sizeof(gps_tracker_config.ver), &error);
		
	if (NVRAM_READ_SUCCESS != error)
	{
	#ifdef __GPRS_TRACE__
		gps_tracker_trace(ERR, MOD_MMI,"read nvram record <ver> failed!"); 
	#endif

		return KAL_ERROR;
	}
	
	//dev_type
	ReadRecord(NVRAM_EF_GT_DEV_TYPE_LID, 1, &gps_tracker_config.dev_type, 
		sizeof(gps_tracker_config.dev_type), &error);
		
	if (NVRAM_READ_SUCCESS != error)
	{
	#ifdef __GPRS_TRACE__	
		gps_tracker_trace(ERR, MOD_MMI,"read nvram record <dev_type> failed!");	
	#endif
		return KAL_ERROR;
	}

	//����豸����Ϊ0 ��˵��ϵͳ��һ�����У�nvramΪ��
	if(gps_tracker_config.dev_type == 0)
	{
		gps_tracker_nvram_restore();

	#ifdef __GPRS_TRACE__	
		gps_tracker_trace(ERR, MOD_MMI,"restore...reset...");	
	#endif
#ifndef _WIN32	
		gps_tracker_restart();
		return KAL_SUCCESS;
#endif
	}

	//admin_number
	ReadRecord(NVRAM_EF_GT_ADM_NUM_LID, 1, gps_tracker_config.admin_num, 
			sizeof(gps_tracker_config.admin_num), &error);
		
	if (NVRAM_READ_SUCCESS != error)
	{
		#ifdef __GPRS_TRACE__	
		gps_tracker_trace(ERR, MOD_MMI,"read nvram record <admin_num> failed!");	
		#endif

		return KAL_ERROR;
	}	

	//����
	ReadRecord(NVRAM_EF_GT_PWD_LID, 1, &gps_tracker_config.pwd, 
				sizeof(gps_tracker_config.pwd), &error);
			
	if (NVRAM_READ_SUCCESS != error)
	{
	#ifdef __GPRS_TRACE__	
		gps_tracker_trace(ERR, MOD_MMI,"read nvram record <pwd> failed!"); 
	#endif
		return KAL_ERROR;
	}	
	
	//users	
	ReadRecord(NVRAM_EF_GT_USERS_LID, 1, gps_tracker_config.users, 
		sizeof(gps_tracker_config.users), &error);
	
	if (NVRAM_READ_SUCCESS != error)
	{
	#ifdef __GPRS_TRACE__		
		gps_tracker_trace(ERR, MOD_MMI,"read nvram record <users> failed!");	
	#endif

		return KAL_ERROR;
	}

	//upload_intv
	ReadRecord(NVRAM_EF_GT_UP_INTV_LID, 1, &gps_tracker_config.up_intv, 
		sizeof(gps_tracker_config.up_intv), &error);
	
	if (NVRAM_READ_SUCCESS != error)
	{
		#ifdef __GPRS_TRACE__	
		gps_tracker_trace(ERR, MOD_MMI,"read nvram record <up-intv> failed!");	
		#endif

		return KAL_ERROR;
	}

	//hb intv
	ReadRecord(NVRAM_EF_GT_HB_INTV_LID, 1, &gps_tracker_config.hb_intv, 
		sizeof(gps_tracker_config.hb_intv), &error);
	
	if (NVRAM_READ_SUCCESS != error)
	{
		#ifdef __GPRS_TRACE__	
		gps_tracker_trace(ERR, MOD_MMI,"read nvram record <hb-intv> failed!");	
		#endif
		
		return KAL_ERROR;
	}

	#ifdef __GPRS_TRACE__	
	gps_tracker_trace(ERR, MOD_MMI,"@@@@@@@@@@@@@@@@@@@<hb-intv> : %u", gps_tracker_config.hb_intv);	
	#endif
	
	//alarm intv
	ReadRecord(NVRAM_EF_GT_SMS_INTV_LID, 1, &gps_tracker_config.sms_send_intv, 
		sizeof(gps_tracker_config.sms_send_intv), &error);
	
	if (NVRAM_READ_SUCCESS != error)
	{
		#ifdef __GPRS_TRACE__	
		gps_tracker_trace(ERR, MOD_MMI,"read nvram record <sms-intv> failed!");	
		#endif
		return KAL_ERROR;
	}

	//v1.1
	gps_tracker_config.sms_send_intv = 10;

	ReadRecord(NVRAM_EF_GT_TEMP_THR_LID, 1, &gps_tracker_config.temp_thr, 
		sizeof(gps_tracker_config.temp_thr), &error);
	if (NVRAM_READ_SUCCESS != error)
	{
	#ifdef __GPRS_TRACE__	
		gps_tracker_trace(ERR, MOD_MMI,"read nvram record <temp-thr> failed!");	
	#endif
		return KAL_ERROR;
	}
	
	ReadRecord(NVRAM_EF_GT_VIBR_THR_LID, 1, &gps_tracker_config.vibr_thr, 
		sizeof(gps_tracker_config.vibr_thr), &error);
	if (NVRAM_READ_SUCCESS != error)
	{
	#ifdef __GPRS_TRACE__	
		gps_tracker_trace(ERR, MOD_MMI,"read nvram record <vibr-threshold> failed!");	
	#endif
		return KAL_ERROR;
	}	
	
	ReadRecord(NVRAM_EF_GT_SPEED_THR_LID, 1, &gps_tracker_config.speed_thr, 
		sizeof(gps_tracker_config.speed_thr), &error);
	if (NVRAM_READ_SUCCESS != error)
	{
	#ifdef __GPRS_TRACE__		
		gps_tracker_trace(ERR, MOD_MMI,"read nvram record <speed-thr> failed!");	
	#endif

		return KAL_ERROR;
	}

	ReadRecord(NVRAM_EF_GT_LANG_LID, 1, &gps_tracker_config.lang, 
		sizeof(gps_tracker_config.lang), &error);

	if (NVRAM_READ_SUCCESS != error)
	{
	#ifdef __GPRS_TRACE__	
		gps_tracker_trace(ERR, MOD_MMI,"read nvram record <lang> failed!");	
	#endif

		return KAL_ERROR;
	}

	ReadRecord(NVRAM_EF_GT_TIME_ZONE_LID, 1, &gps_tracker_config.time_zone, 
		sizeof(gps_tracker_config.time_zone), &error);

	if (NVRAM_READ_SUCCESS != error)
	{
		#ifdef __GPRS_TRACE__	
		gps_tracker_trace(ERR, MOD_MMI,"read nvram record <time_zone> failed!");	
		#endif

		return KAL_ERROR;
	}
#ifdef __GPRS_TRACE__	
	gps_tracker_trace(ERR, MOD_MMI,"@@@@@@@@@@@@@@@@@@@<time_zone> : %d", gps_tracker_config.time_zone);	
#endif
	
	ReadRecord(NVRAM_EF_GT_ALARM_SWITCH_LID, 1, &gps_tracker_config.alarm_switch, 
		sizeof(gps_tracker_config.alarm_switch), &error);

	if (NVRAM_READ_SUCCESS != error)
	{
#ifdef __GPRS_TRACE__		
		gps_tracker_trace(ERR, MOD_MMI,"read nvram record <alarm_switch> failed!");	
#endif
		return KAL_ERROR;
	}

	ReadRecord(NVRAM_EF_GT_SMS_SWITCH_LID, 1, &gps_tracker_config.sms_alarm_switch, 
		sizeof(gps_tracker_config.sms_alarm_switch), &error);

	if (NVRAM_READ_SUCCESS != error)
	{
#ifdef __GPRS_TRACE__			
		gps_tracker_trace(ERR, MOD_MMI,"read nvram record <sms_alarm_switch> failed!");	
#endif
		return KAL_ERROR;
	}

	ReadRecord(NVRAM_EF_GT_SERVER_LID, 1, &gps_tracker_config.server, 
		sizeof(gps_tracker_config.server), &error);

	if (NVRAM_READ_SUCCESS != error)
	{
#ifdef __GPRS_TRACE__			
		gps_tracker_trace(ERR, MOD_MMI,"read nvram record <server> failed!");	
#endif
		return KAL_ERROR;
	}
#ifdef __GPRS_TRACE__				
	gps_tracker_trace(WARN, MOD_MMI,"server:%d %s %d.%d.%d.%d %d", gps_tracker_config.server.addr_type,gps_tracker_config.server.domain,
		gps_tracker_config.server.ip[0],gps_tracker_config.server.ip[1],
		gps_tracker_config.server.ip[2],gps_tracker_config.server.ip[3],
		gps_tracker_config.server.port);
#endif
	
	ReadRecord(NVRAM_EF_GT_APN_LID, 1, &gps_tracker_config.apn, 
		sizeof(gps_tracker_config.apn), &error);

	if (NVRAM_READ_SUCCESS != error)
	{
#ifdef __GPRS_TRACE__				
		gps_tracker_trace(ERR, MOD_MMI,"read nvram record <apn> failed!");	
#endif
		return KAL_ERROR;
	}

#ifdef __GPRS_TRACE__				
	gps_tracker_trace(WARN, MOD_MMI,"apn: %s %s %s %d.%d.%d.%d %d %s", gps_tracker_config.apn.apn_name, 
		gps_tracker_config.apn.user_name, gps_tracker_config.apn.passwd,  
		gps_tracker_config.apn.px_addr[0], gps_tracker_config.apn.px_addr[1], 
		gps_tracker_config.apn.px_addr[2], gps_tracker_config.apn.px_addr[3], 
	 	gps_tracker_config.apn.px_port, gps_tracker_config.apn.user_data);
#endif
	//////////////////////////////////////////////////////////////////////////////////////////


	ReadRecord(NVRAM_EF_GT_DEFENCE_LID, 1, &gps_tracker_config.defence, 
		sizeof(gps_tracker_config.defence), &error);

	if (NVRAM_READ_SUCCESS != error)
	{
#ifdef __GPRS_TRACE__					
		gps_tracker_trace(ERR, MOD_MMI,"read nvram record <defence> failed!");	
#endif
		return KAL_ERROR;
	}

	ReadRecord(NVRAM_EF_GT_SMS_CENTER_NUM_LID, 1, gps_tracker_config.sms_center_num, 
		sizeof(gps_tracker_config.sms_center_num), &error);

	if (NVRAM_READ_SUCCESS != error)
	{
#ifdef __GPRS_TRACE__					
		gps_tracker_trace(ERR, MOD_MMI,"read nvram record <sms-center> failed!");	
#endif
		return KAL_ERROR;
	}

	ReadRecord(NVRAM_EF_GT_IMSI_LID, 1, gps_tracker_config.imsi, 
		sizeof(gps_tracker_config.imsi), &error);

	if (NVRAM_READ_SUCCESS != error)
	{
#ifdef __GPRS_TRACE__					
		gps_tracker_trace(ERR, MOD_MMI,"read nvram record <imsi> failed!");	
#endif
		return KAL_ERROR;
	}

	return KAL_SUCCESS;
}


/*****************************************************************************
 * FUNCTION
 *  gps_tracker_establish_conn
 * DESCRIPTION
 *  ��ʼ�� tracker �� ��ʱ��
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  kal_uint32 ������
 *****************************************************************************/
S32 gps_tracker_establish_conn(void)
{
	kal_int32 ret = 0;
	kal_uint8 val;
		
#ifdef __GPRS_TRACE__							
	gps_tracker_trace(WARN, MOD_MMI,
			"gps_tracker_establish_conn: soc_id %u,gps_tracker_working_stage=%d", gps_tracker_soc.socketid,gps_tracker_working_stage);
#endif

	if(EN_GT_WS_GPS == gps_tracker_working_stage)
	{
		soc_close(gps_tracker_soc.socketid);
		
		gps_tracker_soc.socketid = soc_create(SOC_PF_INET, SOC_SOCK_STREAM, 0, MOD_MMI,  gps_tracker_app.acc_id);
		if(gps_tracker_soc.socketid < 0)
		{
		#ifdef __GPRS_TRACE__					
			gps_tracker_trace(ERR, MOD_MMI,
					"soc_create failed");
		#endif
			return KAL_ERROR;
		}
		#ifdef __GPRS_TRACE__							
		gps_tracker_trace(WARN, MOD_MMI,
					"GPS: soc_id %u", gps_tracker_soc.socketid);
		#endif
		
		val = KAL_TRUE;
		
		ret = soc_setsockopt(gps_tracker_soc.socketid, SOC_NBIO, &val, sizeof(val));
		if(SOC_SUCCESS != ret)
		{
		#ifdef __GPRS_TRACE__									
			gps_tracker_trace(ERR, MOD_MMI,
							"soc_setsockopt failed");
		#endif
			return KAL_ERROR;
		}
		
		val = SOC_CONNECT | SOC_READ | SOC_WRITE | SOC_CLOSE;//SOC_READ | SOC_WRITE | SOC_ACCEPT
		ret = soc_setsockopt(gps_tracker_soc.socketid, SOC_ASYNC, &val, sizeof(val));	

		if(SOC_SUCCESS != ret)
		{
		#ifdef __GPRS_TRACE__							
			gps_tracker_trace(ERR, MOD_MMI,
							"soc_setsockopt failed");
		#endif
			return KAL_ERROR;
		}	
		
		//ָ�������¼��Ļص�����
		SetProtocolEventHandler(gps_tracker_conn_notify, MSG_ID_APP_SOC_NOTIFY_IND);
		#ifdef __GPRS_TRACE__							
		gps_tracker_trace(WARN, MOD_MMI,
				"soc_connect()...conn ip %u.%u.%u.%u port:%u;sock_type %u;addr_len %u", 
				gps_tracker_soc.sockaddr.addr[0], gps_tracker_soc.sockaddr.addr[1],
				gps_tracker_soc.sockaddr.addr[2], gps_tracker_soc.sockaddr.addr[3], 
				gps_tracker_soc.sockaddr.port, gps_tracker_soc.sockaddr.sock_type,
				gps_tracker_soc.sockaddr.addr_len);
		#endif
		ret = soc_connect(gps_tracker_soc.socketid, &gps_tracker_soc.sockaddr);
	}
	return KAL_SUCCESS;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_files_init
 * DESCRIPTION
 *  ��ʼ�� log �ļ�
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
void gps_tracker_files_init(void)
{
	S32 size = 0;	
	kal_uint32 file_len = 0;
	
	//ϵͳ��־
	//���Ӿ���·��Ĭ���Ǵ�����c�̸�Ŀ¼��
	gps_tracker_local_write_file_hdl = FS_Open(L"gps_local_write", FS_CREATE|FS_READ_WRITE);

#ifdef __GPRS_TRACE__
	gps_tracker_trace(ERR, MOD_MMI,
		"gps_tracker_local_write_file_hdl = %d", gps_tracker_local_write_file_hdl);
#endif
	FS_Seek(gps_tracker_local_write_file_hdl, 0, FS_FILE_END);
#ifdef __GPRS_TRACE__
	gps_tracker_trace(ERR, MOD_MMI,"init files success!");
#endif
}
/*****************************************************************************
 * FUNCTION
 *  gps_tracker_conn_timer_proc
 * DESCRIPTION
 *  ���Ӷ�ʱ�����̴���
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
void gps_tracker_conn_timer_proc(void)
{
#ifdef __GPRS_TRACE__
	gps_tracker_trace(WARN, MOD_MMI,"############################gps_tracker_conn_timer_proc...");
#endif
#ifdef __LED_INDICATE_STATE__
	if(gps_tracker_gsm_led)
	gps_tracker_close_gsm_work_mode();
#endif
	gps_tracker_gsm_state = EN_GT_GSMS_CONN;

	if( gps_tracker_conn_times <= MAX_GT_CONN_TRY_TIMES)
	{
		if( gps_tracker_conn_times >= 3 )
		{				
			//ip ���� ʧ�ܺ����³���dns
			if( is_need_dns == KAL_TRUE )
			{
			#ifdef __GPRS_TRACE__
				gps_tracker_trace(WARN, MOD_MMI,"conn failed %u times��trying dns...",gps_tracker_conn_times);				
			#endif

				//�ر����Ӷ�ʱ��
				if(IsMyTimerExist(GPS_TRACKER_CONN_TIMER) == KAL_TRUE)
				{
					StopTimer(GPS_TRACKER_CONN_TIMER);
				}
				
				//�´ν���ʧ�ܣ��Ͳ���DNS��
				is_need_dns = KAL_FALSE;

				gps_tracker_gsm_state = EN_GT_GSMS_DNS;

				if(IsMyTimerExist(GPS_TRACKER_DNS_TIMER) != MMI_TRUE)
				{
					gps_tracker_conn_times = 0;				

					gps_tracker_dns_check_timer_proc();
				}				
			}
			else if(gps_tracker_working_stage == EN_GT_WS_GPS)
			{
			#ifdef __GPRS_TRACE__
				gps_tracker_trace(WARN, MOD_MMI,
						"conn  %u times...",gps_tracker_conn_times);
			#endif
				
				gps_tracker_establish_conn();
				gps_tracker_conn_times++;				
					
				//������ʱ���ٴ�����
				if(IsMyTimerExist(GPS_TRACKER_CONN_TIMER) != MMI_TRUE)
				{
				#ifdef __GPRS_TRACE__
					gps_tracker_trace(WARN, MOD_MMI,"@@@@@@@@@@@@@@@@@@@@@@@@@@@XXX-10");
				#endif
					StartTimer(GPS_TRACKER_CONN_TIMER, MAX_GT_CONN_INTV, 
						(FuncPtr)gps_tracker_conn_timer_proc);	
				}
			}
				
		}		
		else if(IsMyTimerExist(GPS_TRACKER_CONN_TIMER) != MMI_TRUE)
		{
		#ifdef __GPRS_TRACE__
			gps_tracker_trace(WARN, MOD_MMI,
					"conn  %u times...",gps_tracker_conn_times);
		#endif

			gps_tracker_establish_conn(); 
			gps_tracker_conn_times++;				
						
			//������ʱ���ٴ�����
		#ifdef __GPRS_TRACE__
			gps_tracker_trace(WARN, MOD_MMI,"@@@@@@@@@@@@@@@@@@@@@@@@@@@XXX-11");
		#endif
			StartTimer(GPS_TRACKER_CONN_TIMER, MAX_GT_CONN_INTV, 
				(FuncPtr)gps_tracker_conn_timer_proc);			
		}
	}
	else
	{		
		//�������ӳ���������������
	#ifdef __GPRS_TRACE__
		gps_tracker_trace(WARN, MOD_MMI,
				"conn failed %d times,reboot...",MAX_GT_CONN_TRY_TIMES);
	#endif
	
#if 0//def __DEL_CONNET_FAIL_RESTRART__		//zzt.20150928.del
		gps_tracker_conn_times = 0;
		if(IsMyTimerExist(GPS_TRACKER_CONN_TIMER) != MMI_TRUE)
			gps_tracker_conn_timer_proc();
#else
		gps_tracker_restart();
#endif
	}
}


/*****************************************************************************
 * FUNCTION
 *  gps_tracker_get_nmea
 * DESCRIPTION
 *  ��ʼ�� ϵͳ io
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
kal_int32 gps_tracker_get_nmea(U8* buf, U8 key, U8* nmea)
{
	U8* head, *tail;


	if ( buf == NULL || nmea == NULL)
		return KAL_ERROR;
#ifdef __GPS_TRACE__		
	gps_tracker_trace(INFO, MOD_MMI,
			"Key: %s;buf len %u", NMEA_KEY[key], strlen(buf));
#endif
	head = strstr(buf, NMEA_KEY[key]);
	if(head == NULL || head >= buf+strlen(buf))
	{
	#ifdef __GPS_TRACE__		
		gps_tracker_trace(WARN, MOD_MMI,
			"can't find %s", NMEA_KEY[key]);
	#endif
		return KAL_ERROR;
	}

	tail = strstr(head, "\r\n");
	if(tail == NULL || tail >= buf+strlen(buf))
	{
	#ifdef __GPS_TRACE__		
		gps_tracker_trace(WARN, MOD_MMI,
			"can't find 0x0A0D");
	#endif
		return KAL_ERROR;
	}
	#ifdef __GPS_TRACE__		
	gps_tracker_trace(INFO, MOD_MMI,
			"nmea str len %u", tail-head);
	#endif
	memcpy(nmea, head, tail-head);
	return KAL_SUCCESS;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_uart_read
 * DESCRIPTION
 *  ��ʼ�� ϵͳ io
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
static U16 gps_tracker_uart_read(U8 *buf, U16 max_len, UART_PORT port, module_type owner)
{
	U16 avail_num;
	U16 wLenRet = 0;

	if (buf == NULL)
		return 0;

	{
		DCL_HANDLE uart_handle;
		UART_CTRL_RX_AVAIL_T data;
		int i;

		uart_handle = DclSerialPort_Open(port,0);
		DclSerialPort_Control(uart_handle,SIO_CMD_GET_RX_AVAIL, (DCL_CTRL_DATA_T*)&data);
		avail_num = data.u2RetSize;

		if(avail_num > max_len)
			avail_num = max_len;

		for(i = 0;i < avail_num;i++)
		{
			buf[i]=gps_getUARTByte(port);
		}

		wLenRet = avail_num;

		gps_clr_rx_buf(port);
	}

		return wLenRet;

}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_uart_write
 * DESCRIPTION
 *  ��ʼ�� ϵͳ io
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
static U16 gps_tracker_uart_write(U8 *buf, U16 max_len, UART_PORT port, module_type owner)
{
	/*
	�򴮿�д��Ϣ

	������豸���롢���FIFO��UART_Purge��

	����շ��͡�����Buffer��UART_ClrTxBuffer��UART_ClrRxBuffer��

	��д������UART_PutBytes	 

	�رմ���
	*/
	U16 avail_num;
	U16 wLenWrite;
	U16 wLenRet = 0;
	U8  byStatus = 0;

	if (buf == NULL)
		return 0;
	
#ifdef __MTK_TARGET__	
#ifdef __GPRS_TRACE__
	gps_tracker_trace(ERR, MOD_MMI, "gps_tracker_uart_write...");
#endif
	
	U_Purge(port, RX_BUF, owner);
	U_Purge(port, TX_BUF, owner);
	U_ClrTxBuffer(port, owner);
	U_ClrRxBuffer(port, owner);

	// ��ȡ���ݣ����������������ݽ��򵥶�������һ����Ҫ�����Ӧ��Э������Ӧ����
	while( (avail_num = U_GetTxRoomLeft(port) > 0  &&  wLenRet < max_len) )
	{
		int i;

		//������Ҫ��ʱ��������ܷ���ʧ��
		for(i = 0; i< 10000; i++);
		
		if (avail_num + wLenRet > max_len)
		{
			avail_num = max_len - wLenRet;
		}

		wLenWrite = U_PutBytes(port, (kal_uint8 *)(buf + wLenRet), (kal_uint16)avail_num, owner);
		wLenRet += wLenWrite;
	}

	// ����֮���������Buffer
	U_Purge(port, TX_BUF, owner);
	U_ClrTxBuffer(port, owner);
#endif
	return wLenRet;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_uart_ready_to_read_ind_handler
 * DESCRIPTION
 *  ��ʼ�� ϵͳ io
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
static kal_int32 gps_tracker_datetime_offset(applib_time_struct* datetime, S16 offset)
{
	applib_time_struct to_incre = {0};
	applib_time_struct date_offset = {0};
	applib_time_struct result = {0};


	if (datetime == NULL)
		return KAL_ERROR;
	
	memcpy(&to_incre, datetime, sizeof(applib_time_struct));

#ifdef __GPRS_TRACE__
	gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@datetime offset %d %u", offset, (U16)offset);
#endif
	
	//ע��ʱ��������
	if( (U16)offset < 32768)//����
	{
		date_offset.nHour = offset/60;
		date_offset.nMin = offset%60;
#ifdef __GPRS_TRACE__
		gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@datetime %u %u %u %u %u %u", datetime->nYear, datetime->nMonth, datetime->nDay,
			datetime->nHour, datetime->nMin, datetime->nSec);
		gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@datetime active offset hour %d minute %d", date_offset.nHour, date_offset.nMin);
#endif
		applib_dt_increase_time(&to_incre, &date_offset, &result);
	}
	else
	{
		date_offset.nHour = (65536 -(U16)offset)/60;
		date_offset.nMin = (65536 - (U16)offset)%60;

#ifdef __GPRS_TRACE__
		gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@datetime negtive offset hour %d minute %d", date_offset.nHour, date_offset.nMin);
#endif

		applib_dt_decrease_time(&to_incre, &date_offset, &result);
	}
	
	memcpy(datetime, &result, sizeof(applib_time_struct));

#ifdef __GPRS_TRACE__	
	gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@after offset,datetime %u %u %u %u %u %u", datetime->nYear, datetime->nMonth, datetime->nDay,
			datetime->nHour, datetime->nMin, datetime->nSec);
#endif
	return KAL_SUCCESS;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_gps_parse_gprmc
 * DESCRIPTION
 *  ��ʼ�� ϵͳ io
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
static kal_int32 gps_tracker_gps_parse_gprmc(gps_tracker_gps_struct* gps, U8* buf)
{
	U8 *head, *tail;
	double dvalue;
	U32    uvalue;
	char tmp[16] = {0};

	
	if (gps == NULL || buf == NULL)
		return KAL_ERROR;
	
	/*GPRMC,133538.12,A,4717.13792,N,00834.16028,E,13.795,111.39,190203,,,A*53*/
	/*GPRMC,,V,,,,,,,,,,N*53*/
#ifdef __GPS_TRACE__
	gps_tracker_trace(INFO, MOD_MMI, "gprmc:%s",buf);
#endif

	//1.ȡʱ��
	head = buf+6;
	tail = strchr(head, ',');	
	
	//����ʱ�������λ��Ϣ�Ƿ���Ч
	if(tail-head == 0)//˵��2���������ڣ�����Ϊ��	
	{
#ifdef __GPS_TRACE__	
		gps_tracker_trace(INFO, MOD_MMI,
			"invalid gprmc");
#endif
		return KAL_ERROR;
	}
	
	memcpy(tmp, head, tail-head);	

	dvalue = atof(tmp);
	uvalue = (U32)dvalue; 
	gps->datetime[3]= uvalue/10000;
	gps->datetime[4]= (uvalue%10000)/100;
	gps->datetime[5]= uvalue%100;
	
	//2. ��λ״̬
	head = tail+1;
	
	if(*head == 'A' )
	{
		gps->state = EN_GT_GS_A;
	}
	else if(*head == 'V')
	{
		gps->state = EN_GT_GS_V;
	}
	else
	{
		gps->state = EN_GT_GS_INV;
	}
	tail = head+1;
	//3. γ��
	head = tail+1;
	tail = strchr(head, ',');
	if(tail-head == 0)//˵��2���������ڣ�����Ϊ��	
	{
#ifdef __GPS_TRACE__	
		gps_tracker_trace(INFO, MOD_MMI,	"invalid gprmc");
#endif
		return KAL_ERROR;
	}
	memset(tmp,0,16);
	memcpy(tmp, head, tail-head);
	dvalue = atof(tmp);
	uvalue = dvalue;
	gps->latitude = ((uvalue/100) * 60 + (dvalue - uvalue/100*100) )*30000;

	//4.γ�Ȱ���
	head = tail+1;
	
	if(*head == 'N' )
	{
		gps->lat_ind= EN_GT_NORTH;
		tail = head+1;
	}
	else if(*head == 'S')
	{
		gps->lat_ind = EN_GT_SOUTH;
		tail = head+1;
	}
	else
	{
		gps->lat_ind = EN_GT_INV;
		tail = head;
	}
	//5. ����
	head = tail+1;
	tail = strchr(head, ',');	
	if(tail-head == 0)//˵��2���������ڣ�����Ϊ��	
	{
#ifdef __GPS_TRACE__	
		gps_tracker_trace(WARN, MOD_MMI,
			"invalid gprmc");
#endif
		return KAL_ERROR;
	}
	memset(tmp,0,16);
	memcpy(tmp, head, tail-head);
	dvalue = atof(tmp);
	uvalue = dvalue;
	gps->longitude = (uvalue/100 * 60 + (dvalue - uvalue/100*100) )*30000;
	//6.���Ȱ���
	head = tail+1;
	
	if(*head == 'E' )
	{
		gps->long_ind= EN_GT_EAST;
		tail = head+1;
	}
	else if(*head == 'W')
	{
		gps->long_ind = EN_GT_WEST;
		tail = head+1;
	}
	else
	{
		gps->long_ind = EN_GT_INV;
		tail = head;
	}
	
	/*GPRMC,133538.12,A,4717.13792,N,00834.16028,E,13.795,111.39,190203,,,A*53*/
	//7.�ٶ�	
	head = tail+1;
	tail = strchr(head, ',');	
	memset(tmp,0,16);
	memcpy(tmp, head, tail-head);
	dvalue = atof(tmp);
	gps->speed= dvalue*1.852*100;//knots ת����kph

	//8.���� ���汱Ϊ�ο�
	head = tail+1;	
	tail = strchr(head, ',');	
	memset(tmp,0,16);
	memcpy(tmp, head, tail-head);
	dvalue = atof(tmp);
	gps->course = dvalue*100;
	//9. ����
	head = tail+1;
	tail = strchr(head, ',');	
	memcpy(tmp, head, tail-head);
	uvalue = atof(tmp);
	gps->datetime[2]= uvalue/10000;
	gps->datetime[1]= (uvalue%10000)/100;
	gps->datetime[0]= uvalue%100;
	//10 ��ƫ��
	head = tail+1;
	tail = strchr(head, ',');	
	memset(tmp,0,16);
	memcpy(tmp, head, tail-head);
	dvalue = atof(tmp);
	gps->magnetic_value = dvalue*100;

	//11. ��ƫ�Ƿ���
	head = tail+1;
	
	if(*head == 'E' )
	{
		gps->magnetic_ind= EN_GT_EAST;
		tail = head+1;
	}
	else if(*head == 'W')
	{
		gps->magnetic_ind = EN_GT_WEST;
		tail = head+1;
	}
	else
	{
		gps->magnetic_ind = EN_GT_INV;
		tail = head;
	}
	//12. ģʽ 
	head = tail+1;
	if(*head == 'A' )
	{
		gps->mode= EN_GT_GM_A;
	}
	else if(*head == 'D')
	{
		gps->mode = EN_GT_GM_D;
	}
	else if(*head == 'E')
	{
		gps->mode = EN_GT_GM_E;
	}
	else if(*head == 'N')
	{
		gps->mode = EN_GT_GM_N;
	}
	else
	{
		gps->mode = EN_GT_GM_INV;
	}


#if 0	//zzt.20150821.del because change time when login rsp
	//��һ�γɹ���λ
	if(is_gps_datetime_updated == KAL_FALSE)
	{
		applib_time_struct date;
		S16 error;

		
		date.nYear= gps->datetime[0] + YEARFORMATE;
		date.nMonth= gps->datetime[1];
		date.nDay= gps->datetime[2];
		date.nHour= gps->datetime[3];
		date.nMin= gps->datetime[4];
		date.nSec= gps->datetime[5];

		
		gps_tracker_datetime_offset(&date, gps_tracker_config.time_zone);

		mmi_dt_set_rtc_dt((MYTIME *)&date);
#ifdef __GPS_TRACE__		
		gps_tracker_trace(ERR, MOD_MMI,"@@@@@@@@@@@@@@@@@@ update systime by gps time!");
#endif
		
		is_gps_datetime_updated = KAL_TRUE;
		is_datetime_updated = KAL_TRUE;
	}
#endif
	
	//print the gps info
#ifdef __GPS_TRACE__	
	gps_tracker_trace(INFO, MOD_MMI,
		"gps info: datetime:%u %u %u %u %u %u; lat:0X%X,%u; long:0X%X,%u; speed %u; course %u, magnetic:%u %u; mod %u",
		gps->datetime[0], gps->datetime[1], gps->datetime[2],
		gps->datetime[3], gps->datetime[4], gps->datetime[5],
		gps->latitude, gps->lat_ind,
			gps->longitude, gps->long_ind, 
			gps->speed, gps->course,
			gps->magnetic_value, gps->magnetic_ind,
			gps->mode);
#endif
	
	return KAL_SUCCESS;
}
/*****************************************************************************
 * FUNCTION
 *  gps_tracker_gps_parse_gpgga
 * DESCRIPTION
 *  ��ʼ�� ϵͳ io
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
static kal_int32 gps_tracker_gps_parse_gpgga(gps_tracker_gps_struct* gps, U8* buf)
{
	U8 *head, *tail;
	double dvalue;
	char tmp[16] = {0};
	U8 i;


	if (gps == NULL || buf == NULL)
		return KAL_ERROR;

	/*GPGGA,133404.00,4717.14117,N,00833.86174,E,1,11,1.49,478.7,M,48.0,M,,0*62*/
	/*GPGGA,,,,,,0,00,99.99,,,,,,*48*/

#ifdef __GPS_TRACE__	
	gps_tracker_trace(INFO, MOD_MMI, "gpgga:%s",buf);
#endif

	//1.ȡʱ��
	head = buf+6;

	//�ҵ���6��λ�õ�ֵ
	for(i = 0; i < 6; i++)
	{
		tail = strchr(head, ',');
		head = tail+1;
	}
	
	//ȡ���Ǹ���	
	tail = strchr(head, ',');
	memcpy(tmp, head, tail-head);
	gps->sat_uesed = atoi(tmp);

	if(gps->sat_uesed == 0)
	{
#ifdef __GPS_TRACE__	
		gps_tracker_trace(ERR, MOD_MMI,
			"sat num: 0;invalid gpgga");
#endif
		return KAL_ERROR;
	}

	//2. HDOP
	head = tail+1;
	tail = strchr(head, ',');	
	memset(tmp,0,16);
	memcpy(tmp, head, tail-head);
	dvalue = atof(tmp);
	gps->hdop = dvalue*100;

	//3. ����
	head = tail+1;
	tail = strchr(head, ',');	
	memset(tmp,0,16);
	memcpy(tmp, head, tail-head);
	dvalue = atof(tmp);
	gps->msl_altitude= dvalue*10;
	
	//print the gps info
#ifdef __GPS_TRACE__	
	gps_tracker_trace(INFO, MOD_MMI,
		"gps info: sat num %u; HDOP %u; msl_altitude %u",
		gps->sat_uesed, gps->hdop, gps->msl_altitude);
#endif
	
	return KAL_SUCCESS;
}


/*****************************************************************************
 * FUNCTION
 *  gps_tracker_parse_nmea
 * DESCRIPTION
 *  ��ʼ�� ϵͳ io
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
static kal_int32 gps_tracker_parse_nmea(gps_tracker_gps_struct* gps, U8 key, U8* buf)
{
	kal_int32 ret = KAL_ERROR;
	


	if (gps == NULL || buf == NULL)
		return KAL_ERROR;

	
	//��ʼ����gps����
	switch(key)
	{
		case EN_GT_NT_GPRMC:
			ret = gps_tracker_gps_parse_gprmc(gps,buf);
			break;
		case EN_GT_NT_GPGGA:
			ret = gps_tracker_gps_parse_gpgga(gps,buf);
			break;
		default:
			ret = KAL_ERROR;
			break;
	}
	

	return ret;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_get_gps_info
 * DESCRIPTION
 *  ��ʼ�� ϵͳ io
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
static kal_int32 gps_tracker_get_gps_info(void)
{
	kal_uint8 buf[512] = {0};
	kal_uint16 len;
	kal_uint8 nmea[128] = {0};
	kal_int32 ret;
	gps_tracker_gps_struct gps = {0};
	U16 distence = 0;
	U8 i;
#ifdef __GPS_TRACE__	
	gps_tracker_trace(INFO, MOD_MMI, "@@@@@@@@@@@@@uart event ,gps_tracker_get_gps_info");
#endif	
	len = gps_tracker_uart_read(buf, 512-1, uart_port2, MOD_MMI);

#ifdef __GPS_TRACE__	
	gps_tracker_trace(INFO, MOD_MMI, "@@@@@@ uart read: %s", buf);
#endif
	ret = gps_tracker_get_nmea(buf, EN_GT_NT_GPRMC, nmea);
	if(ret != KAL_SUCCESS)
	{
		return KAL_ERROR;
	}

	ret = gps_tracker_parse_nmea(&gps, EN_GT_NT_GPRMC, nmea);
	if(ret != KAL_SUCCESS)
	{
		return KAL_ERROR;
	}

	memset(nmea, 0, 128);

	ret = gps_tracker_get_nmea(buf, EN_GT_NT_GPGGA, nmea);
	if(ret != KAL_SUCCESS)
	{
		return KAL_ERROR;
	}
 
	ret = gps_tracker_parse_nmea(&gps, EN_GT_NT_GPGGA, nmea);
	if(ret != KAL_SUCCESS)
	{
		return KAL_ERROR;
	}

	memcpy(&gps_tracker_gps_fresh, &gps, sizeof(gps_tracker_gps_fresh));

#ifdef __GPS_BEST_HDOP__
	memcpy(&gps_tracker_gps_array[gps_index++], &gps, sizeof(gps_tracker_gps_struct));
#endif
	return KAL_SUCCESS;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_uart_ready_to_read_ind_handler
 * DESCRIPTION
 *  ��ʼ�� ϵͳ io
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
static kal_int32 gps_tracker_uart_ready_to_read_ind_handler(void *msg)
{
	kal_int32 ret;
	uart_ready_to_read_ind_struct*  uart_rtr_ind  = NULL;	


	if (msg == NULL)
		return KAL_ERROR;

	uart_rtr_ind  = (uart_ready_to_read_ind_struct*)msg;
	
#if defined(__GPS_TRACE__) || defined(__CONTROL_TRACE__)	// __MTK_TARGET__	
	gps_tracker_trace(INFO, MOD_MMI,
		"gps_tracker_uart_ready_to_read_ind_handler:uart: %d; owner %d; MOD_MMI %d", 
		uart_rtr_ind->port, U_GetOwnerID(uart_rtr_ind->port), MOD_MMI);
#endif

	if(uart_port2 != uart_rtr_ind->port || MOD_MMI != U_GetOwnerID(uart_rtr_ind->port) )
	{
	
#ifdef __GPS_CONTROL_CONNECT__ 
	if(uart_port1 == uart_rtr_ind->port && MOD_MMI == U_GetOwnerID(uart_rtr_ind->port))
		gps_tracker_recv_data_from_uart();
#endif
		// ����ײ������ɹ�����������Ϳ��Ե��ù���������һ��������
		return KAL_SUCCESS;
	}

	//��ȡgps��Ϣ
	ret = gps_tracker_get_gps_info();

#ifdef __LED_INDICATE_STATE__
	if(ret == KAL_SUCCESS)
	{
		gps_tracker_trace(INFO, MOD_MMI,"gps valid");
		if(!gps_tracker_gps_led)
		{
			gps_tracker_trace(INFO, MOD_MMI,"gps_tracker_open_gps_work_led");
			gps_tracker_open_gps_work_mode();
		}
	}
	else
	{
//		gps_tracker_trace(INFO, MOD_MMI,"gps invalid");
		if(gps_tracker_gps_led)
		{
			gps_tracker_trace(INFO, MOD_MMI,"gps_tracker_close_gps_work_led");
			gps_tracker_close_gps_work_mode();
		}
	}
#endif

	if(ret != KAL_SUCCESS)
	{		
		gps_tracker_gps_parse_failed_times++;
		
		if(gps_tracker_gps_parse_failed_times >= 5)
		{
			gps_tracker_gps_state = EN_GT_GPSS_SEARCH;
			return KAL_ERROR;
		}		
	}
	else
	{
		gps_tracker_gps_parse_failed_times = 0;
	}
	
	gps_tracker_gps_state = EN_GT_GPSS_LOCATED;
	return KAL_SUCCESS;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_io_init
 * DESCRIPTION
 *  ��ʼ�� ϵͳ io
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
/*static kal_int32 gps_tracker_uart_init(void)
{

#ifdef __MTK_TARGET__	
	//��ʼ�����ڶ�Ӧ��io��ʹ֮������ UART ģʽ
	GPIO_ModeSetup(gpio_uart2_tx, 1);
	GPIO_ModeSetup(gpio_uart2_rx, 1);


	UART_DriverInit(uart_port2);

	uart_cur_handle = L1SM_GetHandle();
	L1SM_SleepDisable(uart_cur_handle);

	// ��¼����Ҫ�õĴ��ڵĵ�ǰռ����
	uart_orig_owner = U_GetOwnerID(uart_port2);
	
	// ������������Ҫռ�����������
	U_SetOwner(uart_port2, MOD_MMI);	

	// ���ò����ʣ�ȱʡ����ͣλ��У��Ϊ:8,n,1����8������λ��1��ֹͣλ����У��

	U_SetBaudRate(uart_port2, UART_BAUD_9600, MOD_MMI);


	// ���������趨(����ͣλ��У���)ʹ�ú��� UART_ReadDCBConfig �� UART_SetDCBConfig
	// ��ϸ�������ṹ�� UARTDCBStruct

	// ע��һ���¼����Ӻ�����������(�κ�)�����ݵ���ʱ�����ǵĹ��Ӻ�����������
	// ע�⣬ͬһ���¼�ͬʱֻ��ע��һ�����Ӻ�������ˣ�
	// ��������ǵĳ������ڵ�ͬʱ������������Ҫ��ȡ�ʹ���(�κ�)�������ݣ�
	// �ͱ����ɵ�ǰ�Ĺ��Ӻ�����Ϊ����
	// ʵ�����Ҿ���ϵͳ�ײ���Ը�һ�£��ĳ�Windows���ӵķ�ʽ�����ԹҶ�����ܹ����ε��û�����

	U_ClrRxBuffer(uart_port2, MOD_MMI);
	
	SetProtocolEventHandler(gps_tracker_uart_ready_to_read_ind_handler, MSG_ID_UART_READY_TO_READ_IND);
#endif

#if 0//def _WIN32
{
	uart_ready_to_read_ind_struct  uart_rtr_ind;
	uart_rtr_ind.port = uart_port2;
	gps_tracker_uart_ready_to_read_ind_handler((void*)&uart_rtr_ind);
}
#endif

	return KAL_SUCCESS;
}
*/
static kal_int32 gps_tracker_uart_init(void)
{
#ifndef _WIN32	
	static UART_CONFIG_T dcb1=
	{
		UART_BAUD_9600, //UART_BAUD_9600,//lwq UART_BAUD_115200,
		LEN_8,
		SB_1,
		PA_NONE,
		FC_NONE,
		0x11,
		0x13,
		DCL_FALSE
	};
	static UART_CONFIG_T dcb2=
	{
		UART_BAUD_9600, //UART_BAUD_9600,//lwq UART_BAUD_115200,
		LEN_8,
		SB_1,
		PA_NONE,
		FC_NONE,
		0,
		0,
		DCL_FALSE
	};

	//����2 ��ʼ��
	GPIO_ModeSetup(gpio_uart2_tx, 1);
	GPIO_ModeSetup(gpio_uart2_rx, 1); 
 
	{
		//extern void UART_DriverInit(UART_PORT port,kal_uint32 flag);
		extern UART_HWInit(UART_PORT port);
		//UART_DriverInit(uart_port2, 2/*UART_BOOTUP_TRACE_OPEN_VFIFO*/);
		UART_HWInit(uart_port2);
	}	

	gps_UART_turnon_power(uart_port2, KAL_TRUE);	//zzt.20150803
	gps_UART_open(uart_port2, MOD_MMI);	//zzt.end
	gps_uart_set_owner_id(uart_port2,MOD_MMI);	
	gps_UART_SetDCBConfig(uart_port2,&dcb2,MOD_MMI);
	gps_clr_rx_buf(uart_port2);   
	gps_clr_tx_buf(uart_port2);

#ifdef __GPS_CONTROL_CONNECT__ 
	//����1 ��ʼ��
	GPIO_ModeSetup(gpio_uart1_tx, 1);
	GPIO_ModeSetup(gpio_uart1_rx, 1); 	

	gps_UART_turnon_power(uart_port1, KAL_TRUE);	//zzt.20150803
	gps_UART_open(uart_port1, MOD_MMI);	//zzt.end
	gps_uart_set_owner_id(uart_port1,MOD_MMI);	
	gps_UART_SetDCBConfig(uart_port1,&dcb1,MOD_MMI);
	gps_clr_rx_buf(uart_port1);   
	gps_clr_tx_buf(uart_port1);
#endif

	SetProtocolEventHandler(gps_tracker_uart_ready_to_read_ind_handler, MSG_ID_UART_READY_TO_READ_IND);
#endif
	return KAL_SUCCESS;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_send_em_update_req
 * DESCRIPTION
 *  ��ʼ�� ϵͳ io
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
void gps_tracker_send_em_update_req(BOOL state)
{
   S32 i;
   MYQUEUE Message;
   mmi_em_update_req_struct *update_req;
   
   Message.oslMsgId = MSG_ID_MMI_EM_UPDATE_REQ;
   update_req = OslConstructDataPtr(sizeof(mmi_em_update_req_struct));
   for(i = 0; i < EM_INFO_REQ_NUM; i++)
   {
      update_req->info_request[i] = EM_OFF;
   }
   
   if(state)
   {
      update_req->info_request[RR_EM_LAI_INFO] = EM_ON;
	  update_req->info_request[RR_EM_MEASUREMENT_REPORT_INFO] = EM_ON;
	  update_req->info_request[RR_EM_CA_LIST_INFO] = EM_ON;
   }

	
   Message.oslDataPtr = (oslParaType *)update_req;
   Message.oslPeerBuffPtr = NULL;
   Message.oslSrcId = MOD_MMI;
   Message.oslDestId = MOD_L4C;

   OslMsgSendExtQueue(&Message);
}



U8 gps_tracker_get_max_shake_value(void)
{
		char x=0,y=0,z=0;
		char a=0,b=0,c=0;
		
		FSL_MMA_init();
		FSL_MMA_ReadXYZ8(&x, &y, &z);// Read 8bit XYZ 
	
#ifdef _WIN32
	x=62;y=31;z=1;
#endif
	
		x &= 0x3f;
		y &= 0x3f;
		z &= 0x3f;
	#ifdef __G_SENSOR_TRACE__		
		gps_tracker_trace(INFO, MOD_MMI,"binary: x %d; y %d; z %d", x, y, z);
	#endif
		
		if(x>32) x-=64;
		if(y>32) y-=64;
		if(z>32) z-=64;
	#ifdef __G_SENSOR_TRACE__				
		gps_tracker_trace(INFO, MOD_MMI,"complement: x %d; y %d; z %d", x, y, z);
	#endif
	
		a = abs_S8(x);
		b = abs_S8(y);
		c = abs_S8(z);
		#ifdef __G_SENSOR_TRACE__		
		gps_tracker_trace(INFO, MOD_MMI,"changed abs: x %d; y %d; z %d",a, b, c);
		#endif
		return MAX(a, MAX(b,c));
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_acce_get_vibr
 * DESCRIPTION
 *  ��ȡ ������
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
kal_uint8 gps_tracker_acce_get_shake_change(void)
{
	char 	x=0,y=0,z=0;
	char 	a=0,b=0,c=0;
	U8 		change = 0;
	U8		average_change = 0;
	U8 		i;
	U16		sum = 0;
	

	FSL_MMA_ReadXYZ8(&x, &y, &z);// Read 8bit XYZ 

#ifdef _WIN32
	x=62;y=31;z=1;
#endif

	x &= 0x3f;
	y &= 0x3f;
	z &= 0x3f;

#ifdef __G_SENSOR_TRACE__		
	gps_tracker_trace(INFO, MOD_MMI,"binary: x %d; y %d; z %d", x, y, z);
#endif
	if(x>32) x-=64;
	if(y>32) y-=64;
	if(z>32) z-=64;
	
	a=x;b=y;c=z;
	
	a-=gps_tracker_acce.x;
	b-=gps_tracker_acce.y;
	c-=gps_tracker_acce.z;

#ifdef __G_SENSOR_TRACE__		
	gps_tracker_trace(INFO, MOD_MMI,"changed abs: x %d; y %d; z %d",abs_S8(a), abs_S8(b), abs_S8(c));
#endif
	
	change = abs_S8(a) + abs_S8(b) + abs_S8(c);
	
	gps_tracker_acce.x = x;
	gps_tracker_acce.y = y;
	gps_tracker_acce.z = z;

	//��������ƽ��
	shake_buf_index++;
	shake_buf_index %= SHAKE_BUF_LEN;
	shake_buf[shake_buf_index] = change;

	for(i = 0; i< SHAKE_BUF_LEN; i++)
		sum += shake_buf[i];
	
	average_change = sum/SHAKE_BUF_LEN;
#ifdef __G_SENSOR_TRACE__		
	gps_tracker_trace(INFO, MOD_MMI,"@@@@@@@@@ average_change: %u", average_change);
#endif

	return average_change;
}

kal_uint8 gps_tracker_acce_get_shake(void)
{
	char 	x=0,y=0,z=0;
	char 	a=0,b=0,c=0;
	U8 		shake_value = 0;
	U8 		average_shake_value = 0;
	U8 		i;
	U16		sum = 0;
	

	FSL_MMA_ReadXYZ8(&x, &y, &z);// Read 8bit XYZ 

#ifdef _WIN32
	x=62;y=31;z=1;
#endif

	x &= 0x3f;
	y &= 0x3f;
	z &= 0x3f;
	
	//gps_tracker_trace(INFO, MOD_MMI,"binary: x %d; y %d; z %d", x, y, z);

	//�ڵ��İ��У��𶯴������������⣬����������Ĵ���
	if(x ==0 && y ==0 && z==0)
	{
		gps_tracker_acce.x = x;
		gps_tracker_acce.y = y;
		gps_tracker_acce.z = z;

		return 255;
	}	
	
	if(gps_tracker_acce.x ==0 && gps_tracker_acce.y ==0 && gps_tracker_acce.z ==0)
	{
		gps_tracker_acce.x = x;
		gps_tracker_acce.y = y;
		gps_tracker_acce.z = z;

		return 254;
	}
	//////////////////////////////////////////////////////////
	
	if( x > 32 ) x -= 64;
	if( y > 32 ) y -= 64;
	if( z > 32 ) z -= 64;
	
	shake_value = abs_S8(x) + abs_S8(y) + abs_S8(z);


	//��������ƽ��
	shake_buf_index++;
	shake_buf_index %= SHAKE_BUF_LEN;
	shake_buf[shake_buf_index] = shake_value;

	for(i = 0; i< SHAKE_BUF_LEN; i++)
		sum += shake_buf[i];
	
	average_shake_value = sum/SHAKE_BUF_LEN;

	return average_shake_value;
}


kal_uint8 gps_tracker_acce_get_tilt(void)
{
	kal_uint8 result = 0;

	result = FSL_MMA_IICRead(MMA_TILT);

	return result;
}

#ifdef __ACC_EINT_MODE__
void gps_tracker_shake_eint_callback(void)
{
	gps_tracker_shake_delay_flag = 1;
}
#endif

void gps_tracker_shake_detect_timer_proc(void)
{	

	U8 shake = 0;
	static U8 index=0;
#ifdef __GPS_MCU_CONNECT__
	U8 shake_type = 0;
	U8 shake_num = 0;
	U8 i;
#endif
	
#ifdef __ACC_EINT_MODE__
	gps_tracker_trace(INFO, MOD_MMI,"gps_tracker_shake_detect_timer_proc,gps_tracker_shake_delay_flag=%d",gps_tracker_shake_delay_flag);

	if(!gps_tracker_shake_delay_flag)
		return;

	gps_tracker_shake_delay_flag = 0;
	StopTimer(GPS_TRACKER_SHAKE_DETECT_TIMER);
	StartTimer(GPS_TRACKER_SHAKE_DETECT_TIMER, 1000, gps_tracker_shake_eint_callback);	//1//1����ֻ��Ӧһ���ж�
#endif	
	
	//��ȡ�𶯱仯ֵ
	gps_tracker_shake_value_fresh = gps_tracker_acce_get_shake_change();	
#ifdef __G_SENSOR_TRACE__		
	gps_tracker_trace(INFO, MOD_MMI,"gps_tracker_shake_detect_timer_proc,gps_tracker_shake_value_fresh=%d,index=%d",gps_tracker_shake_value_fresh,index);
#endif

#ifdef __GPS_MCU_CONNECT__
	gps_tracker_trace(INFO, MOD_MMI,"send mcu gps_tracker_shake_value_fresh = %d", gps_tracker_shake_value_fresh);

	gps_tracker_shake_value_array_mcu[shake_value_mcu_index++] = gps_tracker_shake_value_fresh;

	for(i=0; i<3; i++)
	{
		if(gps_tracker_shake_value_array_mcu[i] >25)
			shake_num++;
	}

	if(shake_num>=2)
	{
		shake_type = 2;
		gps_tracker_trace(INFO, MOD_MMI,"send mcu serveal shake warring");
		gps_tracker_communicate_mcu(GT_MCU_SEND_VIBRATE_DATA, &shake_type, 1, NULL);
	}
	else if(gps_tracker_shake_value_fresh >10 &&(shake_value_mcu_index%3==0))
	{
		shake_type = 1;
		gps_tracker_trace(INFO, MOD_MMI,"send mcu one shake warring");
		gps_tracker_communicate_mcu(GT_MCU_SEND_VIBRATE_DATA, &shake_type, 1, NULL);
		shake_value_mcu_index=0;
	}

	if(shake_value_mcu_index>=3)
		shake_value_mcu_index = 0;

#endif

	gps_tracker_shake_value_array[index] = gps_tracker_shake_value_fresh;
	if(++index >= gps_tracker_config.up_intv)
		index = 0;
	
#ifndef __ACC_EINT_MODE__
	if(IsMyTimerExist(GPS_TRACKER_SHAKE_DETECT_TIMER) != MMI_TRUE)
	{
		StartTimer(GPS_TRACKER_SHAKE_DETECT_TIMER, MAX_GT_SHAKE_DETECT_INTV, 
				(FuncPtr)gps_tracker_shake_detect_timer_proc);
	}
#endif
} 
/*****************************************************************************
 * FUNCTION
 *  gps_tracker_acce_init
 * DESCRIPTION
 *  ��ʼ�� ���ٶȴ�����
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
kal_int32 gps_tracker_acce_init(void)
{
#ifdef __ACC_EINT_MODE__
	DCL_HANDLE handle;
#endif
#ifdef __MTK_TARGET__
	FSL_MMA_init();	// IIC hardware init
#endif

#ifdef __ACC_EINT_MODE__
	handle = DclGPIO_Open(DCL_GPIO, ACC_INT_PIN);
	DclGPIO_Control(handle, GPIO_CMD_SET_MODE_3, 0);
	DclGPIO_Control(handle, GPIO_CMD_SET_DIR_IN, 0);
	DclGPIO_Control(handle, GPIO_CMD_DISABLE_PULL, 0);
	DclGPIO_Close(handle);
	
	EINT_Set_Sensitivity(ACC_EINT_NO, EDGE_SENSITIVE);
	EINT_Set_Polarity(ACC_EINT_NO, KAL_FALSE);
 	EINT_Registration(ACC_EINT_NO, KAL_FALSE, KAL_FALSE, gps_tracker_shake_detect_timer_proc, KAL_TRUE);
#endif


 #ifndef __ACC_EINT_MODE__
	//������״̬���
	if(IsMyTimerExist(GPS_TRACKER_SHAKE_DETECT_TIMER) != MMI_TRUE)
	{
		StartTimer(GPS_TRACKER_SHAKE_DETECT_TIMER, MAX_GT_SHAKE_DETECT_INTV, 
				(FuncPtr)gps_tracker_shake_detect_timer_proc);
	}
#endif
	return KAL_SUCCESS;
}
void vibr_on()
{
	DCL_HANDLE handle;
	PMU_CTRL_LDO_BUCK_SET_VOLTAGE_EN val;
	val.voltage = PMU_VOLT_03_000000_V;
	val.mod=VIBR;
	handle=DclPMU_Open(DCL_PMU, FLAGS_NONE);
	DclPMU_Control(handle, LDO_BUCK_SET_VOLTAGE_EN, (DCL_CTRL_DATA_T *)&val);
	DclPMU_Close(handle);
}

void vibr_off()
{
	DCL_HANDLE handle;
	PMU_CTRL_LDO_BUCK_SET_EN val;
	val.enable=DCL_FALSE;
	val.mod=VIBR;
	handle=DclPMU_Open(DCL_PMU, FLAGS_NONE);
	DclPMU_Control(handle, LDO_BUCK_SET_EN, (DCL_CTRL_DATA_T *)&val);
	DclPMU_Close(handle);
}

U32 gps_tracker_gps_pwr_on(void)
{	
#ifdef __MTK_TARGET__
	vibr_on();
#endif
   	return KAL_SUCCESS;
}
U32 gps_tracker_gps_pwr_off(void)
{	
#ifdef __MTK_TARGET__
	vibr_off();
#endif
   	return KAL_SUCCESS;
}
void gps_tracker_dc_in_proc(void)
{
	gps_tracker_trace(ERR, MOD_MMI,"#### DC IN!");
}
void gps_tracker_dc_out_proc(void)
{
	gps_tracker_trace(ERR, MOD_MMI,"#### DC OUT!");
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_io_init
 * DESCRIPTION
 *  ��ʼ�� ϵͳ io
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
void gps_tracker_io_init(void)
{

	//��ʼ�����ٶȴ�����
	gps_tracker_acce_init();

	//��ʼ������
	gps_tracker_uart_init();	
	
	//GPS power on
	gps_tracker_gps_pwr_on();

#ifdef __GPS_MCU_CONNECT__
	gps_tracker_init_mcu_io();
#else	//zzt.debug
	//relay out control,output mode
	GPIO_ModeSetup(RELAY_CTL_IO, 0);
	GPIO_InitIO(1, RELAY_CTL_IO);
	GPIO_WriteIO(1, RELAY_CTL_IO);
#endif	

	//dc input det ,input mode
	GPIO_ModeSetup(DC_INPUT_DET_IO, 0);
	GPIO_InitIO(0, DC_INPUT_DET_IO);

#ifdef __LED_INDICATE_STATE__
	gps_tracker_init_led();
#endif
	
	gps_tracker_trace(ERR, MOD_MMI,"init io success!");
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_variable_init
 * DESCRIPTION
 *  ϵͳȫ�ֱ�����ʼ��
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
void gps_tracker_variable_init(void)
{
	U8* imei;	


	srv_imei_get_imei(MMI_SIM1, gps_tracker_imei, sizeof(gps_tracker_imei));
	gps_tracker_trace(ERR, MOD_MMI,"@@@@@@@@@@@@@@ IMEI %s", gps_tracker_imei);
	
#ifdef _WIN32
	strcpy(gps_tracker_imei, "868790790000680");
#endif

	//dev_id
	memcpy(gps_tracker_dev_id, gps_tracker_imei, MAX_GT_IMEI_LEN-1);

	gps_tracker_trace(ERR, MOD_MMI,"@@@@@@@@@@@@@@ dev_id %s", gps_tracker_dev_id);

	//sn mutex ��ʼ��
	gps_tracker_sn_mutex = kal_create_mutex("sn mutex");
	
	//���Ŷ��� mutex ��ʼ��
	sms_send_array_mutex = kal_create_mutex("sms alarm array mutex");

	//������Ϣ���� mutex ��ʼ��
	rcv_msg_array_mutex = kal_create_mutex("rcv msg array mutex");
	

	//login cb
	gps_tracker_login_cb.prot_type = EN_GT_PT_LOGIN;
	gps_tracker_login_cb.is_responsed = KAL_FALSE;	

	gps_tracker_login_cb.dev_req_packet.head.start = PACKET_START;
	gps_tracker_login_cb.dev_req_packet.head.prot_type = EN_GT_PT_LOGIN;
	gps_tracker_login_cb.dev_req_packet.tail.stop = PACKET_STOP;	
	gps_tracker_login_cb.is_updated = KAL_FALSE;


	hex_str_2_bytes(gps_tracker_dev_id, strlen(gps_tracker_dev_id), 
		gps_tracker_login_cb.dev_req_packet.content.login.dev_id, MAX_GT_DEV_ID_BYTE_LEN);
	gps_tracker_login_cb.dev_req_packet.content.login.auth_code = 0;
	gps_tracker_login_cb.dev_req_packet.content.login.dev_type = gps_tracker_config.dev_type;

	//gps cb
	gps_tracker_gps_cb.prot_type = EN_GT_PT_GPS;
	//�����һ�����ݱ��洢������
	gps_tracker_gps_cb.is_responsed = KAL_TRUE;	

	memset(&gps_tracker_gps_cb.dev_req_packet.content.gps, 0, sizeof(gps_tracker_gps_cb.dev_req_packet.content.gps));
	
	gps_tracker_gps_cb.dev_req_packet.head.start = PACKET_START;
	gps_tracker_gps_cb.dev_req_packet.head.prot_type = EN_GT_PT_GPS;
	gps_tracker_gps_cb.dev_req_packet.tail.stop = PACKET_STOP;
	gps_tracker_gps_cb.is_updated = KAL_FALSE;

	//status cb
	gps_tracker_status_cb.prot_type = EN_GT_PT_STATUS;
	gps_tracker_status_cb.is_responsed = KAL_TRUE;	
	
	gps_tracker_status_cb.dev_req_packet.head.start = PACKET_START;
	gps_tracker_status_cb.dev_req_packet.head.prot_type = EN_GT_PT_STATUS;
	gps_tracker_status_cb.dev_req_packet.tail.stop = PACKET_STOP;
	gps_tracker_status_cb.is_updated = KAL_FALSE;

	//data cb
	gps_tracker_dev_data_cb.prot_type = EN_GT_PT_DEV_DATA;
	gps_tracker_dev_data_cb.is_responsed = KAL_FALSE;	
	
	gps_tracker_dev_data_cb.dev_req_packet.head.start = PACKET_START;
	gps_tracker_dev_data_cb.dev_req_packet.head.prot_type = EN_GT_PT_DEV_DATA;
	gps_tracker_dev_data_cb.dev_req_packet.tail.stop = PACKET_STOP;
	gps_tracker_dev_data_cb.is_updated = KAL_FALSE;

	//hb cb
	gps_tracker_hb_cb.prot_type = EN_GT_PT_HB;
	gps_tracker_hb_cb.is_responsed = KAL_FALSE;	
	
	gps_tracker_hb_cb.dev_req_packet.head.start = PACKET_START;
	gps_tracker_hb_cb.dev_req_packet.head.prot_type = EN_GT_PT_HB;
	gps_tracker_hb_cb.dev_req_packet.tail.stop = PACKET_STOP;
	gps_tracker_hb_cb.is_updated = KAL_FALSE;

	//alarm cb
	gps_tracker_alarm_cb.prot_type = EN_GT_PT_ALARM;
	gps_tracker_alarm_cb.is_responsed = KAL_FALSE;	
	
	gps_tracker_alarm_cb.dev_req_packet.head.start = PACKET_START;
	gps_tracker_alarm_cb.dev_req_packet.head.prot_type = EN_GT_PT_ALARM;
	gps_tracker_alarm_cb.dev_req_packet.tail.stop = PACKET_STOP;
	gps_tracker_alarm_cb.is_updated = KAL_FALSE;

#ifdef __PROTOCOL_CONTROL__
	gps_tracker_control_cb.prot_type = EN_GT_PT_CONTROL;
	gps_tracker_control_cb.is_responsed = KAL_FALSE;	
	
	gps_tracker_control_cb.dev_req_packet.head.start = PACKET_START;
	gps_tracker_control_cb.dev_req_packet.head.prot_type = EN_GT_PT_CONTROL;
	gps_tracker_control_cb.dev_req_packet.tail.stop = PACKET_STOP;
	gps_tracker_control_cb.is_updated = KAL_FALSE;
#endif

	//state
	gps_tracker_gsm_state = EN_GT_GSMS_INIT;
	gps_tracker_gps_state = EN_GT_GPSS_INIT;

	//sock init
	gps_tracker_soc.sockaddr.sock_type = SOC_SOCK_STREAM;
	gps_tracker_soc.sockaddr.port = gps_tracker_config.server.port;	

#ifdef _WIN32
	//gps_tracker_soc.sockaddr.port = 9000;
#endif		

	memcpy(gps_tracker_soc.sockaddr.addr, gps_tracker_config.server.ip, MAX_GT_IP_ADDR_LEN);
	gps_tracker_soc.sockaddr.addr_len = MAX_GT_IP_ADDR_LEN;
	gps_tracker_soc.soc_status = EN_GT_SOCS_INIT;
	

	gps_tracker_acce.x = 0;
	gps_tracker_acce.y = 0;
	gps_tracker_acce.z = 0;
	
	gps_tracker_trace(ERR, MOD_MMI,"init variables success!");
}


/*****************************************************************************
 * FUNCTION
 *  gps_tracker_em_status_ind_hdlr
 * DESCRIPTION
 *  ϵͳ������ȡ��ʼ������
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
static void gps_tracker_em_status_ind_hdlr(void *info)
{
	U16 mm_pdu_len;	
	mmi_em_status_ind_struct *msg = NULL;

   
	if(info == NULL)
	{
	#ifdef __LBS_TRACE__
		gps_tracker_trace(ERR, MOD_MMI,"MSG_ID_MMI_EM_STATUS_IND: null msg!");
	#endif
		return;
	}

	msg = (mmi_em_status_ind_struct *)info;
	
    // ����RR_EM_LAI_INFO
	if(msg->em_info == RR_EM_LAI_INFO)
	{
		rr_em_lai_info_struct *lai_ptr;
		
		lai_ptr = (rr_em_lai_info_struct *)get_pdu_ptr(msg->info,&mm_pdu_len);

		gps_tracker_cell.mcc = lai_ptr->mcc[0]*100 + lai_ptr->mcc[1]*10 + lai_ptr->mcc[2];
		
		if(lai_ptr->mnc[2] == 0x0F)
		{
			gps_tracker_cell.mnc = lai_ptr->mnc[0]*10 + lai_ptr->mnc[1];
		}
		else
		{
			gps_tracker_cell.mnc = lai_ptr->mnc[0]*100 + lai_ptr->mnc[1]*10 + lai_ptr->mnc[2];
		}
		gps_tracker_cell.lac_sid = lai_ptr->lac[0]*0x100 + lai_ptr->lac[1];

		gps_tracker_cell.cellid_nid = lai_ptr->cell_id;
	}
	else if(msg->em_info == RR_EM_MEASUREMENT_REPORT_INFO)
	{		
		rr_em_measurement_report_info_struct *mr_ptr;

	#ifdef __LBS_TRACE__	
		gps_tracker_trace(INFO, MOD_MMI,"QQQQQQQQQQQQ RR_EM_MEASUREMENT_REPORT_INFO");
	#endif	
		mr_ptr = (rr_em_measurement_report_info_struct *)get_pdu_ptr(msg->info,&mm_pdu_len);

		gps_tracker_cell.sig_stren = (mr_ptr->serv_rla_in_quarter_dbm+479)/30;
	#ifdef __LBS_TRACE__	
		gps_tracker_trace(INFO, MOD_MMI,"%u %u %d %d %d %d",
			mr_ptr->serving_arfcn, mr_ptr->serving_bsic,mr_ptr->serv_rla_in_quarter_dbm,
			mr_ptr->c1_serv_cell, mr_ptr->c2_serv_cell, mr_ptr->c31_serv_cell);
	#endif
	} 
	else if(msg->em_info == RR_EM_CHANNEL_DESCR_INFO)
	{		
		/*rr_em_channel_descr_info_struct *cd_ptr;
		U8 num_of_carriers;
		U8 i;

		
		gps_tracker_trace(ERR, MOD_MMI,"QQQQQQQQQQQQ RR_EM_CHANNEL_DESCR_INFO");
		
		cd_ptr = (rr_em_channel_descr_info_struct *)get_pdu_ptr(msg->info,&mm_pdu_len);

		gps_tracker_trace(ERR, MOD_MMI,"QQQQQQQQQQQQ num_of_carriers %u", cd_ptr->num_of_carriers);
		num_of_carriers = cd_ptr->num_of_carriers;
		
		for(i = 0; i < num_of_carriers; i++)
		{
			gps_tracker_trace(ERR, MOD_MMI,"%u %u",i, cd_ptr->arfcn[i]);
		}*/
	} 
	else if(msg->em_info == RR_EM_CA_LIST_INFO)
	{		
		rr_em_ca_list_info_struct *cl_ptr;
		U8 num_of_carriers;
		U8 i;

	#ifdef __LBS_TRACE__			
		gps_tracker_trace(INFO, MOD_MMI,"QQQQQQQQQQQQ RR_EM_CA_LIST_INFO");
	#endif
		
		cl_ptr = (rr_em_ca_list_info_struct *)get_pdu_ptr(msg->info,&mm_pdu_len);
	#ifdef __LBS_TRACE__	
		gps_tracker_trace(INFO, MOD_MMI,"QQQQQQQQQQQQ num_of_carriers %u", cl_ptr->number_of_channels);
	#endif
		num_of_carriers = cl_ptr->number_of_channels;
		
		for(i = 0; i < num_of_carriers; i++)
		{
		#ifdef __LBS_TRACE__		
			gps_tracker_trace(INFO, MOD_MMI,"%u %u",i, cl_ptr->arfcn_list[i]);
		#endif
		}
		
		memcpy(&em_ca_list, cl_ptr, sizeof(em_ca_list));
	} 
	else if(msg->em_info == RR_EM_CONTROL_MSG_INFO)
	{
		/*rr_em_control_msg_info_struct *cm_ptr;

		
		gps_tracker_trace(ERR, MOD_MMI,"QQQQQQQQQQQQ RR_EM_CONTROL_MSG_INFO");
		
		cm_ptr = (rr_em_control_msg_info_struct *)get_pdu_ptr(msg->info,&mm_pdu_len);

		gps_tracker_trace(ERR, MOD_MMI,"QQQQQQQQQQQQ msg_type %u rr_cause %u", cm_ptr->msg_type, cm_ptr->rr_cause);*/
	} 
	
	free_peer_buff(msg->info);   
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_init
 * DESCRIPTION
 *  ϵͳ������ȡ��ʼ������
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
void gps_tracker_cell_init(void)
{
	SetProtocolEventHandler(gps_tracker_em_status_ind_hdlr, MSG_ID_MMI_EM_STATUS_IND);

	gps_tracker_send_em_update_req(KAL_TRUE);
	
	gps_tracker_trace(ERR, MOD_MMI,"init cell success!");
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_task_init
 * DESCRIPTION
 *  ϵͳ������ȡ��ʼ������
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
void gps_tracker_task_init(void)
{
	gps_tracker_upload_timer_proc();

	gps_tracker_sms_send_timer_proc();
	
	gps_tracker_trace(ERR, MOD_MMI,"init task success!");
}


void gps_tracker_restart(void)
{
     DCL_HANDLE dcl_wdt_handle;

	gps_tracker_trace(ERR, MOD_MMI,"gps_tracker_restart");

      dcl_wdt_handle=DclWDT_Open(DCL_WDT, 0);
      DclWDT_Control(dcl_wdt_handle, WDT_CMD_ABN_RESET, 0);
      DclWDT_Close(dcl_wdt_handle);
}
#ifdef __GPS_PACKAGE_BY_QUEUE__
void gps_tracker_queue_init(void)
{
	initQueue(&gps_tracker_upload_queue);
}
#endif
/*****************************************************************************
 * FUNCTION
 *  gps_tracker_auto_reset
 * DESCRIPTION
 *  ��ʱ����
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
void gps_tracker_auto_reset(void)
{//	ÿ������3���Զ����� !
	applib_time_struct Curtime = {0};


	StopTimer(GPS_TRACKER_AUTO_RESET_READY);
	StopTimer(GPS_TRACKER_AUTO_RESET_START);
	
	applib_dt_get_rtc_time(&Curtime);
	
	if(Curtime.nHour == 3 && Curtime.nMin == 0 && (Curtime.nSec > 0 && Curtime.nSec <10))
	{
		gps_tracker_restart();
	}

	StartTimer(GPS_TRACKER_AUTO_RESET_START,1000,gps_tracker_auto_reset);
	
}


/*****************************************************************************
 * FUNCTION
 *  gps_tracker_init
 * DESCRIPTION
 *  ϵͳ������ȡ��ʼ������
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
void gps_tracker_init(void)
{	
	
	//1.��ʼ��logϵͳ
	gps_tracker_files_init();	
	
	//2. ��ȡ�豸Ĭ�ϲ���
	gps_tracker_config_init();		

	//3. ��ʼ��ȫ�ֱ���
	gps_tracker_variable_init();	

	//5. ��ʼ������
	gps_tracker_io_init();	

	//6. ��վ��ʼ��
	gps_tracker_cell_init();
	
	//7. ��ȡ��������Ϣ
	gps_tracker_server_init();	

	//8. ��ʼ�����ݲɼ�
//	gps_tracker_task_init();		//zzt.20150730.move login success
#ifdef __GPS_PACKAGE_BY_QUEUE__
	gps_tracker_queue_init();
#endif

	//9. ��ʱ��������  moodif by zhangping 20150503
	StartTimer(GPS_TRACKER_AUTO_RESET_READY,1000*20,gps_tracker_auto_reset);

#ifdef __GPS_BAT_CONNECT__
	StartTimer(GPS_TRACKER_BATTERY_CONNECT, 4000, gps_get_data_from_battery);
#endif
	
	gps_tracker_trace(ERR, MOD_MMI,"init gps tracker success!");

}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_format_gsm_cell_req
 * DESCRIPTION
 *  ���� gsm ��վ��������
 * PARAMETERS
 * [IN]mcc
 * [IN]mnc
 * [IN]lac
 * [IN]cellid
 * [OUT]gsm_cell_req
 * RETURNS
 *  ������
 *****************************************************************************/
S32 gps_tracker_format_gsm_cell_req(kal_uint32 mcc, kal_uint32 mnc, kal_uint32 lac, 
											kal_uint32 cellid, kal_char* gsm_cell_req)
{	
	//sprintf(gsm_cell_req, "GET /cell2gps/cell2gps3.php?type=%u&mcc=%u&mnc=%u&lac=%u&cellid=%u HTTP/1.1\r\nHost: %s\r\nConnection: keep-alive\r\nAccept: text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8\r\nAccept-Encoding: gzip,deflate,sdch\r\nAccept-Language: zh-TW,zh;q=0.8,en-US;q=0.6,en;q=0.4,zh-CN;q=0.2\r\nAccept-Charset: GBK,utf-8;q=0.7,*;q=0.3\r\n\r\n",
	//	mnc+1, mcc, mnc, lac, cellid, gps_tracker_host.domain_name);

	//sprintf(gsm_cell_req, "GET /inteliot/inteliot/php/address.php?terminalindex=%u&lac=%u&cellid=%u HTTP/1.1\r\nHost: %s:%u\r\nConnection: keep-alive\r\nAccept: text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8\r\nAccept-Encoding: gzip,deflate,sdch\r\nAccept-Language: zh-TW,zh;q=0.8,en-US;q=0.6,en;q=0.4,zh-CN;q=0.2\r\nAccept-Charset: GBK,utf-8;q=0.7,*;q=0.3\r\n\r\n",
	//		mnc+1, lac, cellid, gps_tracker_host.domain_name, gps_tracker_host.sockaddr.port);

	
	sprintf(gsm_cell_req, "this is the test phone data\r\n");

	return KAL_SUCCESS;
}

void do_nothing(void)
{
	;
}
/*****************************************************************************
 * FUNCTION
 *  gps_tracker_dev_data_rsp_proc
 * DESCRIPTION
 * dev_cfg  ������Ӧ�Ĵ����������ݲ�ͬ����Ϣ���ͻظ���ͬ����Ӧ
 * PARAMETERS
 * RETURNS
 *  ������
 *****************************************************************************/
kal_int32 gps_tracker_dev_data_rsp_proc(gps_tracker_data_content_struct* data)
{
	//U16 error;
	//U16 send_buf[MAX_GT_SMS_CONTENT_LEN] = {0};
	//U8 tmp_buf[MAX_GT_SMS_CONTENT_LEN*2] = {0};

	if (data == NULL)
		return KAL_ERROR;

	//���ݲ�ͬ�Ķ������ͻظ���ͬ������
	switch(data->type)
	{					
		case EN_GT_DT_ADMIN_NUM:
			#ifdef __GPRS_TRACE__		
			gps_tracker_trace(ERR, MOD_MMI,
					"receive server's device data <admin> response");	
			#endif
			break;
		case EN_GT_DT_PWD:
			#ifdef __GPRS_TRACE__					
			gps_tracker_trace(ERR, MOD_MMI,
					"receive server's device data <pwd> response");			
			#endif
			break;
		case EN_GT_DT_USER:
		#ifdef __GPRS_TRACE__			
			gps_tracker_trace(ERR, MOD_MMI,
					"receive server's device data <user> response");	
		#endif
			break;
		case EN_GT_DT_UP_INTV:
		#ifdef __GPRS_TRACE__				
			gps_tracker_trace(ERR, MOD_MMI,
					"receive server's device data <up-intv> response");
		#endif
			break;
		case EN_GT_DT_HB_INTV:
		#ifdef __GPRS_TRACE__						
			gps_tracker_trace(ERR, MOD_MMI,
					"receive server's device data <hb-intv> response");
		#endif
			break;
		case EN_GT_DT_SMS_ALARM_INTV:
		#ifdef __GPRS_TRACE__						
			gps_tracker_trace(ERR, MOD_MMI,
					"receive server's device data <alarm-intv> response");
		#endif
			break;
		case EN_GT_DT_TEMP_THR:
		#ifdef __GPRS_TRACE__						
			gps_tracker_trace(ERR, MOD_MMI,
					"receive server's device data <temp-thr> response");
		#endif
			break;
		case EN_GT_DT_VIBR_THR:
		#ifdef __GPRS_TRACE__				
			gps_tracker_trace(ERR, MOD_MMI,
					"receive server's device data <vibr-thr> response");
		#endif
			break;
		case EN_GT_DT_SPEED_THR:
		#ifdef __GPRS_TRACE__							
			gps_tracker_trace(ERR, MOD_MMI,
					"receive server's device data <speed-thr> response");
		#endif
			break;
		case EN_GT_DT_LOC:
			{
				U16 i = 0;
				U8* data_value = (U8*)&data->value;
				U8 tmp_buf[MAX_GT_SMS_CONTENT_LEN*2] = {0};

				#ifdef __GPRS_TRACE__						
				gps_tracker_trace(ERR, MOD_MMI,"EN_GT_DT_LOC ");
				#endif
				
				//�������ֽ���ת���ɻ����ֽ���
				for(i = 0; i < data->value_len; i++)
				{
					if(i%2 == 0)
					{
						tmp_buf[i + 1] = data_value[i];
					}
					else
					{
						tmp_buf[i - 1] = data_value[i];
					}
				}

				gps_tracker_send_sms(gps_tracker_sms_req.number.num_u16, (U16*)tmp_buf);
				#ifdef __GPRS_TRACE__						
				gps_tracker_trace(ERR, MOD_MMI,
					"receive server's device data <loc> response");
				#endif
				return KAL_SUCCESS;
			}					
			break;
		case EN_GT_DT_LANG:
		#ifdef __GPRS_TRACE__							
			gps_tracker_trace(ERR, MOD_MMI,
					"receive server's device data <lang> response");
		#endif
			break;
		case EN_GT_DT_TIME_ZONE:
		#ifdef __GPRS_TRACE__					
			gps_tracker_trace(ERR, MOD_MMI,
					"receive server's device data <time-zone> response");	
		#endif
			break;
		case EN_GT_DT_RESTORE:
		#ifdef __GPRS_TRACE__							
			gps_tracker_trace(ERR, MOD_MMI,
					"receive server's device data <restore> response");
		#endif
			break;
		#ifdef __GPRS_TRACE__									
		case EN_GT_DT_ALARM_SWITCH:
			gps_tracker_trace(ERR, MOD_MMI,
					"receive server's device data <alarm-switch> response");	
		#endif
			break;
		case EN_GT_DT_SMS_ALARM_SWITCH:
		#ifdef __GPRS_TRACE__												
			gps_tracker_trace(ERR, MOD_MMI,
					"receive server's device data <sms-switch> response");
		#endif
			break;		
		case EN_GT_DT_IGNORE_ALARM:
		#ifdef __GPRS_TRACE__													
			gps_tracker_trace(ERR, MOD_MMI,
					"receive server's device data <ignore-switch> response");
		#endif
			break;		
		case EN_GT_DT_APN:
		#ifdef __GPRS_TRACE__													
			gps_tracker_trace(ERR, MOD_MMI,
					"receive server's device data <apn> response");
		#endif
			break;
		case EN_GT_DT_SERVER:	
		#ifdef __GPRS_TRACE__														
			gps_tracker_trace(ERR, MOD_MMI,
					"receive server's device data <server> response");	
		#endif
			break;
		case EN_GT_DT_DEFENCE:
		#ifdef __GPRS_TRACE__																	
			gps_tracker_trace(ERR, MOD_MMI,
					"receive server's device data <defence> response");	
		#endif
			break;
		case EN_GT_DT_SMS_CENTER:
		#ifdef __GPRS_TRACE__																	
			gps_tracker_trace(ERR, MOD_MMI,
					"receive server's device data <sms-center> response");	
		#endif
			break;
		case EN_GT_DT_VER:
		#ifdef __GPRS_TRACE__																	
			gps_tracker_trace(ERR, MOD_MMI,
					"receive server's device data <ver> response");		
		#endif
			return KAL_SUCCESS;
		default:
			break;
	}				
	return KAL_SUCCESS;
}

void gps_tracker_alarm_ignore_pwr_low_timer_proc()
{
#ifdef __G_SENSOR_TRACE__
	gps_tracker_trace(ERR, MOD_MMI, "ignore pwr_low alarm");
#endif
}

void gps_tracker_alarm_ignore_pwr_off_timer_proc()
{
#ifdef __G_SENSOR_TRACE__
	gps_tracker_trace(ERR, MOD_MMI, "ignore pwr_off alarm");
#endif
}
void gps_tracker_alarm_ignore_vibr_timer_proc()
{
#ifdef __G_SENSOR_TRACE__
	gps_tracker_trace(ERR, MOD_MMI, "ignore vibr alarm");
#endif
}
void gps_tracker_alarm_ignore_pwr_oil_timer_proc()
{
#ifdef __G_SENSOR_TRACE__
	gps_tracker_trace(ERR, MOD_MMI, "ignore pwr_oil alarm");
#endif
}
void gps_tracker_alarm_ignore_sos_timer_proc()
{
#ifdef __G_SENSOR_TRACE__
	gps_tracker_trace(ERR, MOD_MMI, "ignore sos alarm");
#endif
}

void gps_tracker_alarm_ignore_temp_timer_proc()
{
#ifdef __G_SENSOR_TRACE__
	gps_tracker_trace(ERR, MOD_MMI, "ignore temp alarm");
#endif
}

void gps_tracker_alarm_ignore_speed_timer_proc()
{
#ifdef __G_SENSOR_TRACE__
	gps_tracker_trace(ERR, MOD_MMI, "ignore speed alarm");
#endif
}
void gps_tracker_alarm_ignore_fence_timer_proc()
{
#ifdef __G_SENSOR_TRACE__
	gps_tracker_trace(ERR, MOD_MMI, "ignore fence alarm");
#endif
}


/*****************************************************************************
 * FUNCTION
 *  gps_tracker_srv_data_req_proc
 * DESCRIPTION
 * dev_cfg  ������Ӧ�Ĵ����������ݲ�ͬ����Ϣ���ͻظ���ͬ����Ӧ
 * PARAMETERS
 * RETURNS
 *  ������
 *****************************************************************************/
kal_int32 gps_tracker_srv_data_req_proc(gps_tracker_data_content_struct* data)
{
	U16 error;
	S32 ret;


	if (data == NULL)
		return KAL_ERROR;
	
	switch(data->type)
	{					
		case EN_GT_DT_ADMIN_NUM:
		#ifdef __GPRS_TRACE__	
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@ admin num %s", data->value.admin_number);
		#endif
			// ����ϵͳȫ�ֱ���
			strcpy(gps_tracker_config.admin_num, data->value.admin_number);

			// ���½� NVRAM 
			error = NVRAM_WRITE_FAIL;
		
			WriteRecord(NVRAM_EF_GT_ADM_NUM_LID, 1, gps_tracker_config.admin_num, 
				sizeof(gps_tracker_config.admin_num), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
			#ifdef __G_SENSOR_TRACE__	
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <admin_num> failed! admin number:%s",
					gps_tracker_config.admin_num);	
			#endif

				return EN_GT_EC_NVRAM_OPT_ERR;
			}		
			break;
		case EN_GT_DT_PWD:
		#ifdef __GPRS_TRACE__				
			// ����ϵͳȫ�ֱ���
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@ pwd %u", data->value.pwd);
		#endif
			gps_tracker_config.pwd = data->value.pwd;

			// ���½� NVRAM	
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_PWD_LID, 1, &gps_tracker_config.pwd, 
				sizeof(gps_tracker_config.pwd), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
			#ifdef __GPRS_TRACE__		
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <pwd> failed!");	
			#endif
				return EN_GT_EC_NVRAM_OPT_ERR;
			}
			break;
		case EN_GT_DT_USER:
			#ifdef __GPRS_TRACE__					
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@ 1:%s 2:%s 3:%s 4:%s", 
				data->value.users[0], data->value.users[1],data->value.users[2],data->value.users[3]);
			#endif
			
			memcpy(gps_tracker_config.users, data->value.users, sizeof(gps_tracker_config.users));			
			
			// 2 ����nvram�����user �б�
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_USERS_LID, 1, &gps_tracker_config.users, 
				sizeof(gps_tracker_config.users), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
			#ifdef __GPRS_TRACE__				
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <users> failed!");	
			#endif

				return EN_GT_EC_NVRAM_OPT_ERR;
			}
			break;
		case EN_GT_DT_UP_INTV:
			// 1 ��������
			#ifdef __GPRS_TRACE__							
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@ up_intv %u", data->value.upload_intv);
			#endif
			gps_tracker_config.up_intv = data->value.upload_intv;
			
			// 2 ����nvram�����user �б�
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_UP_INTV_LID, 1, &gps_tracker_config.up_intv, 
				sizeof(gps_tracker_config.up_intv), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
			#ifdef __GPRS_TRACE__				
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <up_intv> failed!");	
			#endif
				return EN_GT_EC_NVRAM_OPT_ERR;
			}	
			
			break;
		case EN_GT_DT_HB_INTV:
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@@@server <hb_intv> : %d", data->value.hb_intv);
/*
			// 1 ��������
			gps_tracker_config.hb_intv = data->value.hb_intv;
			
			// 2 ����nvram�����user �б�
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_HB_INTV_LID, 1, &gps_tracker_config.hb_intv, 
				sizeof(gps_tracker_config.hb_intv), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <hb_intv> failed!");	

				return EN_GT_EC_NVRAM_OPT_ERR;
			}
*/			
			break;
		case EN_GT_DT_SMS_ALARM_INTV:
			// 1 ��������
/*			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@ sms_send_intv %u", data->value.sms_send_intv);
			gps_tracker_config.sms_send_intv = data->value.sms_send_intv;
			
			// 2 ����nvram�����user �б�
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_SMS_INTV_LID, 1, &gps_tracker_config.sms_send_intv, 
				sizeof(gps_tracker_config.sms_send_intv), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <sms_send_intv> failed!");	

				return EN_GT_EC_NVRAM_OPT_ERR;
			}	
*/			
			break;
		case EN_GT_DT_TEMP_THR:
			// 1 ��������
		#ifdef __GPRS_TRACE__			
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@ temp_thr %u", data->value.temp_thr);
		#endif
			gps_tracker_config.temp_thr = data->value.temp_thr;
			
			// 2 ����nvram�����user �б�
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_TEMP_THR_LID, 1, &gps_tracker_config.temp_thr, 
				sizeof(gps_tracker_config.temp_thr), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
		#ifdef __GPRS_TRACE__						
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <temp_thr> failed!");	
		#endif

				return EN_GT_EC_NVRAM_OPT_ERR;
			}
			break;
		case EN_GT_DT_VIBR_THR:
			// 1 ��������
		#ifdef __GPRS_TRACE__				
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@ vibr_thr %u", data->value.vibr_thr);
		#endif
			gps_tracker_config.vibr_thr = data->value.vibr_thr;
			
			// 2 ����nvram�����user �б�
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_VIBR_THR_LID, 1, &gps_tracker_config.vibr_thr, 
				sizeof(gps_tracker_config.vibr_thr), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
			#ifdef __GPRS_TRACE__					
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <vibr_thr> failed!");	
			#endif

				return EN_GT_EC_NVRAM_OPT_ERR;
			}	
			break;
		case EN_GT_DT_SPEED_THR:
			// 1 ��������
			#ifdef __GPRS_TRACE__								
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@ speed_thr %u", data->value.speed_thr);
			#endif
			gps_tracker_config.speed_thr = data->value.speed_thr;
			
			// 2 ����nvram�����user �б�
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_SPEED_THR_LID, 1, &gps_tracker_config.speed_thr, 
				sizeof(gps_tracker_config.speed_thr), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
			#ifdef __GPRS_TRACE__											
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <speed_thr> failed!");	
			#endif

				return EN_GT_EC_NVRAM_OPT_ERR;
			}	
			break;
		case EN_GT_DT_LANG:
			//Ŀǰ����ƽ̨�����ݵĲ������ã���ʱ���δ����ݵĸ���
			/*
			// 1 ��������
			gps_tracker_config.lang = data->value.lang;
			
			// 2 ����nvram�����user �б�
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_LANG_LID, 1, &gps_tracker_config.lang, 
				sizeof(gps_tracker_config.lang), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <lang> failed!");	

				return EN_GT_EC_NVRAM_OPT_ERR;
			}
			*/
			break;
		case EN_GT_DT_TIME_ZONE:
		#ifdef __GPRS_TRACE__									
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@@@server <time-zone> : %d", data->value.time_zone);
		#endif
			//Ŀǰ����ƽ̨�����ݵĲ������ã���ʱ���δ����ݵĸ���
			/*
			// 1 ��������
			gps_tracker_config.time_zone = data->value.time_zone;
			
			// 2 ����nvram�����user �б�
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_TIME_ZONE_LID, 1, &gps_tracker_config.time_zone, 
				sizeof(gps_tracker_config.time_zone), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <time-zone> failed!");	

				return EN_GT_EC_NVRAM_OPT_ERR;
			}
			
			//���ò���������Ч
			if(IsMyTimerExist(GPS_TRACKER_SMS_DELAY_REPLAY_TIMER) != MMI_TRUE)
			{
				gps_tracker_trace(ERR, MOD_MMI, "delay reset...");
				StartTimer(GPS_TRACKER_SMS_DELAY_REPLAY_TIMER, MAX_GT_SMS_DELAY_REPLY_TIME, 
						(FuncPtr)gps_tracker_sms_delay_rst_proc);
			}
			*/
			break;
		case EN_GT_DT_ALARM_SWITCH:
			// 1 ��������
		#ifdef __GPRS_TRACE__									
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@ alarm_switch:switch type:%d switch value %d", 
				data->value.alarm_switch.type, data->value.alarm_switch.value);
		#endif
			if(data->value.alarm_switch.value == 1)
			{
				*(U8*)&gps_tracker_config.alarm_switch |= (1<<data->value.alarm_switch.type);
			}
			else
			{
				*(U8*)&gps_tracker_config.alarm_switch &= ~(1<<data->value.alarm_switch.type);
			}

		#ifdef __GPRS_TRACE__								
			gps_tracker_trace(ERR, MOD_MMI,
					"alarm:pwr_low %u pwr_off %u vibr %u oil_pwr %u speed %u" ,
					gps_tracker_config.alarm_switch.pwr_low, gps_tracker_config.alarm_switch.pwr_off,
					gps_tracker_config.alarm_switch.vibr, gps_tracker_config.alarm_switch.oil_pwr,
					gps_tracker_config.alarm_switch.speed);
		#endif
			// 2 ����nvram�����user �б�
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_ALARM_SWITCH_LID, 1, &gps_tracker_config.alarm_switch, 
				sizeof(gps_tracker_config.alarm_switch), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
			#ifdef __GPRS_TRACE__											
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <alarm-switch> failed!");	
			#endif

				return EN_GT_EC_NVRAM_OPT_ERR;
			}	
			break;
		case EN_GT_DT_SMS_ALARM_SWITCH:
			// 1 ��������
			#ifdef __GPRS_TRACE__														
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@ sms_alarm_switch:switch type:%d switch value %d", 
				data->value.sms_alarm_switch.type, data->value.sms_alarm_switch.value);
			#endif

			if(data->value.sms_alarm_switch.value == 1)
			{
				*(U8*)&gps_tracker_config.sms_alarm_switch |= (1<<data->value.sms_alarm_switch.type);
			}
			else
			{
				*(U8*)&gps_tracker_config.sms_alarm_switch &= ~(1<<data->value.sms_alarm_switch.type);
			}
			#ifdef __GPRS_TRACE__																	
			gps_tracker_trace(ERR, MOD_MMI,
					"sms_alarm:pwr_low %u pwr_off %u vibr %u oil_pwr %u  speed %u ",
					gps_tracker_config.sms_alarm_switch.pwr_low, gps_tracker_config.sms_alarm_switch.pwr_off,
					gps_tracker_config.sms_alarm_switch.vibr, gps_tracker_config.sms_alarm_switch.oil_pwr,
					gps_tracker_config.sms_alarm_switch.speed);
			#endif
			
			// 2 ����nvram�����user �б�
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_SMS_SWITCH_LID, 1, &gps_tracker_config.sms_alarm_switch, 
				sizeof(gps_tracker_config.sms_alarm_switch), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
			#ifdef __GPRS_TRACE__																				
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <sms-switch> failed!");	
			#endif

				return EN_GT_EC_NVRAM_OPT_ERR;
			}	
			
			break;
		case EN_GT_DT_LOC:	
			//do nothing
			break;
		case EN_GT_DT_IGNORE_ALARM:
		#ifdef __GPRS_TRACE__																	
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@ ignore alarm %u", data->value.ignore_alarm);
		#endif
			
			switch(data->value.ignore_alarm)
			{
				case EN_GT_AT_PWR_LOW:
					if(IsMyTimerExist(GPS_TRACKER_ALARM_IGNORE_PWR_LOW_TIMER) != MMI_TRUE)
					{
						StartTimer(GPS_TRACKER_ALARM_IGNORE_PWR_LOW_TIMER,10*60*1000,(FuncPtr)gps_tracker_alarm_ignore_pwr_low_timer_proc);
					}
					break;
				case EN_GT_AT_PWR_OFF:
					if(IsMyTimerExist(GPS_TRACKER_ALARM_IGNORE_PWR_OFF_TIMER) != MMI_TRUE)
					{
						StartTimer(GPS_TRACKER_ALARM_IGNORE_PWR_OFF_TIMER,10*60*1000,(FuncPtr)gps_tracker_alarm_ignore_pwr_off_timer_proc);
					}
					break;
				case EN_GT_AT_VIBR:
					if(IsMyTimerExist(GPS_TRACKER_ALARM_IGNORE_VIBR_TIMER) != MMI_TRUE)
					{
						StartTimer(GPS_TRACKER_ALARM_IGNORE_VIBR_TIMER,10*60*1000,(FuncPtr)gps_tracker_alarm_ignore_vibr_timer_proc);
					}
					break;

				case EN_GT_AT_OIL_PWR:
					if(IsMyTimerExist(GPS_TRACKER_ALARM_IGNORE_PWR_OIL_TIMER) != MMI_TRUE)
					{
						StartTimer(GPS_TRACKER_ALARM_IGNORE_PWR_OIL_TIMER,10*60*1000,(FuncPtr)gps_tracker_alarm_ignore_pwr_oil_timer_proc);
					}
					break;
				case EN_GT_AT_SPEED:
					if(IsMyTimerExist(GPS_TRACKER_ALARM_IGNORE_SPEED_TIMER) != MMI_TRUE)
					{
						StartTimer(GPS_TRACKER_ALARM_IGNORE_SPEED_TIMER,10*60*1000,(FuncPtr)gps_tracker_alarm_ignore_speed_timer_proc);
					}
					break;
				default:
					break;
			}	
			break;
		case EN_GT_DT_LOG_LEVEL:	
			gps_tracker_log_level = data->value.log_level;
			break;
		case EN_GT_DT_SET_TIME:	
			{				
				applib_time_struct date;	

				date.nYear= data->value.datetime[0] + YEARFORMATE;
				date.nMonth= data->value.datetime[1];
				date.nDay= data->value.datetime[2];
				date.nHour= data->value.datetime[3];
				date.nMin= data->value.datetime[4];
				date.nSec= data->value.datetime[5];
				
				mmi_dt_set_rtc_dt((MYTIME *)&date);
			}
			break;
		case EN_GT_DT_RESTORE:
			ret = gps_tracker_nvram_restore();

			if(KAL_SUCCESS != ret)
			{
			#ifdef __GPRS_TRACE__																	
				gps_tracker_trace(ERR, MOD_MMI,
						"restore failed");	
			#endif

				return ret;
			}
			
			if(IsMyTimerExist(GPS_TRACKER_SMS_DELAY_REPLAY_TIMER) != MMI_TRUE)
			{
			#ifdef __GPRS_TRACE__																		
				gps_tracker_trace(ERR, MOD_MMI, "delay reset...");
			#endif
				StartTimer(GPS_TRACKER_SMS_DELAY_REPLAY_TIMER, MAX_GT_SMS_DELAY_REPLY_TIME, 
						(FuncPtr)gps_tracker_sms_delay_rst_proc);
			}
			break;
		case EN_GT_DT_SMS:	
			{
				U16 i = 0;
				U8* content = (U8*)data->value.sms.sms_content;
				U16 num[MAX_GT_PHONE_NUM_LEN] = {0};
				U16 send_buf[MAX_GT_SMS_CONTENT_LEN] = {0};
				U8 tmp_buf[MAX_GT_SMS_CONTENT_LEN] = {0};
						
				//�������ֽ���ת���ɻ����ֽ���
				for(i = 0; i < data->value_len; i++)
				{
					if(i%2 == 0)
					{
						tmp_buf[i + 1] = content[i];
					}
					else
					{
						tmp_buf[i - 1] = content[i];
					}
				}
				
				//�������⴮���������滻
				if(wcscmp((wchar_t*)tmp_buf, L"imei") == 0)
				{
					mbstowcs((wchar_t*)tmp_buf, gps_tracker_imei, strlen(gps_tracker_imei));
				}
				else if(wcscmp((wchar_t*)tmp_buf, L"dev_id") == 0)
				{
					mbstowcs((wchar_t*)tmp_buf, gps_tracker_dev_id, strlen(gps_tracker_dev_id));
				}
				
				for (i =0; i < MAX_GT_USER_COUNT; i++)
				{
					if(strlen(data->value.sms.users[i]) > 0)
					{
						mbstowcs(num, data->value.sms.users[i], strlen(data->value.sms.users[i]));

						gps_tracker_send_sms(num, (U16*)tmp_buf);
						/*srv_sms_send_ucs2_text_msg((S8*)tmp_buf,
									(S8*)num,
									SRV_SMS_SIM_1,
									NULL,
									NULL);*/
					}					
				}
			}
			break;
		case EN_GT_DT_DEFENCE:	
			// 1 ��������
			gps_tracker_config.defence =  data->value.defence;
			
			// 2 ����nvram�����user �б�
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_DEFENCE_LID, 1, &gps_tracker_config.defence, 
				sizeof(gps_tracker_config.defence), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
			#ifdef __GPRS_TRACE__																	
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <defence> failed!");	
			#endif

				return EN_GT_EC_NVRAM_OPT_ERR;
			}
			#ifdef __GPRS_TRACE__																				
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@ defence switch %u", gps_tracker_config.defence);
			#endif
			break;
		case EN_GT_DT_SERVER:

			//���ﲻ�����忽������Ϊip ��ʽ�İ�����û���������ᵼ����ipʧ��ʱתDNS ��DNS��ʧ�ܡ�
			gps_tracker_config.server.addr_type = data->value.server.addr_type;

			if(gps_tracker_config.server.addr_type == EN_GT_ADT_IP)
			{
				memcpy(gps_tracker_config.server.ip, data->value.server.ip, sizeof(gps_tracker_config.server.ip));
			}
			else
			{
				memcpy(gps_tracker_config.server.domain, data->value.server.domain, sizeof(gps_tracker_config.server.domain));
			}

			gps_tracker_config.server.port = data->value.server.port;

			//memcpy(&gps_tracker_config.server, &data->value.server, sizeof(gps_tracker_config.server));

			// ���µ�nvram
			WriteRecord(NVRAM_EF_GT_SERVER_LID, 1, &gps_tracker_config.server, 
				sizeof(gps_tracker_config.server), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
			#ifdef __GPRS_TRACE__																				
				gps_tracker_trace(ERR, MOD_MMI,"write nvram record <server> failed!");	
			#endif

				return KAL_ERROR;
			}
			#ifdef __GPRS_TRACE__																				
			gps_tracker_trace(ERR, MOD_MMI, "domain %s port %u", gps_tracker_config.server.domain, gps_tracker_config.server.port);
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@ EN_GT_DT_SERVER %u OK", EN_GT_DT_SERVER);
			#endif
			//���ò���������Ч
			if(IsMyTimerExist(GPS_TRACKER_SMS_DELAY_REPLAY_TIMER) != MMI_TRUE)
			{
			#ifdef __GPRS_TRACE__																							
				gps_tracker_trace(ERR, MOD_MMI, "delay reset...");
			#endif
				StartTimer(GPS_TRACKER_SMS_DELAY_REPLAY_TIMER, MAX_GT_SMS_DELAY_REPLY_TIME, 
						(FuncPtr)gps_tracker_sms_delay_rst_proc);
			}
			break;
		case EN_GT_DT_SMS_CENTER:
			// 1 ��������
			memcpy(gps_tracker_config.sms_center_num, data->value.sms_center_num, sizeof(gps_tracker_config.sms_center_num));
			// 2 ����nvram�����user �б�
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_SMS_CENTER_NUM_LID, 1, gps_tracker_config.sms_center_num, 
				sizeof(gps_tracker_config.sms_center_num), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
			#ifdef __GPRS_TRACE__																				
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <sms-center> failed!");	
			#endif

				return EN_GT_EC_NVRAM_OPT_ERR;
			}
		#ifdef __GPRS_TRACE__																					
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@ sms-center %s", gps_tracker_config.sms_center_num);
		#endif
			break;
		case EN_GT_DT_PWR_OIL_SWITCH:
		#ifdef __GPRS_TRACE__																					
			gps_tracker_trace(ERR, MOD_MMI,"received EN_IDS_DT_PWR_OIL_SWITCH msg, oil_pwr_switch=%u", 
				data->value.oil_pwr_switch);
		#endif

			if(data->value.oil_pwr_switch == 0)
			{
				GPIO_WriteIO(1,RELAY_CTL_IO);

				//�͵�Ͽ��澯
				if(IsMyTimerExist(GPS_TRACKER_ALARM_IGNORE_PWR_OIL_TIMER) != MMI_TRUE)
				{
					gps_tracker_alarm.oil_pwr_ind = 1;
				}

				gps_tracker_state.oil_pwr_state = 0;
			}
			else
			{
				GPIO_WriteIO(0,RELAY_CTL_IO);

				gps_tracker_state.oil_pwr_state = 1;
			}
			break;
		case EN_GT_DT_IO:
			#ifdef __GPRS_TRACE__																							
			gps_tracker_trace(ERR, MOD_MMI,"received EN_GT_DT_IO msg, port = %d, state = %d", 
				data->value.io.port, data->value.io.state);
			#endif

		/*zzt 20151014 add marco, because mcu used gpio1,gpio2, so del code*/
		#ifndef __GPS_MCU_CONNECT__			
			GPIO_ModeSetup(data->value.io.port|0x80, 0);
			GPIO_InitIO(1, data->value.io.port|0x80);
			GPIO_WriteIO(data->value.io.state, data->value.io.port|0x80);			
		#endif
			//���͵�澯 58 �ߵ�ƽ relay ��ͨ
			if(RELAY_CTL_PIN_NO == data->value.io.port && data->value.io.state == 1)
			{
				gps_tracker_alarm.oil_pwr_ind = 1;
			}
			else
			{
				gps_tracker_alarm.oil_pwr_ind = 0;
			}
			break;
		default:
			break;
	}	
#ifdef __GPRS_TRACE__																				
	gps_tracker_trace(ERR, MOD_MMI,
					"@@@@@@@@@@@@@@@@@@@@@@@ server req %u success!", data->type);
#endif
	return KAL_SUCCESS;
}

void gps_tracker_local_data_upload_timer_proc()
{
	S32 size = 0;
	kal_int32 ret;
		

	
	// ������gps������Ҫ�ϴ�				
	FS_GetFileSize(gps_tracker_local_read_file_hdl, &size);
	
	if(size > 0)
	{					
		S32 read_size;
		S32 want_read_size;
		S32 new_file_size;
		U8 buf[64] = {0};
		U8 i;
		

		//ÿ�ζ�ȡ10����
		if(size < GPS_PACKET_SIZE*10)
		{
			want_read_size = size;
		}
		else
		{
			want_read_size = GPS_PACKET_SIZE*10;
		}

		//��������ļ��Ĵ�С
		new_file_size = size - want_read_size;

		//��ĩβ��ʼ��ȡ����
		FS_Seek( gps_tracker_local_read_file_hdl, new_file_size, FS_FILE_BEGIN );
		
		for(i = 0; i < want_read_size/GPS_PACKET_SIZE; i++)
		{
			FS_Read(gps_tracker_local_read_file_hdl, (void*)buf,GPS_PACKET_SIZE, &read_size);
	
			//�������к�
			*(kal_uint16*)(buf+6) =  gps_tracker_get_sn();

			//����crc
			*(kal_uint16*)(buf+2) = get_crc16(buf+4,GPS_PACKET_SIZE-6);//6 =��ʼ���� + У�鳤�� + ��������					
			
			ret = soc_send(gps_tracker_soc.socketid, buf, GPS_PACKET_SIZE, 0);
#ifdef __GPRS_TRACE__																				
			gps_tracker_trace(WARN, MOD_MMI,"gps local packet:prot_id %u sn %u",
				buf[5], *(U16*)&buf[6]);
#endif

			//����ļ���������Ҫ�򿪵ģ�����Ϊ�����ٶȺܿ죬������Ҫ���õ����޼���ֵ�ͻ�ܸߣ���ᵼ�������߼���Ҫ��
			//���ܳ���ʱ����ܽ������������ﲻ��1. ���յ���Ӧʱ���1.��Ϊ��1ʱ���˱��������Բ���������
			//gps_tracker_send_failed_times ++;

			if(ret != GPS_PACKET_SIZE)
			{
		#ifdef __GPRS_TRACE__																							
				gps_tracker_trace(WARN, MOD_MMI,"send gps local data failed,GPS_PACKET_SIZE %u;send ret %u",
					GPS_PACKET_SIZE, ret);
		#endif

				return ;//����ʧ�ܣ���һ�ζ�ʱ�������³��Է���
			}							
		}

		//todo ������ݶ����ˣ���ɾ���ļ�
		if(new_file_size == 0)
		{
			//�ر��ļ�
			FS_Close(gps_tracker_local_read_file_hdl);

			//ɾ���ļ�
			FS_Delete(L"gps_local_read");
		#ifdef __GPRS_TRACE__																							
			gps_tracker_trace(WARN, MOD_MMI,"@@@@@@@@@@@@@@@@delet gps local read file");
		#endif
			
			//ֹͣ�ϴ���ʱ��
			if(IsMyTimerExist(GPS_TRACKER_LOCAL_DATA_UPLOAD_TIMER) == MMI_TRUE)
			{
				StopTimer(GPS_TRACKER_LOCAL_DATA_UPLOAD_TIMER);
			}			

			return;
		}
		else//todo ���û�ж��꣬�����ļ�����
		{	
			FS_Seek( gps_tracker_local_read_file_hdl, new_file_size, FS_FILE_BEGIN );
			FS_Truncate(gps_tracker_local_read_file_hdl);
		}
		
	}	
	if(IsMyTimerExist(GPS_TRACKER_LOCAL_DATA_UPLOAD_TIMER) != MMI_TRUE)
	{
		StartTimer(GPS_TRACKER_LOCAL_DATA_UPLOAD_TIMER,1000,(FuncPtr)gps_tracker_local_data_upload_timer_proc);
	}
}
/*****************************************************************************
 * FUNCTION
 *  gps_tracker_login_rsp_proc
 * DESCRIPTION
 * ��½��Ϣ�Ĵ����������ݲ�ͬ����Ϣ���ͻظ���ͬ����Ӧ
 * PARAMETERS
 * RETURNS
 *  ������
 *****************************************************************************/
kal_int32 gps_tracker_login_rsp_proc()
{	
	kal_uint32 size = 0;
	kal_uint8 buf[MAX_GT_SEND_LEN] = {0};	
	
	

	//�رյ�½��ʱ������				
	if(IsMyTimerExist(GPS_TRACKER_LOGIN_TIMER) == MMI_TRUE)
	{
		StopTimer(GPS_TRACKER_LOGIN_TIMER);
	#ifdef __GPRS_TRACE__																								
		gps_tracker_trace(WARN, MOD_MMI,"stop GPS_TRACKER_LOGIN_TIMER timer");	
	#endif
	}	

	//�������������ϴ���ʱ��
	if(IsMyTimerExist(GPS_TRACKER_LOCAL_DATA_UPLOAD_TIMER) != MMI_TRUE)
	{		
		//StartTimer(GPS_TRACKER_LOCAL_DATA_UPLOAD_TIMER,1000,(FuncPtr)gps_tracker_local_data_upload_timer_proc);
		// todo �ر�д����ļ�
		// ����Ϊ��ȡ���ļ�
		// ���´����ʹ�д����ļ�			
		FS_GetFileSize(gps_tracker_local_write_file_hdl, &size);

		if(size > 0)
		{
			int ret;
			
			FS_Close(gps_tracker_local_write_file_hdl);

			//�Ƚ����������ļ�������
			ret = FS_Rename(L"gps_local_write", L"gps_local_read");

		#ifdef __GPRS_TRACE__																							
			gps_tracker_trace(ERR, MOD_MMI,
					"FS_Rename ret = %d", ret);
		#endif
			gps_tracker_local_read_file_hdl = FS_Open(L"gps_local_read", FS_CREATE|FS_READ_WRITE);
		#ifdef __GPRS_TRACE__																							
			gps_tracker_trace(ERR, MOD_MMI,
				"gps_tracker_local_read_file_hdl = %d", gps_tracker_local_read_file_hdl);
		#endif
			gps_tracker_local_write_file_hdl = FS_Open(L"gps_local_write", FS_CREATE_ALWAYS|FS_READ_WRITE);

		#ifdef __GPRS_TRACE__																							
			gps_tracker_trace(ERR, MOD_MMI,
				"gps_tracker_local_write_file_hdl = %d", gps_tracker_local_write_file_hdl);
		#endif
		}
	
		
		gps_tracker_local_data_upload_timer_proc();
	}
		
	if(IsMyTimerExist(GPS_TRACKER_HB_TIMER) != MMI_TRUE)
	{
		//����������ʱ��
		//StartTimer(GPS_TRACKER_HB_TIMER, gps_tracker_config.hb_intv*1000, (FuncPtr)gps_tracker_hb_timer_proc);		
		gps_tracker_hb_timer_proc();
	}
		
	return KAL_SUCCESS;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_rcv_proc
 * DESCRIPTION
 * �յ���վ��Ӧ�󣬸��ݲ�ͬ����Ϣ���ͣ�������Ϣ����
 * PARAMETERS
 * RETURNS
 *  ������
 *****************************************************************************/
kal_int32 gps_tracker_rcv_proc( U8* buf )
{	
	gps_tracker_msg_head_struct* head;
	kal_int32 ret;


	if (buf == NULL)
		return KAL_ERROR;

	head = (gps_tracker_msg_head_struct*)buf;
#ifdef __GPRS_TRACE__																								
	gps_tracker_trace(INFO, MOD_MMI,
		"into gps_tracker_rcv_proc()...,head->prot_type=%d",head->prot_type);
#endif

	
	//������Ӧ
	switch(head->prot_type)
	{
		case EN_GT_PT_LOGIN:						
			//if (head->sn == gps_tracker_login_cb.sn)
			{	
			#ifdef __GPRS_TRACE__																								
				gps_tracker_trace(WARN, MOD_MMI,"login rsp sn ok ");
			#endif
				
				if(gps_tracker_send_failed_times >0)
				{					
				//	gps_tracker_send_failed_times--;
					gps_tracker_send_failed_times = 0;	//zzt.20150805.modify
				}
				gps_tracker_login_times = 0;
				gps_tracker_login_cb.is_responsed = KAL_TRUE;//�յ���Ӧ��	

				gps_tracker_task_init();	//zzt.20150730.move from init
				#ifdef __LED_INDICATE_STATE__
				if(!gps_tracker_gsm_led)
				gps_tracker_open_gsm_work_mode();
				#endif

				if(gps_tracker_gsm_state != EN_GT_GSMS_WORKING)
				{
					//t_rtc rtc;					
					

					
					//����ϵͳʱ��	
					if(is_datetime_updated != KAL_TRUE)
					{
						U8* date_time;
						applib_time_struct date;

					
						date_time = ((U8*)buf + sizeof(gps_tracker_msg_head_struct) - 6);
						
						date.nYear= date_time[0] + YEARFORMATE;
						date.nMonth= date_time[1];
						date.nDay= date_time[2];
						date.nHour= date_time[3];
						date.nMin= date_time[4];
						date.nSec= date_time[5];

						//����汾������Ҫƫ�ƣ���Ϊ����汾�ķ�����ʱ���Ǹ�������ʱ��

#ifdef OVERSEA
						gps_tracker_datetime_offset(&date, gps_tracker_config.time_zone);
#endif
						
						mmi_dt_set_rtc_dt((MYTIME *)&date);

						is_datetime_updated = KAL_TRUE;
					}
					//��ȡϵͳʱ��	
					/*
					RTC_GetTime(&rtc); 
					
					gps_tracker_trace(WARN, MOD_MMI, 
							"datetime %u %u %u %u %u %u", 
							rtc.rtc_year, rtc.rtc_mon, rtc.rtc_day, rtc.rtc_hour, rtc.rtc_min, rtc.rtc_sec);
					*/
					//�����￪�� working��־����Ϊ�����ķ��Ͳ����������Ͼ�Ҫ��working��־�ˣ�����ŵ���󣬻�Ӱ�����Ĳ���
					gps_tracker_gsm_state = EN_GT_GSMS_WORKING;
					
					ret = gps_tracker_login_rsp_proc();
					if(KAL_SUCCESS != ret)
					{
					#ifdef __GPRS_TRACE__																								
						gps_tracker_trace(ERR, MOD_MMI,"login rsp proc failed.");
					#endif
						return ret;
					}					
					
					//�����������ver ��Ϣ
					{
						U16 len;
						U8 buffer[MAX_GT_SEND_LEN] = {0};
						
						// ����cb
						gps_tracker_dev_data_cb.dev_req_packet.content.data.type = EN_GT_DT_VER;
						gps_tracker_dev_data_cb.dev_req_packet.content.data.value_len = MAX_GT_VER_STR_LEN;
						strcpy(gps_tracker_dev_data_cb.dev_req_packet.content.data.value.ver, gps_tracker_config.ver);
					#ifdef __GPRS_TRACE__																								
						gps_tracker_trace(INFO, MOD_MMI,"send ver %s to server", gps_tracker_config.ver);		
					#endif	
						//��ʽ����½��Ϣ
						len = gps_tracker_format_cb_to_buffer(&gps_tracker_dev_data_cb, buffer, MAX_GT_SEND_LEN);
						
						ret = gps_tracker_send_req(buffer, len);
						if(KAL_SUCCESS != ret)
						{
						#ifdef __GPRS_TRACE__																									
							gps_tracker_trace(ERR, MOD_MMI,
									"send req failed!");
						#endif
							return ret;
						}						
					}															
				}
			}
			break;			
		case EN_GT_PT_GPS: 			
			//if (head->sn == gps_tracker_gps_cb.sn)
			{	
			#ifdef __GPS_PACKAGE_BY_QUEUE__
				U8* p_uploaded_data;
				p_uploaded_data = (U8*)DeQueue(&gps_tracker_upload_queue);
				if(p_uploaded_data)
					free_ctrl_buffer(p_uploaded_data);
			#endif

			#ifdef __GPRS_TRACE__																								
				gps_tracker_trace(WARN, MOD_MMI,"gps rsp sn ok ");
			#endif
				if(gps_tracker_send_failed_times >0)
				{					
				//	gps_tracker_send_failed_times--;
					gps_tracker_send_failed_times = 0;	//zzt.20150805.modify
				}
				gps_tracker_gps_cb.is_responsed = KAL_TRUE;
			}
			break;
		case EN_GT_PT_STATUS: 			
			//if (head->sn == gps_tracker_status_cb.sn)
			{	
			#ifdef __GPRS_TRACE__
				gps_tracker_trace(WARN, MOD_MMI,"status rsp sn ok ");
			#endif
				if(gps_tracker_send_failed_times >0)
				{					
				//	gps_tracker_send_failed_times--;
					gps_tracker_send_failed_times = 0;	//zzt.20150805.modify
				}
				gps_tracker_status_cb.is_responsed = KAL_TRUE;
			}
			break;
		case EN_GT_PT_HB: 
			//if (head->sn == gps_tracker_hb_cb.sn)
			{	
		#ifdef __GPRS_TRACE__	
				gps_tracker_trace(WARN, MOD_MMI,"hb rsp sn ok ");
		#endif
				//����������
				if(gps_tracker_hb_failed_times >0)
				{
				//	gps_tracker_hb_failed_times --;
					gps_tracker_hb_failed_times = 0;	//zzt.20150805.modify
				}
				gps_tracker_hb_cb.is_responsed = KAL_TRUE;
			}
			break; 
		case EN_GT_PT_ALARM: 					
			//if (head->sn == gps_tracker_alarm_cb.sn)
			{	
		#ifdef __GPRS_TRACE__	
				gps_tracker_trace(WARN, MOD_MMI,"alarm rsp sn ok ");
		#endif
				if(gps_tracker_send_failed_times >0)
				{					
				//	gps_tracker_send_failed_times--;
					gps_tracker_send_failed_times = 0;	//zzt.20150805.modify
				}
				gps_tracker_alarm_cb.is_responsed = KAL_TRUE;
			}
			break;			
		case EN_GT_PT_DEV_DATA:			
			//if (head->sn == gps_tracker_dev_data_cb.sn)
			{		
				gps_tracker_data_content_struct* data;				
		#ifdef __GPRS_TRACE__
				gps_tracker_trace(WARN, MOD_MMI,"dev_data rsp sn ok ");
		#endif

				if(gps_tracker_send_failed_times > 0)
				{					
				//	gps_tracker_send_failed_times--;
					gps_tracker_send_failed_times = 0;	//zzt.20150805.modify
				}

				gps_tracker_dev_data_cb.is_responsed = KAL_TRUE;	

				data = (gps_tracker_data_content_struct*)((U8*)head + sizeof(gps_tracker_msg_head_struct));					
				ret = gps_tracker_dev_data_rsp_proc(data);
				if(KAL_SUCCESS != ret)
				{
				#ifdef __GPRS_TRACE__
					gps_tracker_trace(ERR, MOD_MMI,"dev_data rsp proc failed.");
				#endif
					return ret;
				}								
			}		
			break;	
		#ifdef __PROTOCOL_CONTROL__
		case EN_GT_PT_CONTROL: 					
			//if (head->sn == gps_tracker_control_cb.sn)
			{	
				gps_tracker_control_data_struct* control_data;
				U8 *currPos=NULL;
			
				if(gps_tracker_send_failed_times >0)
				{					
				//	gps_tracker_send_failed_times--;
					gps_tracker_send_failed_times = 0;	//zzt.20150805.modify
				}
				currPos = (U8*)head + sizeof(gps_tracker_msg_head_struct);
				if(*currPos==0x0d && *(currPos+1)==0x0a)
				{
					gps_tracker_trace(WARN, MOD_MMI,"control rsp sn ok" );
					gps_tracker_control_cb.is_responsed = KAL_TRUE;
				}
				else
				{
					control_data = (gps_tracker_control_data_struct*)((U8*)head + sizeof(gps_tracker_msg_head_struct));
					if(control_data)
				#ifdef __GPRS_TRACE__
					gps_tracker_trace(WARN, MOD_MMI,"server send to control, control_data->addr=%x,control_data->value_len=%d,control_data->value=%d",control_data->addr, control_data->value_len,control_data->value);
				#endif
					switch(control_data->addr)
					{
					#ifdef __GPS_CONTROL_CONNECT__
						case 0x1a:
						case 0x1b:
						{
							gps_send_data_to_control(control_data->value,control_data->value_len,control_data->addr);
				         		gps_tracker_control_recv_flag = 1;
							break;
						}
					#endif	
					#ifdef __GPS_MCU_CONNECT__
						case 0x1c:	//�����ǿ���ָ���·�����
						{
							U8 i;
						#if 1//def __MCU_TRACE__	
							for(i=0; i<control_data->value_len; i++)
							{
								gps_tracker_trace(INFO,MOD_MMI, "value[%d]=%x", i,control_data->value[i]);
							}
						#endif
							gps_tracker_update_mcu_data(control_data->value_len,control_data->value);
							break;
						}	
					#endif						
						default:
							break;
					}
				}
				
			}
			break;	
		#endif
		case EN_GT_PT_SRV_DATA:
			{
				gps_tracker_data_content_struct* data;
				U8 rsp[20] = {0};
				U8 len = 0;
				
				
				data = (gps_tracker_data_content_struct*)((U8*)head + sizeof(gps_tracker_msg_head_struct));	

				//������������
				ret = gps_tracker_srv_data_req_proc(data);
				if(KAL_SUCCESS != ret)
				{
				#ifdef __GPRS_TRACE__
					gps_tracker_trace(ERR, MOD_MMI,"srv_data rsp proc failed.");
				#endif
					return ret;
				}	
				
				//������Ӧ
				len = sizeof(gps_tracker_msg_head_struct);
				memcpy(rsp, head, len);
				rsp[4] = 0;
				rsp[len++] = 0x0d;
				rsp[len++] = 0x0a;
				
				ret = gps_tracker_send_rsp(rsp, len);
				if(KAL_SUCCESS != ret)
				{
				#ifdef __GPRS_TRACE__
					gps_tracker_trace(ERR, MOD_MMI,
							"send req failed!");
				#endif
					return ret;
				}
			}
			break;			
		default:
			break;			
	}						

	return KAL_SUCCESS;
}
/*****************************************************************************
 * FUNCTION
 *  gps_tracker_conn_notify
 * DESCRIPTION
 *  soc_connect �¼��ص�
 * PARAMETERS
 *  msg_ptr ϵͳ�ص����� app_soc_notify_ind_struct���� 
 *  
 * RETURNS
 *  void
 *****************************************************************************/
void gps_tracker_conn_notify(void *msg_ptr)
{
	/*----------------------------------------------------------------*/
	/* Local Variables												  */
	/*----------------------------------------------------------------*/
	app_soc_notify_ind_struct *soc_notify = NULL;
	kal_int32 ret = 0;
	MMI_BOOL is_ready = MMI_FALSE;	

 
	/*----------------------------------------------------------------*/
	/* Code Body													  */
	/*----------------------------------------------------------------*/
	#ifdef __GPRS_TRACE__	
		gps_tracker_trace(INFO, MOD_MMI,"gps_tracker_conn_notify,msg_ptr=%x",msg_ptr);	
	#endif
	
	if(msg_ptr == NULL)
		return;

	soc_notify = (app_soc_notify_ind_struct*) msg_ptr;
	
	switch (soc_notify->event_type)
	{
	case SOC_WRITE:
	#ifdef __GPRS_TRACE__	
		gps_tracker_trace(INFO, MOD_MMI,"SOC_WRITE,soc_id %u", soc_notify->socket_id);	
	#endif

		break;

	case SOC_READ:
	#ifdef __GPRS_TRACE__	
		gps_tracker_trace(INFO, MOD_MMI,"SOC_READ,soc_id %u", soc_notify->socket_id);	
	#endif
		/* Check if socket is ready for reading */
		is_ready = soc_ready_for_read(soc_notify->socket_id);

		if(is_ready)
		{	
			if(gps_tracker_working_stage == EN_GT_WS_GPS)
			{					
				//memset(rcv_buf, 0, MAX_GT_RCV_LEN);
				U8 rcv_buf[MAX_GT_RCV_LEN] = {0};
				
				ret = soc_recv(soc_notify->socket_id, rcv_buf, MAX_GT_RCV_LEN, 0);				
				
				if(ret > 0 )
				{
					U16 len = ret;
					U8* head = rcv_buf;
					U8* tail = NULL;
					U16 i = 0;

					#if 0//def __GPRS_TRACE__			
					gps_tracker_trace(INFO, MOD_MMI, "recv %u bytes", ret);
					if(len > 30)
						len = 30; 
					for(i = 0; i<len; i++)
					{
						gps_tracker_trace(INFO, MOD_MMI, "data=%x", rcv_buf[i]);	
					}
					#endif	
					for(i = 0; i<len-1; i++)
					{
						U8 req[MAX_GT_RCV_MSG_LEN] = {0};
						
						//�Ұ�ͷ 0xffff
						if(rcv_buf[i]==0xff && rcv_buf[i+1]==0xff)
						{
							head = rcv_buf+i;
						}
						else if(rcv_buf[i]==0x0d && rcv_buf[i+1]==0x0a)
						{
							tail = rcv_buf+i+2;//��β

							//��֤���ȺϷ���
							if(tail - head == head[4] + PACKET_FRAME_LEN)
							{
								//�Ϸ�
								memcpy(req, head, tail-head);
							#ifdef __GPRS_TRACE__	
								gps_tracker_trace(WARN, MOD_MMI,
									"receive prot_id = %u; sn = %u", *(req+5), *(U16*)(req+6));
							#endif

								//ret = rcv_msg_array_put_node(req);
								ret = gps_tracker_rcv_proc(req);
								if(KAL_SUCCESS != ret)
								{
									#ifdef __GPRS_TRACE__	
									gps_tracker_trace(ERR, MOD_MMI,
											"gps_tracker_rcv_proc failed");
									#endif
								}
								//�ҵ��Ϸ���β
								head = tail;
							}
							//�����ҽ�β
							i +=1;	//�����1��forѭ���Լ�1 ����Ч�����ƫ��2���ֽڣ�0d0a��			
						}
					}

					//ʣ�µĲ����ֽ������ﴦ��
				}
				else
				{
					#ifdef __GPRS_TRACE__	
					gps_tracker_trace(ERR, MOD_MMI,"recv failed, ret = %d", ret);
					#endif
				}
			}
		}
		else
		{
			#ifdef __GPRS_TRACE__	
			gps_tracker_trace(ERR, MOD_MMI,"soc not ready for read");	
			#endif
		}
		
		break;		
	case SOC_CONNECT:
		#ifdef __GPRS_TRACE__		
		gps_tracker_trace(WARN, MOD_MMI,"SOC_CONNECT,soc_id %u", soc_notify->socket_id);
		#endif

		if(gps_tracker_working_stage == EN_GT_WS_GPS && gps_tracker_soc.socketid == soc_notify->socket_id)
		{		
			#ifdef __GPRS_TRACE__	
			gps_tracker_trace(WARN, MOD_MMI,"GPS SOC_CONNECT,soc_id %u", soc_notify->socket_id);
			#endif

			//�رս�����ʱ������				
			if(IsMyTimerExist(GPS_TRACKER_CONN_TIMER) == MMI_TRUE)
			{
			    StopTimer(GPS_TRACKER_CONN_TIMER);
			}
			
			//ˢ����Ӧ�ļ�����
			//���Ӽ�����
			gps_tracker_conn_times = 0;
			gps_tracker_login_times = 0;
			gps_tracker_send_failed_times = 0;
			gps_tracker_hb_failed_times = 0;
			
			if(IsMyTimerExist(GPS_TRACKER_LOGIN_TIMER) != MMI_TRUE)
			{
				//StartTimer(GPS_TRACKER_LOGIN_TIMER, MAX_GT_LOGIN_INTV, 
				//	(FuncPtr)gps_tracker_login_timer_proc);					
				gps_tracker_login_cb.is_responsed = KAL_FALSE;
				gps_tracker_login_timer_proc();
			}		
		}
		
		break;		
	case SOC_CLOSE:
	#ifdef __GPRS_TRACE__	
		gps_tracker_trace(ERR, MOD_MMI,
					"######################### SOC_CLOSE: soc_id %u",soc_notify->socket_id);	
	#endif
		
		gps_tracker_send_failed_times = 0;	
		gps_tracker_hb_failed_times = 0;
		gps_tracker_conn_times = 0;
		is_need_dns = KAL_TRUE;
		
		if(gps_tracker_gsm_state == EN_GT_GSMS_LOGIN)
		{
			//��½�׶Σ������������ã�����ּ�ʹip����Ҳ��connect �ɹ�����������close ��Ϣ��
			//������½��do nothing
		}
		else
		{
			gps_tracker_login_times = 0;
			
			if(IsMyTimerExist(GPS_TRACKER_CONN_TIMER) != MMI_TRUE)
			{
				#ifdef __GPRS_TRACE__	
				gps_tracker_trace(WARN, MOD_MMI,"@@@@@@@@@@@@@@@@@@@@@@@@@@@XXX-12");
				#endif
				gps_tracker_conn_timer_proc();
			}
		}		
		
		break;
	default:
		#ifdef __GPRS_TRACE__	
		gps_tracker_trace(ERR, MOD_MMI,
					"Unknown soc event %u",soc_notify->event_type);
		#endif
		break;
	}
}


/*****************************************************************************
 * FUNCTION
 *  gps_tracker_send_dev_cfg_req
 * DESCRIPTION
 *  pwd ������
 * PARAMETERS
 *  void *  
 * RETURNS
 *  kal_uint32 ������
 *****************************************************************************/
kal_int32 gps_tracker_send_req(S8* buffer, U16 len)
{
	kal_int32 ret = 0;
	

	if (buffer == NULL)
		return KAL_ERROR;

	
	//���͵�½��Ϣ	
	if(gps_tracker_gsm_state == EN_GT_GSMS_WORKING)
	{
		ret = soc_send(gps_tracker_soc.socketid, buffer, len, 0);
	
		gps_tracker_send_failed_times++;
#ifdef __GPRS_TRACE__			
		gps_tracker_trace(INFO, MOD_MMI,"send req  packet, prot_id %u sn %u", buffer[5], *(U16*)&buffer[6]);	
#endif
	}
	
	gps_tracker_dev_data_cb.is_responsed = KAL_FALSE;
	
	
	if(ret != len)
	{
	#ifdef __GPRS_TRACE__	
		gps_tracker_trace(ERR, MOD_MMI,"send dev_data_req packet failed");		
	#endif
		return EN_GT_EC_SEND_ERR; 
	}

	return KAL_SUCCESS;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_send_dev_cfg_req
 * DESCRIPTION
 *  pwd ������
 * PARAMETERS
 *  void *  
 * RETURNS
 *  kal_uint32 ������
 *****************************************************************************/
kal_int32 gps_tracker_send_rsp(S8* buffer, U16 len)
{
	kal_int32 ret = 0;

	if (buffer == NULL)
		return KAL_ERROR;
	
	//���͵�½��Ϣ	
	if(gps_tracker_gsm_state == EN_GT_GSMS_WORKING)
	{
		ret = soc_send(gps_tracker_soc.socketid, buffer, len, 0);
	#ifdef __GPRS_TRACE__			
		gps_tracker_trace(INFO, MOD_MMI,"send rsp packet");	
	#endif
	}	
	
	if(ret != len)
	{
	#ifdef __GPRS_TRACE__	
		gps_tracker_trace(ERR, MOD_MMI,"send srv_data_rsp packet failed");	
	#endif

		return EN_GT_EC_SEND_ERR; 
	}
	return KAL_SUCCESS;
}


kal_int32 gps_tracker_sms_dev_id(void)
{
	/*----------------------------------------------------------------*/
	/* Local Variables												  */
	/*----------------------------------------------------------------*/
	U16 send_buf[MAX_GT_SMS_CONTENT_LEN] = {0};
	
	/*----------------------------------------------------------------*/
	/* Code Body													  */
	/*----------------------------------------------------------------*/
	mbstowcs((wchar_t*)send_buf, gps_tracker_dev_id, strlen(gps_tracker_dev_id));
	
	gps_tracker_send_sms(gps_tracker_sms_req.number.num_u16, send_buf);			
	
	return KAL_SUCCESS;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_get_loc_proc
 * DESCRIPTION
 *  get loc ������
 * PARAMETERS
 *  void
 *  
 * RETURNS
 *  kal_uint32 ������
 *****************************************************************************/
kal_int32 gps_tracker_nvram_restore(void)
{
	S16 error = NVRAM_WRITE_FAIL;
	gps_tracker_config_struct config = {0};

#ifdef DEBUG_DOMAIN
	U8 domain[] = "wangys2015.oicp.net";
#else
	U8 domain[] = "liabar.com";
#endif
 
	//����Ĭ��ֵ
	sprintf(config.ver, "%s", GT_VER);//�汾��
	config.dev_type = 0x10;//�豸����
	memset(config.admin_num, 0, sizeof(config.admin_num));
	config.pwd = 537049746;
	memset(config.users, 0, sizeof(config.users));

	config.up_intv = 5;		//10;	//zzt.20150825
	config.hb_intv = MAX_GT_HB_INTV;
	config.sms_send_intv = 10;
	
	config.temp_thr = 60;
	config.vibr_thr = 8;
	config.speed_thr = 120;
	
	config.lang = EN_GT_LANG_CN;

#ifdef OVERSEA
	config.time_zone = 0*60;
#else
	config.time_zone = 8*60;
#endif

	//��������
	*(U8*)&config.alarm_switch = 0xff;
	*(U8*)&config.sms_alarm_switch = 0x00;

	//server
	config.server.addr_type = EN_GT_ADT_DOMAIN;
	strcpy(config.server.domain, domain);
	config.server.port = SERVER_PORT;	

#ifdef _WIN32
	config.server.addr_type = EN_GT_ADT_IP;
	//memcpy(config.server.domain, domain, strlen(domain)+1);
	config.server.ip[0] = 42;
	config.server.ip[1] = 121;
	config.server.ip[2] = 125;
	config.server.ip[3] = 128; 
	config.server.port = 9000;
#endif
	
	//apn
	memset(&config.apn, 0, sizeof(config.apn));
	config.apn_prof_id = 0;

	config.defence = 1;
	
	//�������ĺ���
	strcpy(config.sms_center_num,"");

	//imsi
	memset(config.imsi, 0, sizeof(config.imsi));
	
	gps_tracker_trace(ERR, MOD_MMI,"start resore nvram record"); 
	//////////////////////////���� NVRAM ///////////////////////////////////////////////////
	//ver
	WriteRecord(NVRAM_EF_GT_VER_LID, 1, config.ver, 
		sizeof(config.ver), &error);
		
	if (NVRAM_WRITE_SUCCESS != error)
	{
		gps_tracker_trace(ERR, MOD_MMI,"resore nvram record <ver> failed!"); 

		return KAL_ERROR;
	}
	gps_tracker_trace(ERR, MOD_MMI,"NVRAM_EF_GT_VER_LID: %u", sizeof(config.ver));
	

	//dev_type
	WriteRecord(NVRAM_EF_GT_DEV_TYPE_LID, 1, &config.dev_type, 
		sizeof(config.dev_type), &error);
		
	if (NVRAM_WRITE_SUCCESS != error)
	{
		gps_tracker_trace(ERR, MOD_MMI,"resore nvram record <dev_type> failed!"); 

		return KAL_ERROR;
	}
	gps_tracker_trace(ERR, MOD_MMI,"NVRAM_EF_GT_DEV_TYPE_LID: %u", sizeof(config.dev_type));	
	
	//admin_number
	WriteRecord(NVRAM_EF_GT_ADM_NUM_LID, 1, config.admin_num, 
			sizeof(config.admin_num), &error);
		
	if (NVRAM_WRITE_SUCCESS != error)
	{
		gps_tracker_trace(ERR, MOD_MMI,"resore nvram record <admin> failed!");	

		return KAL_ERROR;
	}		
	gps_tracker_trace(ERR, MOD_MMI,"NVRAM_EF_GT_ADM_NUM_LID: %u", sizeof(config.admin_num));
	
	//����
	WriteRecord(NVRAM_EF_GT_PWD_LID, 1, &config.pwd, 
				sizeof(config.pwd), &error);
			
	if (NVRAM_WRITE_SUCCESS != error)
	{
		gps_tracker_trace(ERR, MOD_MMI,"resore nvram record <pwd> failed!"); 

		return KAL_ERROR;
	}	
	gps_tracker_trace(ERR, MOD_MMI,"NVRAM_EF_GT_PWD_LID: %u", sizeof(config.pwd));
	
	//users 
	WriteRecord(NVRAM_EF_GT_USERS_LID, 1, config.users, 
		sizeof(config.users), &error);
	
	if (NVRAM_WRITE_SUCCESS != error)
	{
		gps_tracker_trace(ERR, MOD_MMI,"resore nvram record <users> failed!");	

		return KAL_ERROR;
	}
	gps_tracker_trace(ERR, MOD_MMI,"NVRAM_EF_GT_USERS_LID: %u", sizeof(config.users));
	
	//upload_intv
	WriteRecord(NVRAM_EF_GT_UP_INTV_LID, 1, &config.up_intv, 
		sizeof(config.up_intv), &error);
	
	if (NVRAM_WRITE_SUCCESS != error)
	{
		gps_tracker_trace(ERR, MOD_MMI,"resore nvram record <up-intv> failed!");	

		return KAL_ERROR;
	}
	gps_tracker_trace(ERR, MOD_MMI,"NVRAM_EF_GT_UP_INTV_LID: %u", sizeof(config.up_intv));
	
	//hb intv
	WriteRecord(NVRAM_EF_GT_HB_INTV_LID, 1, &config.hb_intv, 
		sizeof(config.hb_intv), &error);
	
	if (NVRAM_WRITE_SUCCESS != error)
	{
		gps_tracker_trace(ERR, MOD_MMI,"resore nvram record <hb-intv> failed!");	

		return KAL_ERROR;
	}
	gps_tracker_trace(ERR, MOD_MMI,"NVRAM_EF_GT_HB_INTV_LID: %u", sizeof(config.hb_intv));
	
	//sms_alarm intv
	WriteRecord(NVRAM_EF_GT_SMS_INTV_LID, 1, &config.sms_send_intv, 
		sizeof(config.sms_send_intv), &error);
	
	if (NVRAM_WRITE_SUCCESS != error)
	{
		gps_tracker_trace(ERR, MOD_MMI,"resore nvram record <sms-intv> failed!"); 

		return KAL_ERROR;
	}
	gps_tracker_trace(ERR, MOD_MMI,"NVRAM_EF_GT_SMS_INTV_LID: %u", sizeof(config.sms_send_intv));

	WriteRecord(NVRAM_EF_GT_TEMP_THR_LID, 1, &config.temp_thr, 
		sizeof(config.temp_thr), &error);

	if (NVRAM_WRITE_SUCCESS != error)
	{
		gps_tracker_trace(ERR, MOD_MMI,"resore nvram record <temp-thr> failed!");	

		return KAL_ERROR;
	}
	gps_tracker_trace(ERR, MOD_MMI,"NVRAM_EF_GT_TEMP_THR_LID: %u", sizeof(config.temp_thr));
	
	WriteRecord(NVRAM_EF_GT_VIBR_THR_LID, 1, &config.vibr_thr, 
		sizeof(config.vibr_thr), &error);

	if (NVRAM_WRITE_SUCCESS != error)
	{
		gps_tracker_trace(ERR, MOD_MMI,"resore nvram record <vibr-thr> failed!");	

		return KAL_ERROR;
	}	
	gps_tracker_trace(ERR, MOD_MMI,"NVRAM_EF_GT_VIBR_THR_LID: %u", sizeof(config.vibr_thr));
	
	WriteRecord(NVRAM_EF_GT_SPEED_THR_LID, 1, &config.speed_thr, 
		sizeof(config.speed_thr), &error);

	if (NVRAM_WRITE_SUCCESS != error)
	{
		gps_tracker_trace(ERR, MOD_MMI,"resore nvram record <speed-thr> failed!");	

		return KAL_ERROR;
	}
	gps_tracker_trace(ERR, MOD_MMI,"NVRAM_EF_GT_SPEED_THR_LID: %u", sizeof(config.speed_thr));	

	
	WriteRecord(NVRAM_EF_GT_LANG_LID, 1, &config.lang, 
		sizeof(config.lang), &error);

	if (NVRAM_WRITE_SUCCESS != error)
	{
		gps_tracker_trace(ERR, MOD_MMI,"resore nvram record <lang> failed!"); 

		return KAL_ERROR;
	}
	gps_tracker_trace(ERR, MOD_MMI,"NVRAM_EF_GT_LANG_LID: %u", sizeof(config.lang));
	
	WriteRecord(NVRAM_EF_GT_TIME_ZONE_LID, 1, &config.time_zone, 
		sizeof(config.time_zone), &error);

	if (NVRAM_WRITE_SUCCESS != error)
	{
		gps_tracker_trace(ERR, MOD_MMI,"resore nvram record <time_zone> failed!");	

		return KAL_ERROR;
	}
	gps_tracker_trace(ERR, MOD_MMI,"NVRAM_EF_GT_TIME_ZONE_LID: %u", sizeof(config.time_zone));

	
	WriteRecord(NVRAM_EF_GT_ALARM_SWITCH_LID, 1, &config.alarm_switch, 
		sizeof(config.alarm_switch), &error);

	if (NVRAM_WRITE_SUCCESS != error)
	{
		gps_tracker_trace(ERR, MOD_MMI,"resore nvram record <alarm_switch> failed!"); 

		return KAL_ERROR;
	}
	gps_tracker_trace(ERR, MOD_MMI,"NVRAM_EF_GT_ALARM_SWITCH_LID: %u", sizeof(config.alarm_switch));
	
	WriteRecord(NVRAM_EF_GT_SMS_SWITCH_LID, 1, &config.sms_alarm_switch, 
		sizeof(config.sms_alarm_switch), &error);

	if (NVRAM_WRITE_SUCCESS != error)
	{
		gps_tracker_trace(ERR, MOD_MMI,"resore nvram record <sms_alarm_switch> failed!");	

		return KAL_ERROR;
	}
	gps_tracker_trace(ERR, MOD_MMI,"NVRAM_EF_GT_SMS_SWITCH_LID: %u", sizeof(config.sms_alarm_switch));

	gps_tracker_trace(ERR, MOD_MMI,"server:%d %s %d.%d.%d.%d %u", config.server.addr_type, config.server.domain, 
		config.server.ip[0],config.server.ip[1], config.server.ip[2], config.server.ip[3], config.server.port);
	
	WriteRecord(NVRAM_EF_GT_SERVER_LID, 1, &config.server, 
		sizeof(config.server), &error);

	if (NVRAM_WRITE_SUCCESS != error)
	{
		gps_tracker_trace(ERR, MOD_MMI,"resore nvram record <server> failed!");	

		return KAL_ERROR;
	}
	gps_tracker_trace(ERR, MOD_MMI,"NVRAM_EF_GT_SERVER_LID: %u", sizeof(config.server));
	

	//apn
	WriteRecord(NVRAM_EF_GT_APN_LID, 1, &config.apn, 
		sizeof(config.apn), &error);

	if (NVRAM_WRITE_SUCCESS != error)
	{
		gps_tracker_trace(ERR, MOD_MMI,"resore nvram record <apn> failed!");	

		return KAL_ERROR;
	}
	gps_tracker_trace(ERR, MOD_MMI,"NVRAM_EF_GT_APN_LID: %u", sizeof(config.apn));
	
	//defence
	WriteRecord(NVRAM_EF_GT_DEFENCE_LID, 1, &config.defence, 
		sizeof(config.defence), &error);

	if (NVRAM_WRITE_SUCCESS != error)
	{
		gps_tracker_trace(ERR, MOD_MMI,"resore nvram record <defence> failed!");	

		return KAL_ERROR;
	}
	gps_tracker_trace(ERR, MOD_MMI,"NVRAM_EF_GT_DEFENCE_LID: %u", sizeof(config.defence));

	
	//sms_center num
	WriteRecord(NVRAM_EF_GT_SMS_CENTER_NUM_LID, 1, config.sms_center_num, 
		sizeof(config.sms_center_num), &error);

	if (NVRAM_WRITE_SUCCESS != error)
	{
		gps_tracker_trace(ERR, MOD_MMI,"resore nvram record <sms-center> failed!");	

		return KAL_ERROR;
	}
	gps_tracker_trace(ERR, MOD_MMI,"NVRAM_EF_GT_SMS_CENTER_NUM_LID: %u", sizeof(config.sms_center_num));
	
	//imsi
	WriteRecord(NVRAM_EF_GT_IMSI_LID, 1, config.imsi, 
		sizeof(config.imsi), &error);

	if (NVRAM_WRITE_SUCCESS != error)
	{
		gps_tracker_trace(ERR, MOD_MMI,"resore nvram record <imsi> failed!");	

		return KAL_ERROR;
	}
	gps_tracker_trace(ERR, MOD_MMI,"NVRAM_EF_GT_IMSI_LID: %u", sizeof(config.imsi));
	
	return KAL_SUCCESS;

}
/*****************************************************************************
 * FUNCTION
 *  gps_tracker_reboot_proc
 * DESCRIPTION
 *  log-level ������
 * PARAMETERS
 *  void
 *  
 * RETURNS
 *  kal_uint32 ������
 *****************************************************************************/
kal_int32 gps_tracker_shutdown_proc(void)
{
	/*----------------------------------------------------------------*/
	/* Local Variables												  */
	/*----------------------------------------------------------------*/
	U16 send_buf[MAX_GT_SMS_CONTENT_LEN] = {0};
	U8 tmp_buf[MAX_GT_SMS_CONTENT_LEN] = {0};
	
	/*----------------------------------------------------------------*/
	/* Code Body													  */
	/*----------------------------------------------------------------*/
	//���ݲ�ͬ�Ķ������ͻظ���ͬ������
	if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
	{
		kal_sprintf(tmp_buf, "<shutdown> success.");
	}
	else
	{
		kal_sprintf((char*)tmp_buf, (char*)"<�ر�> �ɹ���");		
	}
	
	app_asc_str_to_ucs2_wcs((kal_uint8 *)send_buf, (kal_uint8 *)tmp_buf);

	gps_tracker_send_sms(gps_tracker_sms_req.number.num_u16, send_buf);
	/*srv_sms_send_ucs2_text_msg((S8*)send_buf,
		(S8*)gps_tracker_sms_req.number.num_u16,
		gps_tracker_sms_req.sim_id,
		NULL,
		NULL);	*/				
	
	return KAL_SUCCESS;
}
kal_int32 gps_tracker_sms_para(void)
{
	/*----------------------------------------------------------------*/
	/* Local Variables												  */
	/*----------------------------------------------------------------*/
	U16 send_buf[MAX_GT_SMS_CONTENT_LEN] = {0};
	
	/*----------------------------------------------------------------*/
	/* Code Body													  */
	/*----------------------------------------------------------------*/
	switch(gps_tracker_sms_req.para.para_index)
	{
		case EN_GT_PI_ADMIN_NUM:
			mbstowcs((wchar_t*)send_buf, gps_tracker_config.admin_num, strlen(gps_tracker_config.admin_num));
			break;
		case EN_GT_PI_PWD:
			kal_wsprintf(send_buf, "%u",	gps_tracker_config.pwd);
			break;
		case EN_GT_PI_USER:
			kal_wsprintf(send_buf, "1:%s 2:%s 3:%s 4:%s",	gps_tracker_config.users[0], gps_tracker_config.users[1],
				gps_tracker_config.users[2], gps_tracker_config.users[3]);
			break;
		case EN_GT_PI_UP_INTV:
			kal_wsprintf(send_buf, "%u",	gps_tracker_config.up_intv);
			break;
		case EN_GT_PI_HB_INTV:
			kal_wsprintf(send_buf, "%u",	gps_tracker_config.hb_intv);
			break;
		case EN_GT_PI_SMS_ALARM_INTV:
			kal_wsprintf(send_buf, "%u",	gps_tracker_config.sms_send_intv);
			break;
		case EN_GT_PI_TEMP_THR:
			kal_wsprintf(send_buf, "%u",	gps_tracker_config.temp_thr);
			break;
		case EN_GT_PI_VIBR_THR:
			kal_wsprintf(send_buf, "%u",	gps_tracker_config.vibr_thr);
			break;
		case EN_GT_PI_SPEED_THR:
			kal_wsprintf(send_buf, "%u",	gps_tracker_config.speed_thr);
			break;
		case EN_GT_PI_LANG:
			kal_wsprintf(send_buf, "%u",	gps_tracker_config.lang);
			break;
		case EN_GT_PI_TIME_ZONE:
			kal_wsprintf(send_buf, "%u",	gps_tracker_config.time_zone);
			break;
		case EN_GT_PI_PWR_LOW_SWITCH:
			kal_wsprintf(send_buf, "%u",	gps_tracker_config.alarm_switch.pwr_low);
			break;
		case EN_GT_PI_PWR_OFF_SWITCH:
			kal_wsprintf(send_buf, "%u",	gps_tracker_config.alarm_switch.pwr_off);
			break;
		case EN_GT_PI_VIBR_SWITCH:
			kal_wsprintf(send_buf, "%u",	gps_tracker_config.alarm_switch.vibr);
			break;
		case EN_GT_PI_OIL_PWR_SWITCH:
			kal_wsprintf(send_buf, "%u",	gps_tracker_config.alarm_switch.oil_pwr);
			break;
		case EN_GT_PI_SPEED_SWITCH:
			kal_wsprintf(send_buf, "%u",	gps_tracker_config.alarm_switch.speed);
			break;
		case EN_GT_PI_SMS_PWR_LOW_SWITCH:
			kal_wsprintf(send_buf, "%u",	gps_tracker_config.sms_alarm_switch.pwr_low);
			break;
		case EN_GT_PI_SMS_PWR_OFF_SWITCH:
			kal_wsprintf(send_buf, "%u",	gps_tracker_config.sms_alarm_switch.pwr_off);
			break;
		case EN_GT_PI_SMS_VIBR_SWITCH:
			kal_wsprintf(send_buf, "%u",	gps_tracker_config.sms_alarm_switch.vibr);
			break;
		case EN_GT_PI_SMS_OIL_PWR_SWITCH:
			kal_wsprintf(send_buf, "%u",	gps_tracker_config.sms_alarm_switch.oil_pwr);
			break;
		case EN_GT_PI_SMS_SPEED_SWITCH:
			kal_wsprintf(send_buf, "%u",	gps_tracker_config.sms_alarm_switch.speed);
			break;
		case EN_GT_PI_SERVER:
			kal_wsprintf(send_buf, "%u %s %u.%u.%u.%u %u",	gps_tracker_config.server.addr_type, gps_tracker_config.server.domain,
				gps_tracker_config.server.ip[0], gps_tracker_config.server.ip[1], gps_tracker_config.server.ip[2], gps_tracker_config.server.ip[3],
				gps_tracker_config.server.port);
			break;
		case EN_GT_PI_APN:
			kal_wsprintf(send_buf,"net_name %s user_name %s pwd %s %u.%u.%u.%u %u %s", gps_tracker_config.apn.apn_name,
				gps_tracker_config.apn.user_name, gps_tracker_config.apn.passwd, 
				gps_tracker_config.apn.px_addr[0], gps_tracker_config.apn.px_addr[1], 
				gps_tracker_config.apn.px_addr[2], gps_tracker_config.apn.px_addr[3],
				gps_tracker_config.apn.px_port, gps_tracker_config.apn.user_data);
			break;
		case EN_GT_PI_DEFENCE:
			kal_wsprintf(send_buf, "%u",	gps_tracker_config.defence);
			break;
		case EN_GT_PI_SMS_CENTER_NUM:
			mbstowcs((wchar_t*)send_buf, gps_tracker_config.sms_center_num, strlen(gps_tracker_config.sms_center_num));
			break;
		case EN_GT_PI_VER:
			mbstowcs((wchar_t*)send_buf, gps_tracker_config.ver, strlen(gps_tracker_config.ver));
			break;
		case EN_GT_PI_IMEI:
			mbstowcs((wchar_t*)send_buf, gps_tracker_imei, strlen(gps_tracker_imei));
			break;
		case EN_GT_PI_DEV_ID:
			mbstowcs((wchar_t*)send_buf, gps_tracker_dev_id, strlen(gps_tracker_dev_id));
			break;
		default:
			break;
	}
	
	gps_tracker_send_sms(gps_tracker_sms_req.number.num_u16, send_buf);			
	
	return KAL_SUCCESS;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_sms_proc
 * DESCRIPTION
 *  ������ȡ��λ��ַ��Ϣ
 * PARAMETERS
 *  void
 *  
 * RETURNS
 *  void
 *****************************************************************************/
kal_uint32 gps_tracker_sms_proc(void)
{
	/*----------------------------------------------------------------*/
	/* Local Variables												  */
	/*----------------------------------------------------------------*/
	U32 ret;
	U16 len;
	U8 buffer[MAX_GT_SEND_LEN] = {0};
 	S16 error;
	U16 send_buf[MAX_GT_SMS_CONTENT_LEN] = {0};
	U8 tmp_buf[MAX_GT_SMS_CONTENT_LEN*2] = {0};
	
	/*----------------------------------------------------------------*/
	/* Code Body													  */
	/*----------------------------------------------------------------*/
	/*��Ҫ 
	�û����ŵĴ������� :
	1) ���±��ز���
	2) �ϱ�������������
	*/
#ifdef __SMS_TRACE__
	gps_tracker_trace(INFO, MOD_MMI, "cmd %u", gps_tracker_sms_req.cmd_id);
#endif

	gps_tracker_dev_data_cb.is_updated = KAL_FALSE;

	memset(&gps_tracker_dev_data_cb.dev_req_packet.content.data, 0, sizeof(gps_tracker_dev_data_cb.dev_req_packet.content.data));
	
	switch(gps_tracker_sms_req.cmd_id)
	{	
		case EN_GT_SMS_CMD_ADMIN:
		#ifdef __SMS_TRACE__			
			gps_tracker_trace(INFO, MOD_MMI,
						"EN_GT_SMS_CMD_ADMIN cmd");	
		#endif

			//���±��ز���
			// ����ϵͳȫ�ֱ���
			strcpy(gps_tracker_config.admin_num, gps_tracker_sms_req.number.num_s8);

			// ���½� NVRAM 
			error = NVRAM_WRITE_FAIL;
		
			WriteRecord(NVRAM_EF_GT_ADM_NUM_LID, 1, gps_tracker_config.admin_num, 
				sizeof(gps_tracker_config.admin_num), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
			#ifdef __SMS_TRACE__			
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <admin_num> failed! admin number:%s",
					gps_tracker_config.admin_num);	
			#endif

				return EN_GT_EC_NVRAM_OPT_ERR;
			}		
	
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf((char*)tmp_buf, "<admin> success.");
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<����Ա> �ɹ���");
			}
			
			// ����cb
			gps_tracker_dev_data_cb.dev_req_packet.content.data.type = EN_GT_DT_ADMIN_NUM;
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value_len = MAX_GT_PHONE_NUM_LEN;
			strcpy(gps_tracker_dev_data_cb.dev_req_packet.content.data.value.admin_number, gps_tracker_sms_req.number.num_s8);
			gps_tracker_dev_data_cb.is_updated = KAL_TRUE;					
			break;
		case EN_GT_SMS_CMD_PWD:
		#ifdef __SMS_TRACE__				
			gps_tracker_trace(INFO, MOD_MMI,
						"EN_GT_SMS_CMD_PWD cmd");	
		#endif
			// ����ϵͳȫ�ֱ���
			gps_tracker_config.pwd = gps_tracker_sms_req.para.new_pwd;

			// ���½� NVRAM	
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_PWD_LID, 1, &gps_tracker_config.pwd, 
				sizeof(gps_tracker_config.pwd), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
		#ifdef __SMS_TRACE__						
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <pwd> failed!");	
		#endif
				return EN_GT_EC_NVRAM_OPT_ERR;
			}
			
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf((char*)tmp_buf, "<pwd> success.");
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<����> �ɹ���");
			}	
			
			gps_tracker_dev_data_cb.dev_req_packet.content.data.type = EN_GT_DT_PWD;
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value_len = sizeof(U32);
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value.pwd = gps_tracker_sms_req.para.new_pwd;
			gps_tracker_dev_data_cb.is_updated = KAL_TRUE;
			break;
		case EN_GT_SMS_CMD_USER:
		#ifdef __SMS_TRACE__				
			gps_tracker_trace(INFO, MOD_MMI,
						"EN_GT_SMS_CMD_USER cmd");	
		#endif
			
			//update the device's local parameter
			// 1 ���ȫ�ֱ���
			if(0 == gps_tracker_sms_req.para.user_index)
			{		
				memset(&gps_tracker_config.users, 0, sizeof(gps_tracker_config.users));		
			}
			else
			{
				memcpy(gps_tracker_config.users[gps_tracker_sms_req.para.user_index-1],
					gps_tracker_sms_req.para.user, sizeof(gps_tracker_sms_req.para.user));
			}
			
			// 2 ����nvram�����user �б�
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_USERS_LID, 1, &gps_tracker_config.users, 
				sizeof(gps_tracker_config.users), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
			#ifdef __SMS_TRACE__			
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <users> failed!");
			#endif

				return EN_GT_EC_NVRAM_OPT_ERR;
			}
			
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf((char*)tmp_buf, "<user> success.");
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<�û�> �ɹ���");
			}
			
			// ����cb
			gps_tracker_dev_data_cb.dev_req_packet.content.data.type = EN_GT_DT_USER;
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value_len = (MAX_GT_USER_COUNT*MAX_GT_PHONE_NUM_LEN);
			if(gps_tracker_sms_req.para.user_index == 0)
			{
				memset(gps_tracker_dev_data_cb.dev_req_packet.content.data.value.users, 0, 
					sizeof(gps_tracker_dev_data_cb.dev_req_packet.content.data.value.users));
			}
			else
			{					
				memcpy(gps_tracker_dev_data_cb.dev_req_packet.content.data.value.users, gps_tracker_config.users,
					sizeof(gps_tracker_dev_data_cb.dev_req_packet.content.data.value.users));
				memcpy(gps_tracker_dev_data_cb.dev_req_packet.content.data.value.users[gps_tracker_sms_req.para.user_index-1], 
					gps_tracker_sms_req.para.user, sizeof(gps_tracker_sms_req.para.user));
			}
			
			gps_tracker_dev_data_cb.is_updated = KAL_TRUE;
			break;
		case EN_GT_SMS_CMD_UP_INTV:
		#ifdef __SMS_TRACE__						
			gps_tracker_trace(INFO, MOD_MMI,
						"EN_GT_SMS_CMD_UP_INTV cmd");
		#endif
			// 1 ��������
			gps_tracker_config.up_intv = gps_tracker_sms_req.para.upload_intv;
			
			// 2 ����nvram�����user �б�
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_UP_INTV_LID, 1, &gps_tracker_config.up_intv, 
				sizeof(gps_tracker_config.up_intv), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
			#ifdef __SMS_TRACE__			
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <up_intv> failed!");	
			#endif

				return EN_GT_EC_NVRAM_OPT_ERR;
			}	

			//׼���ظ�����
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf((char*)tmp_buf, "<up-intv> success.");
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<�ϱ����> �ɹ���");
			}
			
			// ����cb
			gps_tracker_dev_data_cb.dev_req_packet.content.data.type = EN_GT_DT_UP_INTV;
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value_len = sizeof(gps_tracker_sms_req.para.upload_intv);
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value.upload_intv = gps_tracker_sms_req.para.upload_intv;
			gps_tracker_dev_data_cb.is_updated = KAL_TRUE;	
			break;
		case EN_GT_SMS_CMD_HB_INTV:
		#ifdef __SMS_TRACE__				
			gps_tracker_trace(INFO, MOD_MMI,
						"EN_GT_SMS_CMD_HB_INTV cmd");
		#endif

			// 1 ��������
			gps_tracker_config.hb_intv = gps_tracker_sms_req.para.hb_intv;
			
			// 2 ����nvram�����user �б�
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_HB_INTV_LID, 1, &gps_tracker_config.hb_intv, 
				sizeof(gps_tracker_config.hb_intv), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
				#ifdef __SMS_TRACE__				
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <hb_intv> failed!");
				#endif

				return EN_GT_EC_NVRAM_OPT_ERR;
			}

			//׼���ظ�����
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf((char*)tmp_buf, "<hb-intv> success.");
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<�������> �ɹ���");
			}
			
			// ����cb
			gps_tracker_dev_data_cb.dev_req_packet.content.data.type = EN_GT_DT_HB_INTV;
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value_len = sizeof(gps_tracker_sms_req.para.hb_intv);
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value.hb_intv = gps_tracker_sms_req.para.hb_intv;
			gps_tracker_dev_data_cb.is_updated = KAL_TRUE;	
			break;
		case EN_GT_SMS_CMD_SMS_ALARM_INTV:
		#ifdef __SMS_TRACE__				
			gps_tracker_trace(INFO, MOD_MMI,
						"EN_GT_SMS_CMD_ALARM_INTV cmd");
		#endif

			// 1 ��������
			gps_tracker_config.sms_send_intv = gps_tracker_sms_req.para.sms_send_intv;
			
			// 2 ����nvram�����user �б�
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_SMS_INTV_LID, 1, &gps_tracker_config.sms_send_intv, 
				sizeof(gps_tracker_config.sms_send_intv), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
				#ifdef __SMS_TRACE__				
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <sms_send_intv> failed!");	
				#endif

				return EN_GT_EC_NVRAM_OPT_ERR;
			}		
			
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf((char*)tmp_buf, "<sms-intv> success.");
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<���ż��> �ɹ���");
			}
			
			// ����cb
			gps_tracker_dev_data_cb.dev_req_packet.content.data.type = EN_GT_DT_SMS_ALARM_INTV;
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value_len = sizeof(gps_tracker_sms_req.para.sms_send_intv);
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value.sms_send_intv = gps_tracker_sms_req.para.sms_send_intv;
			gps_tracker_dev_data_cb.is_updated = KAL_TRUE;
			break;
		case EN_GT_SMS_CMD_TEMP_THR:
		#ifdef __SMS_TRACE__				
			gps_tracker_trace(INFO, MOD_MMI,
						"EN_GT_SMS_CMD_TEMP_THR cmd");
		#endif
			// 1 ��������
			gps_tracker_config.temp_thr = gps_tracker_sms_req.para.temp_thr;
			
			// 2 ����nvram�����user �б�
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_TEMP_THR_LID, 1, &gps_tracker_config.temp_thr, 
				sizeof(gps_tracker_config.temp_thr), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
		#ifdef __SMS_TRACE__						
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <temp_thr> failed!");	
		#endif

				return EN_GT_EC_NVRAM_OPT_ERR;
			}
			
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf((char*)tmp_buf, "<temp-threshold> success.");
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<�¶�����> �ɹ���");
			}
			
			// ����cb
			gps_tracker_dev_data_cb.dev_req_packet.content.data.type = EN_GT_DT_TEMP_THR;
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value_len = sizeof(gps_tracker_sms_req.para.temp_thr);
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value.temp_thr = gps_tracker_sms_req.para.temp_thr;
			gps_tracker_dev_data_cb.is_updated = KAL_TRUE;	
			break;
		case EN_GT_SMS_CMD_VIBR_THR:
		#ifdef __SMS_TRACE__						
			gps_tracker_trace(INFO, MOD_MMI,
						"EN_GT_SMS_CMD_VIBR_THR cmd");	
		#endif

			// 1 ��������
			gps_tracker_config.vibr_thr = gps_tracker_sms_req.para.vibr_thr;
			
			// 2 ����nvram�����user �б�
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_VIBR_THR_LID, 1, &gps_tracker_config.vibr_thr, 
				sizeof(gps_tracker_config.vibr_thr), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
		#ifdef __SMS_TRACE__									
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <vibr_thr> failed!");	
		#endif

				return EN_GT_EC_NVRAM_OPT_ERR;
			}	
			
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf((char*)tmp_buf, "<vibr-threshold> success.");
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<������> �ɹ���");
			}
			
			// ����cb
			gps_tracker_dev_data_cb.dev_req_packet.content.data.type = EN_GT_DT_VIBR_THR;
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value_len = sizeof(gps_tracker_sms_req.para.vibr_thr);
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value.vibr_thr = gps_tracker_sms_req.para.vibr_thr;
			gps_tracker_dev_data_cb.is_updated = KAL_TRUE;		
			break;
		case EN_GT_SMS_CMD_SPEED_THR:
		#ifdef __SMS_TRACE__									
			gps_tracker_trace(INFO, MOD_MMI,
						"EN_GT_SMS_CMD_SPEED_THR cmd");	
		#endif
			// 1 ��������
			gps_tracker_config.speed_thr = gps_tracker_sms_req.para.speed_thr;
			
			// 2 ����nvram�����user �б�
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_SPEED_THR_LID, 1, &gps_tracker_config.speed_thr, 
				sizeof(gps_tracker_config.speed_thr), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
		#ifdef __SMS_TRACE__									
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <speed_thr> failed!");	
		#endif

				return EN_GT_EC_NVRAM_OPT_ERR;
			}	
	
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf((char*)tmp_buf, "<speed-threshold> success.");
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<�ٶ�����> �ɹ���");
			}
			
			// ����cb
			gps_tracker_dev_data_cb.dev_req_packet.content.data.type = EN_GT_DT_SPEED_THR;
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value_len = sizeof(gps_tracker_sms_req.para.speed_thr);
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value.speed_thr = gps_tracker_sms_req.para.speed_thr;
			gps_tracker_dev_data_cb.is_updated = KAL_TRUE;	
			break;
		case EN_GT_SMS_CMD_LOC:
		#ifdef __SMS_TRACE__									
			gps_tracker_trace(WARN, MOD_MMI, "EN_GT_SMS_CMD_LOC cmd");	
		#endif
			
			//���� cb��׼���ϱ�
			gps_tracker_dev_data_cb.dev_req_packet.content.data.type = EN_GT_DT_LOC;
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value_len = sizeof(gps_tracker_sms_req.para.loc_lang_type);
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value.loc_lang_type = gps_tracker_sms_req.para.loc_lang_type;
			gps_tracker_dev_data_cb.is_updated = KAL_TRUE;	
			break;
		case EN_GT_SMS_CMD_CELL_INFO:
		#ifdef __SMS_TRACE__							
			gps_tracker_trace(INFO, MOD_MMI,
						"EN_GT_SMS_CMD_CELL_INFO cmd");
		#endif

			//���ݲ�ͬ�Ķ������ͻظ���ͬ������
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf(tmp_buf, "<cell> success.mmc %u,mnc %u,lac %u,cell_id %u", 
					gps_tracker_cell.mcc, gps_tracker_cell.mnc, gps_tracker_cell.lac_sid,
					gps_tracker_cell.cellid_nid);
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<��վ> �ɹ��� ���� %u����Ӫ�� %u��С�� %u����վ %u",
					gps_tracker_cell.mcc, gps_tracker_cell.mnc, gps_tracker_cell.lac_sid,
					gps_tracker_cell.cellid_nid);
				
			}
		#ifdef __SMS_TRACE__						
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@EN_GT_SMS_CMD_CELL_INFO %u OK", EN_GT_SMS_CMD_CELL_INFO);
		#endif
			break;	
		case EN_GT_SMS_CMD_LANG:	
		#ifdef __SMS_TRACE__							
			gps_tracker_trace(INFO, MOD_MMI,
						"EN_GT_SMS_CMD_LANG cmd");	
		#endif
			// 1 ��������
			gps_tracker_config.lang = gps_tracker_sms_req.para.lang;
			
			// 2 ����nvram
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_LANG_LID, 1, &gps_tracker_config.lang, 
				sizeof(gps_tracker_config.lang), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
				#ifdef __SMS_TRACE__							
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <lang> failed!");	
				#endif

				return EN_GT_EC_NVRAM_OPT_ERR;
			}	
	
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf((char*)tmp_buf, "<lang> success.");
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<����> �ɹ���");
			}
			
			// ����cb
			gps_tracker_dev_data_cb.dev_req_packet.content.data.type = EN_GT_DT_LANG;
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value_len = sizeof(gps_tracker_sms_req.para.lang);
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value.lang = gps_tracker_sms_req.para.lang;
			gps_tracker_dev_data_cb.is_updated = KAL_TRUE;	
			break;			
		case EN_GT_SMS_CMD_LOG_LEVEL:	
		#ifdef __SMS_TRACE__									
			gps_tracker_trace(INFO, MOD_MMI,
						"EN_GT_SMS_CMD_LOG_LEVEL cmd");
		#endif

			gps_tracker_log_level = gps_tracker_sms_req.para.log_level;
	
			//���ݲ�ͬ�Ķ������ͻظ���ͬ������
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf(tmp_buf, "<log-level> success.");
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<��־����> �ɹ���");		
			}	
		#ifdef __SMS_TRACE__							
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@EN_GT_SMS_CMD_LOG_LEVEL %u OK", EN_GT_SMS_CMD_LOG_LEVEL);
		#endif
			break;			
		case EN_GT_SMS_CMD_TIME_ZONE:
			//����ʱ��
		#ifdef __SMS_TRACE__									
			gps_tracker_trace(INFO, MOD_MMI,
						"EN_GT_SMS_CMD_TIME_ZONE cmd");
		#endif

			// 1 ��������
			gps_tracker_config.time_zone = gps_tracker_sms_req.para.time_zone;
			
			// 2 ����nvram�����user �б�
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_TIME_ZONE_LID, 1, &gps_tracker_config.time_zone, 
				sizeof(gps_tracker_config.time_zone), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
		#ifdef __SMS_TRACE__												
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <time-zone> failed!");	
		#endif

				return EN_GT_EC_NVRAM_OPT_ERR;
			}	
	
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf((char*)tmp_buf, "<time-zone> success.");
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<ʱ��> �ɹ���");
			}

			if(IsMyTimerExist(GPS_TRACKER_SMS_DELAY_REPLAY_TIMER) != MMI_TRUE)
			{
		#ifdef __SMS_TRACE__												
				gps_tracker_trace(ERR, MOD_MMI, "delay reset...");
		#endif
				
				StartTimer(GPS_TRACKER_SMS_DELAY_REPLAY_TIMER, MAX_GT_SMS_DELAY_REPLY_TIME, 
						(FuncPtr)gps_tracker_sms_delay_rst_proc);
			}
			
			// ����cb
			gps_tracker_dev_data_cb.dev_req_packet.content.data.type = EN_GT_DT_TIME_ZONE;
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value_len = sizeof(gps_tracker_sms_req.para.time_zone);
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value.time_zone = gps_tracker_sms_req.para.time_zone;
			gps_tracker_dev_data_cb.is_updated = KAL_TRUE;			
			break;	
		case EN_GT_SMS_CMD_SHUTDOWN:
			//not support
			break;
		case EN_GT_SMS_CMD_RESTORE:
		#ifdef __SMS_TRACE__												
			gps_tracker_trace(INFO, MOD_MMI,
						"EN_GT_SMS_CMD_RESTORE cmd");
		#endif

			ret = gps_tracker_nvram_restore();

			if(KAL_SUCCESS != ret)
			{
		#ifdef __SMS_TRACE__												
				gps_tracker_trace(ERR, MOD_MMI,
						"restore failed");	
		#endif

				return ret;
			}

			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf(tmp_buf, "<restore> success.");
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<�ָ�����> �ɹ���");		
			}
				
			if(IsMyTimerExist(GPS_TRACKER_SMS_DELAY_REPLAY_TIMER) != MMI_TRUE)
			{
		#ifdef __SMS_TRACE__												
				gps_tracker_trace(ERR, MOD_MMI, "delay reset...");
		#endif
				StartTimer(GPS_TRACKER_SMS_DELAY_REPLAY_TIMER, MAX_GT_SMS_DELAY_REPLY_TIME, 
						(FuncPtr)gps_tracker_sms_delay_rst_proc);
			}
			
			// ����cb
			gps_tracker_dev_data_cb.dev_req_packet.content.data.type = EN_GT_DT_RESTORE;
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value_len = 0;
			gps_tracker_dev_data_cb.is_updated = KAL_TRUE;		
			break;
		case EN_GT_SMS_CMD_ALARM_SWITCH:	
		#ifdef __SMS_TRACE__												
			gps_tracker_trace(INFO, MOD_MMI,
						"EN_GT_SMS_CMD_ALARM_SWITCH cmd");
		#endif
			
			*(U8*)&gps_tracker_config.alarm_switch &= (~(1<< gps_tracker_sms_req.para.alarm_switch.type ));
			*(U8*)&gps_tracker_config.alarm_switch |= ( gps_tracker_sms_req.para.alarm_switch.value << gps_tracker_sms_req.para.alarm_switch.type);

			#ifdef __SMS_TRACE__									
			gps_tracker_trace(ERR, MOD_MMI,
					"alarm:pwr_low %u pwr_off %u vibr %u oil_pwr %u speed %u",
					gps_tracker_config.sms_alarm_switch.pwr_low, gps_tracker_config.sms_alarm_switch.pwr_off,
					gps_tracker_config.sms_alarm_switch.vibr, gps_tracker_config.sms_alarm_switch.oil_pwr,
					gps_tracker_config.sms_alarm_switch.speed);
			#endif
			
			// 2 ����nvram�����user �б�
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_ALARM_SWITCH_LID, 1, &gps_tracker_config.alarm_switch, 
				sizeof(gps_tracker_config.alarm_switch), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
		#ifdef __SMS_TRACE__												
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <alarm-switch> failed!");	
		#endif

				return EN_GT_EC_NVRAM_OPT_ERR;
			}	
	
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf((char*)tmp_buf, "<alarm-switch> success.");
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<�澯����> �ɹ���");
			}
			
			// ����cb
			gps_tracker_dev_data_cb.dev_req_packet.content.data.type = EN_GT_DT_ALARM_SWITCH;
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value_len = sizeof(gps_tracker_sms_req.para.alarm_switch);

			memcpy(&gps_tracker_dev_data_cb.dev_req_packet.content.data.value.alarm_switch, 
				&gps_tracker_sms_req.para.alarm_switch, sizeof(gps_tracker_sms_req.para.alarm_switch));
			
			gps_tracker_dev_data_cb.is_updated = KAL_TRUE;		
			break;	
		case EN_GT_SMS_CMD_SMS_SWITCH:
		#ifdef __SMS_TRACE__													
			gps_tracker_trace(INFO, MOD_MMI,
						"EN_GT_SMS_CMD_SMS_SWITCH cmd");
		#endif

			*(U8*)&gps_tracker_config.sms_alarm_switch &= (~(1 << gps_tracker_sms_req.para.sms_alarm_switch.type));
			*(U8*)&gps_tracker_config.sms_alarm_switch |= (gps_tracker_sms_req.para.sms_alarm_switch.value << gps_tracker_sms_req.para.sms_alarm_switch.type);
			
			// 2 ����nvram
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_SMS_SWITCH_LID, 1, &gps_tracker_config.sms_alarm_switch, 
				sizeof(gps_tracker_config.sms_alarm_switch), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
				#ifdef __SMS_TRACE__														
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <sms-switch> failed!");	
				#endif

				return EN_GT_EC_NVRAM_OPT_ERR;
			}	
	
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf((char*)tmp_buf, "<sms-switch> success.");
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<���ſ���> �ɹ���");
			}
			
			// ����cb
			gps_tracker_dev_data_cb.dev_req_packet.content.data.type = EN_GT_DT_SMS_ALARM_SWITCH;
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value_len = sizeof(gps_tracker_sms_req.para.sms_alarm_switch);
			memcpy(&gps_tracker_dev_data_cb.dev_req_packet.content.data.value.sms_alarm_switch, 
				&gps_tracker_sms_req.para.sms_alarm_switch, sizeof(gps_tracker_sms_req.para.sms_alarm_switch));
			gps_tracker_dev_data_cb.is_updated = KAL_TRUE;		
			break;	
		case EN_GT_SMS_CMD_IGNORE_ALARM:	
		#ifdef __SMS_TRACE__													
			gps_tracker_trace(INFO, MOD_MMI, "EN_GT_SMS_CMD_IGNORE_ALARM cmd");
		#endif

			switch(gps_tracker_sms_req.para.ignore_alarm)
			{
				case EN_GT_AT_PWR_LOW:
					if(IsMyTimerExist(GPS_TRACKER_ALARM_IGNORE_PWR_LOW_TIMER) != MMI_TRUE)
					{
					#ifdef __SMS_TRACE__													
						gps_tracker_trace(ERR, MOD_MMI, "start GPS_TRACKER_ALARM_IGNORE_PWR_LOW_TIMER timer");
					#endif
						StartTimer(GPS_TRACKER_ALARM_IGNORE_PWR_LOW_TIMER,10*60*1000, (FuncPtr)do_nothing);
					}
					break;
				case EN_GT_AT_PWR_OFF:
					if(IsMyTimerExist(GPS_TRACKER_ALARM_IGNORE_PWR_OFF_TIMER) != MMI_TRUE)
					{
						#ifdef __SMS_TRACE__														
						gps_tracker_trace(ERR, MOD_MMI, "start GPS_TRACKER_ALARM_IGNORE_PWR_OFF_TIMER timer");
						#endif
						StartTimer(GPS_TRACKER_ALARM_IGNORE_PWR_OFF_TIMER,10*60*1000,(FuncPtr)do_nothing);
					}
					break;
				case EN_GT_AT_VIBR:
					if(IsMyTimerExist(GPS_TRACKER_ALARM_IGNORE_VIBR_TIMER) != MMI_TRUE)
					{
					#ifdef __SMS_TRACE__													
						gps_tracker_trace(ERR, MOD_MMI, "start GPS_TRACKER_ALARM_IGNORE_VIBR_TIMER timer");
					#endif
						StartTimer(GPS_TRACKER_ALARM_IGNORE_VIBR_TIMER,10*60*1000,(FuncPtr)do_nothing);
					}
					break;

				case EN_GT_AT_OIL_PWR:
					if(IsMyTimerExist(GPS_TRACKER_ALARM_IGNORE_PWR_OIL_TIMER) != MMI_TRUE)
					{
						#ifdef __SMS_TRACE__														
						gps_tracker_trace(ERR, MOD_MMI, "start GPS_TRACKER_ALARM_IGNORE_PWR_OIL_TIMER timer");
						#endif
						StartTimer(GPS_TRACKER_ALARM_IGNORE_PWR_OIL_TIMER,10*60*1000,(FuncPtr)do_nothing);
					}
					break;
				case EN_GT_AT_SPEED:
					if(IsMyTimerExist(GPS_TRACKER_ALARM_IGNORE_SPEED_TIMER) != MMI_TRUE)
					{
					#ifdef __SMS_TRACE__													
						gps_tracker_trace(ERR, MOD_MMI, "start GPS_TRACKER_ALARM_IGNORE_SPEED_TIMER timer");
					#endif
						StartTimer(GPS_TRACKER_ALARM_IGNORE_SPEED_TIMER, 10*60*1000,(FuncPtr)do_nothing);
					}
					break;
				default:
					break;
			}	
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf((char*)tmp_buf, "<ignore-alarm> success.");
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<���Ը澯> �ɹ���");
			}
			
			// ����cb
			gps_tracker_dev_data_cb.dev_req_packet.content.data.type = EN_GT_DT_IGNORE_ALARM;
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value_len = sizeof(gps_tracker_sms_req.para.ignore_alarm);
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value.ignore_alarm = gps_tracker_sms_req.para.ignore_alarm;
			gps_tracker_dev_data_cb.is_updated = KAL_TRUE;				
			break;				
		case EN_GT_SMS_CMD_SERVER:		
			#ifdef __SMS_TRACE__														
			gps_tracker_trace(INFO, MOD_MMI,
						"EN_GT_SMS_CMD_SERVER cmd");
			#endif
			//������Ҫ��ʱ��û�������������Ҳ�ɹ���������Ҫ����ɸ��²��������ϴ����ø���
			//���ﲻ�����忽������Ϊip ��ʽ�İ�����û���������ᵼ����ipʧ��ʱתDNS ��DNS��ʧ�ܡ�
			gps_tracker_config.server.addr_type = gps_tracker_sms_req.para.server.addr_type;

			if(gps_tracker_config.server.addr_type == EN_GT_ADT_IP)
			{
				memcpy(gps_tracker_config.server.ip, gps_tracker_sms_req.para.server.ip, sizeof(gps_tracker_config.server.ip));
			}
			else
			{
				memcpy(gps_tracker_config.server.domain, gps_tracker_sms_req.para.server.domain, sizeof(gps_tracker_config.server.domain));
			}

			gps_tracker_config.server.port = gps_tracker_sms_req.para.server.port;
			
			//memcpy(&gps_tracker_config.server, &gps_tracker_sms_req.para.server, sizeof(gps_tracker_config.server));

			// ���µ�nvram
			WriteRecord(NVRAM_EF_GT_SERVER_LID, 1, &gps_tracker_config.server, 
				sizeof(gps_tracker_config.server), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
			#ifdef __SMS_TRACE__															
				gps_tracker_trace(ERR, MOD_MMI,"write nvram record <server> failed!");	
			#endif
				return KAL_ERROR;
			}
/*
			//ͬʱ����agps
			memcpy(&gps_tracker_config.agps_server, &gps_tracker_sms_req.para.server, sizeof(gps_tracker_config.agps_server));

			// ���µ�nvram
			WriteRecord(NVRAM_EF_GT_AGPS_SRV_LID, 1, &gps_tracker_config.agps_server, 
				sizeof(gps_tracker_config.agps_server), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
				gps_tracker_trace(ERR, MOD_MMI,"write nvram record <agps> failed!");	

				return KAL_ERROR;
			}
*/				
			//���ݲ�ͬ�Ķ������ͻظ���ͬ������
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf(tmp_buf, "<server> success.");
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<������> �ɹ���");		
			}		

			//�������ݸ��º�׼������
			if(IsMyTimerExist(GPS_TRACKER_SMS_DELAY_REPLAY_TIMER) != MMI_TRUE)
			{
			#ifdef __SMS_TRACE__														
				gps_tracker_trace(ERR, MOD_MMI, "delay reset...");
			#endif
				StartTimer(GPS_TRACKER_SMS_DELAY_REPLAY_TIMER, MAX_GT_SMS_DELAY_REPLY_TIME, 
						(FuncPtr)gps_tracker_sms_delay_rst_proc);
			}
			
			///////////////////////////////////////////////////////
			// ׼���ϴ����ø���
			// ����cb
			gps_tracker_dev_data_cb.dev_req_packet.content.data.type = EN_GT_DT_SERVER;
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value_len = sizeof(gps_tracker_sms_req.para.server);
			memcpy(&gps_tracker_dev_data_cb.dev_req_packet.content.data.value.server, 
				&gps_tracker_sms_req.para.server, sizeof(gps_tracker_sms_req.para.server));
			gps_tracker_dev_data_cb.is_updated = KAL_TRUE;
			///////////////////////////////////////////////////////	
			#ifdef __SMS_TRACE__																	
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@EN_GT_SMS_CMD_SERVER %u OK", EN_GT_SMS_CMD_SERVER);		
			#endif
			break;
		case EN_GT_SMS_CMD_DEFENCE:	
			#ifdef __SMS_TRACE__																	
			gps_tracker_trace(INFO, MOD_MMI,
						"EN_GT_SMS_CMD_DEFENCE cmd");
			#endif

			// 1 ��������
			gps_tracker_config.defence = gps_tracker_sms_req.para.defence;
			
			// 2 ����nvram�����user �б�
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_DEFENCE_LID, 1, &gps_tracker_config.defence, 
				sizeof(gps_tracker_config.defence), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
			#ifdef __SMS_TRACE__															
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <defence> failed!");	
			#endif

				return EN_GT_EC_NVRAM_OPT_ERR;
			}
			
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf((char*)tmp_buf, "<denfence> success.");
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<����> �ɹ���");
			}
			
			// ����cb
			gps_tracker_dev_data_cb.dev_req_packet.content.data.type = EN_GT_DT_DEFENCE;
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value_len = sizeof(gps_tracker_sms_req.para.defence);
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value.defence = gps_tracker_sms_req.para.defence;
			gps_tracker_dev_data_cb.is_updated = KAL_TRUE;		
			break;	
		case EN_GT_SMS_CMD_SMS_CENTER:
			#ifdef __SMS_TRACE__																		
			gps_tracker_trace(INFO, MOD_MMI,
						"EN_GT_SMS_CMD_SMS_CENTER cmd");
			#endif

			// 1 ��������
			memcpy(gps_tracker_config.sms_center_num, gps_tracker_sms_req.para.sms_center_num, 
				sizeof(gps_tracker_sms_req.para.sms_center_num));
			
			// 2 ����nvram�����user �б�
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_SMS_CENTER_NUM_LID, 1, gps_tracker_config.sms_center_num, 
				sizeof(gps_tracker_config.sms_center_num), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
			#ifdef __SMS_TRACE__															
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <sms-center> failed!");
			#endif

				return EN_GT_EC_NVRAM_OPT_ERR;
			}	
	
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf((char*)tmp_buf, "<sms-center> success.");
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<��������> �ɹ���");
			}
			
			// ����cb
			gps_tracker_dev_data_cb.dev_req_packet.content.data.type = EN_GT_DT_SMS_CENTER;
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value_len = sizeof(gps_tracker_sms_req.para.sms_center_num);
			memcpy(gps_tracker_dev_data_cb.dev_req_packet.content.data.value.sms_center_num, 
				gps_tracker_sms_req.para.sms_center_num, sizeof(gps_tracker_sms_req.para.sms_center_num));
			gps_tracker_dev_data_cb.is_updated = KAL_TRUE;		
			break;	
		case EN_GT_SMS_CMD_VER:	
			#ifdef __SMS_TRACE__																		
			gps_tracker_trace(INFO, MOD_MMI,
						"EN_GT_SMS_CMD_VER cmd");
			#endif

			//���ݲ�ͬ�Ķ������ͻظ���ͬ������
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf(tmp_buf, "<ver> success.ver: %s", gps_tracker_config.ver);
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<�汾> �ɹ����汾��%s", gps_tracker_config.ver);		
			}
		#ifdef __SMS_TRACE__															
			gps_tracker_trace(ERR, MOD_MMI, "EN_GT_SMS_CMD_VER %u OK", EN_GT_SMS_CMD_VER);
		#endif
			break;
		case EN_GT_SMS_CMD_IMEI:
		#ifdef __SMS_TRACE__															
			gps_tracker_trace(INFO, MOD_MMI,
						"EN_GT_SMS_CMD_IMEI cmd");
		#endif

			strcpy(tmp_buf, gps_tracker_imei);
		#ifdef __SMS_TRACE__															
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@  EN_GT_SMS_CMD_IMEI %u OK", EN_GT_SMS_CMD_IMEI);
		#endif
			break;	
		case EN_GT_SMS_CMD_DEV_ID:
			#ifdef __SMS_TRACE__																		
			gps_tracker_trace(INFO, MOD_MMI,
						"EN_GT_SMS_CMD_DEV_ID cmd");
			#endif
			
			strcpy(tmp_buf, gps_tracker_dev_id);
		#ifdef __SMS_TRACE__															
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@  EN_GT_SMS_CMD_DEV_ID %u OK", EN_GT_SMS_CMD_DEV_ID);
		#endif
			break;	
		case EN_GT_SMS_CMD_PARA:
			#ifdef __SMS_TRACE__																		
			gps_tracker_trace(INFO, MOD_MMI,
						"EN_GT_SMS_CMD_PARA cmd");
			#endif
			
			ret = gps_tracker_sms_para();

			if(KAL_SUCCESS != ret)
			{
				return ret;
			}	
			#ifdef __SMS_TRACE__															
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@  EN_GT_SMS_CMD_PARA %u OK", EN_GT_SMS_CMD_PARA);
			#endif
			//����Ҫ���أ���Ϊgps_tracker_sms_para()�����Ѿ��ظ�������
			return KAL_SUCCESS;
			break;	
		case EN_GT_SMS_CMD_PWR_OIL_SWITCH:
			#ifdef __SMS_TRACE__																		
			gps_tracker_trace(ERR, MOD_MMI,
				"EN_GT_SMS_CMD_PWR_OIL_SWITCH cmd,para %u", gps_tracker_sms_req.para.pwr_oil_switch);
			#endif
			if(gps_tracker_sms_req.para.pwr_oil_switch == 0)
			{
				GPIO_WriteIO(1,RELAY_CTL_IO);

				//�͵�Ͽ��澯
				if(IsMyTimerExist(GPS_TRACKER_ALARM_IGNORE_PWR_OIL_TIMER) != MMI_TRUE)
				{
					gps_tracker_alarm.oil_pwr_ind = 1;
				}

				gps_tracker_state.oil_pwr_state = 0;
			}
			else
			{
				GPIO_WriteIO(0,RELAY_CTL_IO);

				gps_tracker_state.oil_pwr_state = 1;
			}			
			//׼���ظ�����
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				sprintf((char*)tmp_buf, "<pwr-oil> success.");
			}
			else
			{
				sprintf((char*)tmp_buf, (char*)"<�͵�> �ɹ���");
			}
			break;
		
		default:
			return EN_GT_EC_INVALID_CMD;
			break;
	}

	//�ظ�����
	if(strlen(tmp_buf) != 0)
	{
		app_asc_str_to_ucs2_wcs((kal_uint8 *)send_buf, (kal_uint8 *)tmp_buf);

		gps_tracker_send_sms(gps_tracker_sms_req.number.num_u16, send_buf);
	}

	//������Ҫ������������͸�������	
	if( gps_tracker_dev_data_cb.is_updated == KAL_TRUE )
	{
		//��ʽ����½��Ϣ
		len = gps_tracker_format_cb_to_buffer(&gps_tracker_dev_data_cb, buffer, MAX_GT_SEND_LEN);

		ret = gps_tracker_send_req(buffer, len);		
		if(KAL_SUCCESS != ret)
		{
			return ret;
		}
	}	
	return KAL_SUCCESS;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_validate_user
 * DESCRIPTION
 *  �������̴�������
 * PARAMETERS
 *  
 * RETURNS
 *  
 *****************************************************************************/

kal_bool gps_tracker_validate_user(void)
{
	kal_int32 i;


	
	if(0 == max_match_r_cmp(gps_tracker_sms_req.number.num_s8, gps_tracker_config.admin_num))
	{
		return KAL_TRUE;
	}
	else
	{
		for(i = 0; i < MAX_GT_USER_COUNT; i++)
		{
			if(0 == max_match_r_cmp(gps_tracker_sms_req.number.num_s8, gps_tracker_config.users[i]))
			{
				return KAL_TRUE;
			}
		}
	}
	
	return KAL_FALSE;
}
/*****************************************************************************
 * FUNCTION
 *  gps_tracker_sms_err_proc
 * DESCRIPTION
 *  �������̴�������
 * PARAMETERS
 *  
 * RETURNS
 *  
 *****************************************************************************/
void gps_tracker_sms_err_proc(kal_int32 err)
{
	U16* err_sms = NULL;

	err_sms = gps_tracker_sms_get_err_desc(err, gps_tracker_sms_req.cmd_lang_type);	

	gps_tracker_send_sms(gps_tracker_sms_req.number.num_u16, err_sms);	
}

kal_int32 ids_sms_msg_handle(sms_node_struct *msg_node)
{
	kal_int32 ret;


	if (msg_node == NULL)
		return KAL_ERROR;

	//����������Ϣ��ȡ�����������ͺ��������		
	ret = gps_tracker_sms_decode(msg_node); 	

	if(ret != KAL_SUCCESS)
	{
		gps_tracker_sms_err_proc(ret);

		return ret;
	}

	// ���ݲ�ͬ�Ķ���������в�ͬ�Ĵ���
	//ret = ids_sms_proc();
	ret = gps_tracker_sms_proc();

	// ��������
	if(ret != KAL_SUCCESS)
	{
		//������Ӧ�������Ѿ��ظ��ˣ����ﲻ��Ҫ�ٶ��Żظ�
		return KAL_ERROR;
	}

	return KAL_SUCCESS;
}


S32 ids_sms_listen(mmi_evt_sms_disp_struct *ev_data)
{
	U8 number[64] = {0};
	S8 s_asc_buf[256];// message content ascii buffer...
	U8 *textBuff = NULL;
	U8 *textBuffShifted = NULL;
	U8 *TPUD[1];
	U8 TPUDLEN[1];
	U8 is_obj_present = 0;
	U16 textLen = 0;
	U16 total_len = 0;
	U8 dcs = 0;
	U16 buffLen = 0;

	smslib_general_struct *app_lib_data = NULL; 
	mmi_sms_new_msg_pdu_ind_struct *data = NULL;

	memset(s_asc_buf,0,sizeof(s_asc_buf));
	if (ev_data == NULL)
	{
		return KAL_ERROR;
	}

	app_lib_data = (smslib_general_struct *)OslMalloc(sizeof(smslib_general_struct)); 
	data = (mmi_sms_new_msg_pdu_ind_struct *)ev_data->data;

	smslib_decode_pdu(data->pdu, (kal_uint8)data->pdu_length, app_lib_data);
	smslib_get_msg_content(KAL_FALSE, app_lib_data, NULL);

	if (app_lib_data->tpdu.msg_len==0)
	{
		OslMfree((void *)app_lib_data);
		return KAL_ERROR;
	}

	if (app_lib_data->tpdu.mti == 2 && app_lib_data->tpdu.msg_class == 4)
	{
		OslMfree((void *)app_lib_data);
		return KAL_ERROR;
	}

	buffLen = app_lib_data->forMMI_UserData_length;

	if (app_lib_data->tpdu.alphabet_type == SMSLIB_GSM7_BIT)
	{
		dcs = SRV_SMS_DCS_7BIT;
	}	
	else if (app_lib_data->tpdu.alphabet_type == SMSLIB_EIGHT_BIT)
	{
		dcs = SRV_SMS_DCS_8BIT;
	}
	else if (app_lib_data->tpdu.alphabet_type == SMSLIB_UCS2)
	{
		dcs = SRV_SMS_DCS_UCS2;
	}

#ifdef __SMS_TRACE__															
	gps_tracker_trace(INFO, MOD_MMI,"buflen:%u", buffLen);
#endif

	if (app_lib_data->tpdu.alphabet_type == SMSLIB_GSM7_BIT)
	{
		buffLen *= 2;
	}

	buffLen = (buffLen < SRV_SMS_MAX_BUFF_SIZE - ENCODING_LENGTH) ? buffLen : (SRV_SMS_MAX_BUFF_SIZE - ENCODING_LENGTH);

#ifdef __SMS_TRACE__															
	gps_tracker_trace(INFO, MOD_MMI,"buflen:%u", buffLen);
#endif

	textBuff = (U8 *)OslMalloc(buffLen);
	textBuffShifted = textBuff;
	{
		TPUDLEN[0] = (U8) app_lib_data->tpdu_len;
		TPUD[0] = (U8*) app_lib_data->forMMI_UserData;
		EMSMsgPreview(
			1,
			dcs,
			(U8)GetUDHIBit(app_lib_data->tpdu.fo),
			TPUD,
			TPUDLEN,
			buffLen,
			textBuffShifted,
			&is_obj_present,
			&textLen);

		total_len += textLen;
		textBuffShifted += textLen;
		buffLen -= textLen;
		textLen = 0;
	}

	if (dcs == SMSAL_UCS2_DCS || dcs == SMSLIB_GSM7_BIT)
	{
		mmi_ucs2cpy((S8*)s_asc_buf, (S8*)textBuff);
	}
	else
	{
		if (strlen((char*)textBuff) >= 256/2)
		{
			OslMfree((void *)app_lib_data);
			OslMfree((void *)textBuff);
			return FALSE;
		}
		else
		{
			mmi_asc_to_ucs2((S8*)s_asc_buf, (S8*)textBuff);
		}
	}

	srv_sms_convert_l4_num_to_ascii_num((U8*)number, &(app_lib_data->forMMI_TPAddr), SRV_SMS_MAX_ADDR_LEN);

	smslib_dealloc_sms_struct(app_lib_data);
	//lwq if(ret > 0)
	{/*
		srv_sms_send_sms_ack(1,0,ev_data->sim_id,(0xFFFF),
			(U8)app_lib_data->tpdu.msg_class,
			(MMI_BOOL)app_lib_data->tpdu.msg_wait.is_msg_wait,
			MMI_FALSE);*/
	}
	OslMfree((void *)app_lib_data);
	OslMfree((void *)textBuff);
	//lwq if(ret == 1)
	{
		srv_sms_delete_msg(ev_data->msg_id, NULL, (void*)NULL);
	}
	{
		sms_node_struct sms_node;

		memset(&sms_node, 0, sizeof(sms_node));

		memcpy(sms_node.number, number, strlen((char*)number));
		memcpy(sms_node.content, s_asc_buf, total_len);
	#ifdef __SMS_TRACE__															
		gps_tracker_trace(INFO, MOD_MMI,"sms number %s content len %u", sms_node.number, total_len);
	#endif
		ids_sms_msg_handle(&sms_node);
	}
	return	KAL_SUCCESS;//ret > 0;
}

S32 gps_tracker_sms_delay_rst_proc()
{
#ifndef _WIN32		
	gps_tracker_restart();
#endif
	return KAL_SUCCESS; 
}
#ifdef __GPS_BAT_CONNECT__
U8 cmd[]={
	GPS_BAT_SUM_VOLTAGE,			//�ܵ�ѹ
	GPS_BAT_CURRENT,				//��ŵ����
	GPS_BAT_DESIGN_CAPABILITY,		//�������
	GPS_BAT_DESIGN_VOLITAGE,		//��Ƶ�ѹ
	GPS_BAT_FCC,						//��ذ��������
	GPS_BAT_REMAIN_CAPABILITY,		//ʣ������
	GPS_BAT_PERCENT_CAPABILITY,	//�����ٷֱ�
	GPS_BAT_CYCLE_COUNT,			//ѭ������
	GPS_BAT_MANUFACTURENAME,		//����������
	GPS_BAT_TEMPERATURE,			//����¶�
	GPS_BAT_STATUS,					//��ذ�״̬
	GPS_BAT_INFO,					//��ذ���Ϣ
	GPS_BAT_BUNCH_NUM,				//��ش���
	GPS_BAT_BAR_CODE1,				//�������1
	GPS_BAT_BAR_CODE2,				//�������2
	GPS_BAT_BAR_CODE3,				//�������3
};
void gps_get_data_from_battery(void) 
{
	static U8 index=0;
	U8 buffer[256]={0};
	U8 len=0,i=0;
	U16 word=0;

	
/*	if(cmd[index] == GPS_BAT_MANUFACTURENAME ||cmd[index] == GPS_BAT_INFO ||cmd[index] ==GPS_BAT_BAR_CODE1 ||cmd[index] ==GPS_BAT_BAR_CODE2 
		||cmd[index] ==GPS_BAT_BAR_CODE3)
	{
		FSL_bat_IICReadBlock(cmd[index],buffer,&len);
		for(i=0; i<len; i++)
		{
			gps_tracker_trace(INFO, MOD_MMI,"block=%x,i=%d",buffer[i],i);
		}
		gps_tracker_trace(INFO, MOD_MMI,"block string=%s",buffer);
	}
	else
	{
		gps_tracker_trace(INFO,MOD_MMI,"word=%d",FSL_bat_IICReadWord(cmd[index]));
	}
	index++;
	if(index >= sizeof(cmd))
		index = 0;*/

	word = FSL_bat_IICReadWord(GPS_BAT_SUM_VOLTAGE);
	if(word != 0xffff)
		battery_info.total_voltage = word;

	word = FSL_bat_IICReadWord(GPS_BAT_REMAIN_CAPABILITY);
	if(word != 0xffff)
		battery_info.remain_capability = word;

	word = FSL_bat_IICReadWord(GPS_BAT_PERCENT_CAPABILITY);
	if(word != 0xffff)
		battery_info.percent_capability = word;

	word = FSL_bat_IICReadWord(GPS_BAT_BUNCH_NUM);
	if(word != 0xffff)
		battery_info.bunch_num = word;

	word = FSL_bat_IICReadWord(GPS_BAT_CYCLE_COUNT);
	if(word != 0xffff)
		battery_info.cycle_count = word;
	
	battery_info.total_voltage = (battery_info.total_voltage*battery_info.bunch_num)/3000;	//�ܵ�ѹ=(�ܵ�ѹ*�ܴ���)/(3��*1000)

#ifdef __BAT_TRACE__
	gps_tracker_trace(INFO,MOD_MMI,"total=%d,remain_cap=%d,percent=%d,bunch_num=%d,cycle=%d",battery_info.total_voltage,battery_info.remain_capability,battery_info.percent_capability,
		battery_info.bunch_num,battery_info.cycle_count);
#endif

	StartTimer(GPS_TRACKER_BATTERY_CONNECT, 1000, gps_get_data_from_battery);
}
#endif

#ifdef __GPS_CONTROL_CONNECT__ 
#define START 0x3A
#define END1 0x0D
#define END2 0x0A
void gps_send_data_to_control(U8* data,U8 data_len,U8 addr)
{
	U8 buf[512]={0};
	U8 checkSumLow,checkSumHigh;
	U8 len,wlen,i;
	U16 checksum=0;

	buf[0] = START;
	buf[1] = addr;
	buf[2] = data_len;
	memcpy(buf+3, data, data_len);

	checksum += buf[1];
	checksum += buf[2];
	for(i=0;i<data_len;i++)
	{
		checksum += *(data+i);
	}
	checkSumHigh = (checksum&0xff00)>>8;
	checkSumLow = checksum&0x00ff;

	buf[3+data_len] = checkSumHigh;
	buf[4+data_len] = checkSumLow;
	buf[5+data_len] = END1;
	buf[6+data_len] = END2;
	len = 7+data_len;

#ifdef __CONTROL_TRACE__
	gps_tracker_trace(INFO, MOD_MMI, "gps_send_data_to_control,len=%x,addr=%x",data_len,addr);
#endif
	
	for(i=0;i<len;i++)
	{
	#ifdef __CONTROL_TRACE__
		gps_tracker_trace(INFO, MOD_MMI, "uart write:buf[%d]=%x,len=%d",i,buf[i],len);
	#endif
	}

//	wlen = gps_tracker_uart_write(buf, len, uart_port1, MOD_MMI);
	U_PutUARTBytes(uart_port1,buf,len);

}

kal_uint8 gps_get_data_from_control(kal_uint8* recvBuf,kal_uint8 bufSize,kal_uint8* address)
{
	U8 buf[512]={0};
	U8 i = 0,j = 0;
	U8 addr;
	U8 data[32];
	U8 checkSumLow,checkSumHigh,end1,end2;
	U16 checkSum = 0;
	kal_uint8 read_length,dataLen;
	kal_uint8 status;
	read_length = gps_tracker_uart_read(buf, 512-1, uart_port1, MOD_MMI);

#ifdef __CONTROL_TRACE__
	gps_tracker_trace(INFO, MOD_MMI, "uart event ,gps_get_data_from_control,read_len=%d",read_length);
#endif
	if(read_length < 7)
	{
#ifdef __CONTROL_TRACE__
		gps_tracker_trace(ERR, MOD_MMI, "uart read data length less than 7,error protocol");
#endif
		return 0;
	}
	
	//decode
	while(buf[i]!=0x3a)
	{
		i++;
	}
	if(i >= read_length)	//not found 0x3a,error
	{
	#ifdef __CONTROL_TRACE__
		gps_tracker_trace(ERR, MOD_MMI, "uart read data error");
	#endif
		return 0;
	}
	
	addr = buf[i+1];
	dataLen = buf[i+2];
	checkSumLow = buf[i+3+dataLen];
	checkSumHigh = buf[i+4+dataLen];
	end1 = buf[i+5+dataLen];
	end2 = buf[i+6+dataLen];
	
#ifdef __CONTROL_TRACE__
	gps_tracker_trace(INFO, MOD_MMI, "uart read data addr=%x,dataLen=%x,low=%x,high=%x,end1=%x,end2=%x",addr,dataLen,checkSumLow,checkSumHigh,end1,end2);
#endif

	if(end1 != 0x0d ||end2 != 0x0a)
	{
	#ifdef __CONTROL_TRACE__
		gps_tracker_trace(ERR, MOD_MMI, "uart read data error");
	#endif
		return 0;
	}
	
	while(j<dataLen)
	{
		data[j]=buf[i+3+j];
//		gps_tracker_trace(INFO, MOD_MMI,"data[%d]=%x",j,data[j]);
		j++;
	}
	
	checkSum += addr;
	checkSum += dataLen;
	for(j=0;j<dataLen;j++)
	{
		checkSum += data[j];
	}
//	gps_tracker_trace(INFO, MOD_MMI, "uart read checksum=%x",checkSum);

	if(!(checkSumLow == checkSum&0x00ff && checkSumHigh == (checkSum&0xff00)>>8))
	{
	#ifdef __CONTROL_TRACE__	
		gps_tracker_trace(ERR, MOD_MMI, "uart read data error,checksum error,checkSum=%x,low=%x,high=%x",checkSum,checkSumLow,checkSumHigh);
	#endif
		return 0;
	}

	*address = addr;
	memcpy(recvBuf,data,dataLen);
	return dataLen;
}
#endif

