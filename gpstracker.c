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
*文件内部的宏定义,如果某些宏只在本c文件使用，请放到这里
***************************************************************/
#define GT_VER "S1.1_H1.1" //每升级一个版本修改此处 sv 软件版本 hv 硬件版本

//海外版本开关
//#define OVERSEA

#define SUPER_PWD 314524776 //666888

void gps_tracker_restart(void);
/**************************************************************
*全局变量定义
***************************************************************/
//系统命令
struct sms_cmd{
	kal_uint32 cmd_id; 
	kal_uint16 cmd_en[MAX_GT_CMD_LEN]; 
	kal_uint16 cmd_zh[MAX_GT_CMD_LEN]; 
}gps_tracker_sms_cmd[] = 
{
	{EN_GT_SMS_CMD_ADMIN, L"admin", {0x7BA1, 0x7406, 0x5458, 0x0000}/*L"管理员"*/},
	{EN_GT_SMS_CMD_PWD, L"pwd", {0x5BC6,0x7801,0x0000}/*L"密码"*/},
	{EN_GT_SMS_CMD_USER, L"user", { 0x7528 ,0x6237,0x0000}/*L"用户"*/},
	{EN_GT_SMS_CMD_UP_INTV, L"up-intv", {0x4E0A,0x62A5,0x95F4,0x9694,0x0000}/*L"上报间隔"*/},
	{EN_GT_SMS_CMD_HB_INTV, L"hb-intv", {0x5FC3,0x8DF3,0x95F4,0x9694,0x0000}/*L"心跳间隔"*/},
	{EN_GT_SMS_CMD_SMS_ALARM_INTV, L"sms-intv", {0x77ED,0x4FE1,0x95F4,0x9694,0x0000}/*L"短信间隔"*/},
	{EN_GT_SMS_CMD_TEMP_THR, L"temp-thr", {0x6E29,0x5EA6,0x95E8,0x9650,0x0000}/*L"温度门限"*/},
	{EN_GT_SMS_CMD_VIBR_THR, L"vibr-thr", {0x9707,0x52A8,0x95E8,0x9650,0x0000}/*L"振动门限"*/},
	{EN_GT_SMS_CMD_SPEED_THR, L"speed-thr", {0x8D85 ,0x901F ,0x95E8 ,0x9650,0x0000}/*L"超速门限"*/},
	{EN_GT_SMS_CMD_LOC, L"loc", {0x4F4D,0x7F6E,0x0000}/*L"位置"*/},
	{EN_GT_SMS_CMD_CELL_INFO, L"cell", {0x57FA,0x7AD9,0x0000}/*L"基站"*/},
	{EN_GT_SMS_CMD_LANG, L"lang", {0x8BED,0x8A00,0x0000}/*L"语言"*/},
	{EN_GT_SMS_CMD_LOG_LEVEL, L"log-level", {0x65E5 ,0x5FD7 ,0x7EA7 ,0x522B,0x0000}/*L"日志级别"*/},
	{EN_GT_SMS_CMD_TIME_ZONE, L"time-zone", {0x65F6 ,0x533A,0x0000}/*L"时区"*/},
	{EN_GT_SMS_CMD_SHUTDOWN, L"shutdown", {0x5173,0x95ED,0x0000}/*L"关闭"*/},
	{EN_GT_SMS_CMD_RESTORE, L"restore", { 0x6062 ,0x590D ,0x51FA ,0x5382,0x0000}/*L"恢复出厂"*/},
	{EN_GT_SMS_CMD_ALARM_SWITCH, L"alarm-switch", {0x544A,0x8B66,0x5F00,0x5173,0x0000}/*L"告警开关"*/},
	{EN_GT_SMS_CMD_SMS_SWITCH, L"sms-switch", {0x77ED,0x4FE1,0x5F00,0x5173,0x0000}/*L"短信开关"*/},	
	{EN_GT_SMS_CMD_IGNORE_ALARM, L"ignore-alarm", {0x5FFD,0x7565,0x544A,0x8B66,0x0000}/*L"忽略告警"*/},
	{EN_GT_SMS_CMD_APN, L"apn", {0x63A5 ,0x5165 ,0x70B9,0x0000}/*L"接入点"*/},
	{EN_GT_SMS_CMD_SERVER, L"server", {0x670D ,0x52A1 ,0x5668,0x0000}/*L"服务器"*/},
	{EN_GT_SMS_CMD_DEFENCE, L"defence", {0x9632,0x62A4,0x0000}/*L"防护"*/},
	{EN_GT_SMS_CMD_SMS_CENTER, L"sms-center", {0x77ED, 0x4FE1, 0x4E2D, 0x5FC3,0x0000}/*L"短信中心"*/},
	{EN_GT_SMS_CMD_VER, L"ver", {0x7248, 0x672C,0x0000}/*L"版本"*/},
	{EN_GT_SMS_CMD_IMEI, L"imei", L"imei"/*L"imei"*/},
	{EN_GT_SMS_CMD_DEV_ID, L"dev-id", {0x8BBE, 0x5907, 0x7801,0x0000}/*L"设备码"*/},
	{EN_GT_SMS_CMD_PARA, L"para", {0x53C2, 0x6570,0x0000}/*L"参数"*/},
	{EN_GT_SMS_CMD_PWR_OIL_SWITCH, L"pwr-oil", { 0x6CB9, 0x7535, 0x0000}}//油电 <开关> | 开关 :0油电断开 1油电接通
};

//系统命令
struct err_desc{
	kal_uint32 err_code; 
	kal_uint16 desc_en[MAX_GT_ERR_DESC_LEN]; 
	kal_uint16 desc_zh[MAX_GT_ERR_DESC_LEN]; 
}gps_tracker_err_desc[] = 
{
	{EN_GT_EC_INVALID_SMS, L"invalid sms", {0x65E0, 0x6548, 0x77ED, 0x4FE1, 0x0000}/*L"无效短信"*/},
	{EN_GT_EC_INVALID_CMD, L"invalid cmd", {0x26080, 0x25928, 0x25351, 0x20196, 0x0000}/*L"无效指令"*/},
	{EN_GT_EC_INVALID_PARA, L"invalid parameter", {0x65E0,0x6548,0x53C2,0x6570,0x0000}/*L"无效参数"*/},
	{EN_GT_EC_NVRAM_OPT_ERR, L"nvram opt err", {0x26412, 0x22320, 0x25968, 0x25454, 0x25805, 0x20316, 0x22833, 0x36133,0x0000}/*L"本地数据操作失败"*/},
	{EN_GT_EC_SEND_ERR, L"send err", { 0x53D1 ,0x9001 ,0x9519 ,0x8BEF,0x0000}/*L"发送错误"*/},
	{EN_GT_EC_RCV_ERR, L"receive err", { 0x63A5 ,0x6536 ,0x9519 ,0x8BEF,0x0000}/*L"接收错误"*/},
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
//系统的cb序列号，从0 开始 递增
kal_uint32 gps_tracker_sn = 0;

//sn 对应mutex,保证sn的连续一致性递增
kal_mutexid gps_tracker_sn_mutex;

//系统日志级别
#ifdef _WIN32
U8 gps_tracker_log_level = INFO;
#else
U8 gps_tracker_log_level = INFO;
#endif

U8 gps_tracker_working_stage = EN_GT_WS_GPS;

/***********************************************************
数据结构体: 保存对象的实际数据
cb结构体: 按照发送接口定义的数据接口
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
//基站信息
gps_tracker_cell_info_struct 	gps_tracker_cell = {0};

gps_tracker_alarm_struct 		gps_tracker_alarm = {0};
/************************************************************/
gps_tracker_dev_req_struct 	gps_tracker_login_cb = {0};
//gps 信息
gps_tracker_dev_req_struct 	gps_tracker_gps_cb = {0};
//设备状态
gps_tracker_dev_req_struct 	gps_tracker_status_cb = {0};
//设备配置参数，开机从nvram获取，全程配置更新
gps_tracker_dev_req_struct 	gps_tracker_dev_data_cb = {0};

gps_tracker_dev_req_struct 	gps_tracker_hb_cb = {0};
//告警
gps_tracker_dev_req_struct 	gps_tracker_alarm_cb = {0};

//与控制器通讯信息
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

//当gps连续1分钟没有数据则重启
U8 gps_tracker_gps_failed_times = 0;

//dns 尝试次数
kal_uint8 gps_tracker_dns_times = 0;

//数据失败次数
kal_uint8 gps_tracker_send_failed_times = 0;
//心跳失败次数
kal_uint8 gps_tracker_hb_failed_times = 0;

//设备id
kal_uint8 gps_tracker_dev_id[MAX_GT_DEV_ID_LEN] = {0};
kal_uint8 gps_tracker_imei[MAX_GT_IMEI_LEN] = {0};

#ifdef __GPS_BAT_CONNECT__
battery_info_struct battery_info;
#endif

//log 日志 相关全局变量
kal_char log_buf[MAX_GT_LOG_BUF_LEN] = {0};

//本地数据文件
FS_HANDLE gps_tracker_local_read_file_hdl;
FS_HANDLE gps_tracker_local_write_file_hdl;

//uart 相关
static kal_uint8	uart_cur_handle;
static module_type	uart_orig_owner;

//系统时间是否ok
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

//当前震动值
U8 gps_tracker_shake_value = 0;
U8 gps_tracker_shake_value_fresh = 0;
U8 gps_tracker_shake_value_array[10];	//zzt.20150801
#ifdef __GPS_MCU_CONNECT__
U8 shake_value_mcu_index = 0;
U8 gps_tracker_shake_value_array_mcu[3];
#endif
#ifdef __ACC_EINT_MODE__
U8 gps_tracker_shake_delay_flag = 1;	//中断的延时标志
#endif

//连续检测到的震动次数
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

/*********************短信队列结构定义************************/
//这是一个循环消息队列 ，通过数组求余实现，模拟FIFO
#define MAX_GT_SMS_SEND_ARRAY_LEN 50
sms_node_struct sms_send_array[MAX_GT_SMS_SEND_ARRAY_LEN];

//消息队列用
kal_mutexid sms_send_array_mutex;
//FIFO 从tail 插入node，从 head 获取node
U8 fence_send_array_len = 0;
U8 fence_send_array_head_index = 0;
U8 fence_send_array_tail_index = 0;
/*********************接收缓冲队列结构定义************************/
//这是一个循环消息队列 ，通过数组求余实现，模拟FIFO
#define MAX_GT_RCV_MSG_ARRAY_LEN 20
#define MAX_GT_RCV_MSG_LEN 250
U8 rcv_msg_array[MAX_GT_RCV_MSG_ARRAY_LEN][MAX_GT_RCV_MSG_LEN];

//消息队列用
kal_mutexid rcv_msg_array_mutex;
//FIFO 从tail 插入node，从 head 获取node
U8 rcv_msg_array_len = 0;
U8 rcv_msg_array_head_index = 0;
U8 rcv_msg_array_tail_index = 0;

U16 gprs_account_name[] = L"IntelIOT";
/**************************************************************
*内部函数声明
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
*外部函数声明
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
*外部全局变量
***************************************************************/
extern srv_dtcnt_prof_gprs_struct g_data_account_gprs_profile_srv;
extern mmi_dtcnt_node_display_struct g_data_account_display_cntx;
#ifdef __ACC_EINT_MODE__
extern const unsigned char ACC_EINT_NO;
#endif

/**************************************************************
*公共函数
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
 *	打印trace，同时保存到本地log文件中
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
	char buf[100] = {0};//此处定义的长度务必小与log_buf的长度-前缀的长度
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
	
	//判断日志级别
	if(level < gps_tracker_log_level)
		return;
	

	//写入日志级别
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

	//取出log的内容部分 
	va_start (args, fmt);
	vsnprintf(buf, 99, fmt, args); 
	va_end (args);	

	//将前缀部分和log内容合并一起
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
	
	//同时输出到catcher log 中
	kal_prompt_trace(MOD_WPS, log_buf);	
//	U_PutUARTBytes(uart_port1,log_buf,sth);
#endif
}
#endif
/*****************************************************************************
 * FUNCTION
 *	remove_space_sharp
 * DESCRIPTION
 *	去掉短信内容中 头部和尾部的 空格 #号
 * PARAMETERS
 *	content 短信消息体
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

	//去掉头部的空格
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
	

	//去掉头部的空格
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

	//将有用的东西整体拷贝出来保证内存干净
	memcpy(new_content, head, (tail-head)*sizeof(kal_uint16));
	memset(content, 0, wcslen(content) * sizeof(U16));
	memcpy(content, new_content, (tail-head)*sizeof(kal_uint16));
	
	return KAL_SUCCESS;
}

/*****************************************************************************
 * FUNCTION
 *	max_match_r_cmp
 * DESCRIPTION
 *	号码比较函数,从右到左最大匹配进行比较
 * PARAMETERS
 *	void
 *	
 * RETURNS
 *	最大匹配相同，返回 0
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
 *	JSHash 算法
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
 *	将16进制的串转换成字节数组
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
		//判断hex 字符串的有效性
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
 *	获取字节数组的 CRC
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
		// 1.value 右移8位(相当于除以256)
		// 2.value与进来的数据进行异或运算后再与0xFF进行与运算
		//	  得到一个索引index，然后查找CRC16_TABLE表相应索引的数据
		// 1和2得到的数据再进行异或运算。
		value = (value >> 8) ^ CRC16_TABLE[(value ^ bytes[i]) & 0xff];			
	}
	
	// 取反
	return ~value;
}

/*****************************************************************************
 * FUNCTION
 *  wstr_get_wstr
 * DESCRIPTION
 *  从wstr 中获取第一个空格前的子串
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
	

	//1. 提取命令内容
	tmp1 = *str;	

	if(tmp1 == NULL)
		return KAL_ERROR;
	
	tmp2 = wcschr(tmp1, L' ');	
	
	if(tmp2 == NULL)
	{
		//没有找到空格，说明后面没有内容了
		kal_wstrcpy(sub_str, tmp1);

		tmp1 = NULL;
	}	
	else if( tmp2 >= *str + wcslen(*str) )
	{
		//说明空格已经超出整个短信内容了，出错
		return EN_GT_EC_INVALID_SMS;
	}
	else
	{
		//这里的地址相减 结果是类型的个数差，也就是U16的个数
		len = tmp2 - tmp1;
		kal_wstrncpy(sub_str, tmp1, len);	
	
		tmp1= tmp2+1;
	}
	if(sub_str[0] == L'*')
	{
		sub_str[0] = 0;
	}
	
	//过滤掉tmp1后面的空格
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
			//没有找到空格，说明后面没有内容了
			strcpy(num, tmp1);

			tmp1 = NULL;
		}	
		else if( tmp2 >= str_ip + strlen(str_ip) )
		{
			//说明空格已经超出整个短信内容了，出错
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
		
		//将字符串转化成数字
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
*业务函数
***************************************************************/
/**************************************************************
*围栏消息队列函数
***************************************************************/
/*****************************************************************************
 * FUNCTION
 *	fence_alarm_array_put_node
 * DESCRIPTION
 *	往队列中插入node ，如果发现队列满，则停止插入
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
*接收消息队列函数
***************************************************************/
/*****************************************************************************
 * FUNCTION
 *	fence_alarm_array_put_node
 * DESCRIPTION
 *	往队列中插入node ，如果发现队列满，则停止插入
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
 *	获取命令内容
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
 *	获取命令内容
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
 *  获取命令内容
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
 *  解析短信指令
 * PARAMETERS
 *  cmd 短信指令
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
		//管理员指令不判断权限
		case EN_GT_SMS_CMD_ADMIN:
			break;
		/////////////////////////////////////////////////////////////
		// 不需要权限，需要密码
		case EN_GT_SMS_CMD_IMEI:
		case EN_GT_SMS_CMD_DEV_ID:
		case EN_GT_SMS_CMD_RESTORE:
		case EN_GT_SMS_CMD_SERVER:
		case EN_GT_SMS_CMD_APN:
			break;
		//////////////////////////////////////////////////////////////
		//需要时管理员才能执行的指令
		case EN_GT_SMS_CMD_PWD: 
		case EN_GT_SMS_CMD_USER:	
		case EN_GT_SMS_CMD_UP_INTV:	
		//屏蔽
		//case EN_GT_SMS_CMD_HB_INTV:		
		//屏蔽	
		//case EN_GT_SMS_CMD_SMS_ALARM_INTV:			
		case EN_GT_SMS_CMD_TEMP_THR:	
		case EN_GT_SMS_CMD_VIBR_THR:	
		case EN_GT_SMS_CMD_SPEED_THR:	
		case EN_GT_SMS_CMD_LANG:	
		case EN_GT_SMS_CMD_TIME_ZONE:	
		//屏蔽
		//case EN_GT_SMS_CMD_SHUTDOWN:		
		case EN_GT_SMS_CMD_ALARM_SWITCH:
		case EN_GT_SMS_CMD_SMS_SWITCH:
		case EN_GT_SMS_CMD_DEFENCE:
		case EN_GT_SMS_CMD_IGNORE_ALARM:
		case EN_GT_SMS_CMD_PWR_OIL_SWITCH:
			//判断是否是管理员，只有管理员才能执行相关命令
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
		//是用户就可以执行的指令
		case EN_GT_SMS_CMD_LOC:
		case EN_GT_SMS_CMD_CELL_INFO:
		case EN_GT_SMS_CMD_VER:					
			//判断是否是合法用户
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
		//不对外的指令
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
 *  从内容串中获取号码
 * PARAMETERS
 *  短信参数
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
		//判断密码长度有效性
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
 *  从内容串中获取密码
 * PARAMETERS
 *  短信参数
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

	//宽字符转化成ascii
	wcstombs(pwd_str, tmp, wcslen(tmp));
	
	*pwd = js_hash(pwd_str, strlen(pwd_str));
			
	return KAL_SUCCESS;
}


/*****************************************************************************
 * FUNCTION
 *  gps_tracker_sms_get_para
 * DESCRIPTION
 *  解析短信指令 参数
 * PARAMETERS
 *  短信参数
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
			
			//获取输入密码
			ret = gps_tracker_para_get_pwd(&tmp1, &gps_tracker_sms_req.para.pwd);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__	
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get pwd failed");
			#endif
				
				return EN_GT_EC_INVALID_PARA;
			}
			
			//判断密码是否合法
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
			
			//获取旧密码
			//获取输入密码
			ret = gps_tracker_para_get_pwd(&tmp1, &gps_tracker_sms_req.para.pwd);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get pwd failed");
			#endif
				
				return EN_GT_EC_INVALID_PARA;
			}
			
			//判断密码是否合法
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
			//获取新密码
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
			
			//用户位置索引
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
			//判断索引合法性
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
				//用户号码
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
			
			//语言类型
			ret = wstr_get_uint(&tmp1, &uvalue);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get lang failed");
			#endif
				
				return EN_GT_EC_INVALID_PARA;
			}
			
			//判断语言合法性
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
			//时区
			ret = wstr_get_int(&tmp1, &ivalue);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get time-zone failed");
			#endif
				
				return EN_GT_EC_INVALID_PARA;
			}

			//时区合法性
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
			
			//获取输入密码
			ret = gps_tracker_para_get_pwd(&tmp1, &gps_tracker_sms_req.para.pwd);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get pwd failed");
			#endif
				
				return EN_GT_EC_INVALID_PARA;
			}
			
			//判断密码是否合法
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
			
			//日志级别
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
			
			//获取输入密码
			ret = gps_tracker_para_get_pwd(&tmp1, &gps_tracker_sms_req.para.pwd);
			if(KAL_SUCCESS != ret)
			{
				#ifdef __SMS_TRACE__								
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get pwd failed");
				#endif
				return EN_GT_EC_INVALID_PARA;
			}
			
			//判断密码是否合法
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
			//开关类型
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
			//开关值
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
			//开关类型
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
			//开关值
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
			//开关类型
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
			
			//获取输入密码
			ret = gps_tracker_para_get_pwd(&tmp1, &gps_tracker_sms_req.para.pwd);
			if(KAL_SUCCESS != ret)
			{
				#ifdef __SMS_TRACE__														
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get pwd failed");
				#endif
				return EN_GT_EC_INVALID_PARA;
			}
			
			//判断密码是否合法
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
			//认证id
			ret = wstr_get_uint(&tmp1, &uvalue);
			if(KAL_SUCCESS != ret)
			{
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get auth_id failed");
				
				return EN_GT_EC_INVALID_PARA;
			}	
	
			//判断指令认证码的合法性
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
			//地址类型
			ret = wstr_get_uint(&tmp1, &uvalue);
			if(KAL_SUCCESS != ret)
			{
				#ifdef __SMS_TRACE__														
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms addr type");
				#endif
				return EN_GT_EC_INVALID_PARA;
			}

			//判断服务器类型合法性
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
			//ip 域名
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

				//将点分字符ip转换成数组
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
			//端口号
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
			
			//获取输入密码
			ret = gps_tracker_para_get_pwd(&tmp1, &gps_tracker_sms_req.para.pwd);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__																
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get pwd failed");
			#endif	
				return EN_GT_EC_INVALID_PARA;
			}
			
			//判断密码是否合法
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
			//认证id
			ret = wstr_get_uint(&tmp1, &uvalue);
			if(KAL_SUCCESS != ret)
			{
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get auth_id failed");
				
				return EN_GT_EC_INVALID_PARA;
			}	
	
			//判断指令认证码的合法性
			if(uvalue != gps_tracker_sms_auth_id)
			{
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms auth code,input sms auth_code %u,dev auth_code %u", 
					uvalue, gps_tracker_sms_auth_id);
				return EN_GT_EC_INVALID_PARA;
			}
#endif
			//因为有可选参数，所以必须先清空全局变量
			memset(&gps_tracker_sms_req.para.apn, 0, sizeof(gps_tracker_sms_req.para.apn));
			
			if(tmp1 == NULL)
			{
				//apn 指令可以不带参数，表示清空apn参数
				return KAL_SUCCESS;
			}
			// apn 名称
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

			//用户名
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
			// 密码
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
					//将点分字符ip转换成数组
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
			//防护开关
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
			
			//获取输入密码
			ret = gps_tracker_para_get_pwd(&tmp1, &gps_tracker_sms_req.para.pwd);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__																			
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get pwd failed");
			#endif	
				return EN_GT_EC_INVALID_PARA;
			}
			
			//判断密码是否合法
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
			
			// 短信中心号码
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
			
			//获取输入密码
			ret = gps_tracker_para_get_pwd(&tmp1, &gps_tracker_sms_req.para.pwd);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__																						
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get pwd failed");
			#endif	
				return EN_GT_EC_INVALID_PARA;
			}
			
			//判断密码是否合法
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
			
			//获取输入密码
			ret = gps_tracker_para_get_pwd(&tmp1, &gps_tracker_sms_req.para.pwd);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__																						
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get pwd failed");
			#endif	
				return EN_GT_EC_INVALID_PARA;
			}
			
			//判断密码是否合法
			if(gps_tracker_sms_req.para.pwd != gps_tracker_config.pwd)
			{	
				//超级密码
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
			
			//获取输入密码
			ret = gps_tracker_para_get_pwd(&tmp1, &gps_tracker_sms_req.para.pwd);
			if(KAL_SUCCESS != ret)
			{
			#ifdef __SMS_TRACE__																									
				gps_tracker_trace(ERR, MOD_MMI,
					"invalid sms content,get pwd failed");
			#endif	
				return EN_GT_EC_INVALID_PARA;
			}
			
			//判断密码是否合法
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
 *  获取短信语言类型
 * PARAMETERS
 *  content : 短信内容
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
	
	//1. 获取命令字段
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
	
	//2. 将命令字符串转换成命令枚举
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
 *  发送短信响应
 * PARAMETERS
 *  msg_node : srv_sms_msg_node_struct sms 短信结构体
 *  
 * RETURNS
 *  void
 *****************************************************************************/
kal_int32 gps_tracker_validate_sms(kal_uint16* sms)
{	



	//短信预处理 去除部分手机短信头尾的空格和 # 字符
	remove_space_sharp(sms);

	//判断是否为空
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
 *  发送短信响应
 * PARAMETERS
 *  msg_node : srv_sms_msg_node_struct sms 短信结构体
 *  
 * RETURNS
 *  void
 *****************************************************************************/
kal_uint32 gps_tracker_sms_decode(sms_node_struct *msg_node)
{		
	kal_int32 ret = KAL_ERROR;


	//不用清零
	//memset(&gps_tracker_sms_req, 0, sizeof(gps_tracker_sms_req_struct));

#ifdef _WIN32
	wcscpy(msg_node->content, L" time-zone -130 ");
#endif

	//验证短信内容合法性
	ret = gps_tracker_validate_sms(msg_node->content);
	if(KAL_SUCCESS != ret)
	{
	#ifdef __SMS_TRACE__																										
		gps_tracker_trace(ERR, MOD_MMI,
				"invalid sms,ErrCode = 0x%X", ret);
	#endif
		
		return EN_GT_EC_INVALID_SMS;
	}
	
	//要清空，否在在调用wcslen时，会出现问题
	memset(&gps_tracker_sms_req.content, 0, sizeof(gps_tracker_sms_req.content));
	
	//保存短信内容到全局对象，为了下面进一步解析
	kal_wstrcpy(gps_tracker_sms_req.content.content_u16, msg_node->content);

	//宽字符转换成ascii码，顺便用来检测是不是纯ascii码 。在这个转换过程中，全英文的内容将会部分丢失
	wcstombs(gps_tracker_sms_req.content.content_s8, msg_node->content, wcslen(msg_node->content));
	//根据转换的结果就可以得到指令类型，如果是纯英文的内容，转换前后长度相等，否则不相等
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
	
	//保存手机号码
	strcpy(gps_tracker_sms_req.number.num_s8, msg_node->number);
	mbstowcs(gps_tracker_sms_req.number.num_u16, msg_node->number, strlen(msg_node->number));

	//解析短信内容
	ret = gps_tracker_decode_content(msg_node->content);
	
	if(KAL_SUCCESS != ret)
	{	
	#ifdef __SMS_TRACE__																									
		gps_tracker_trace(ERR, MOD_MMI,
			"decode failed.it is not the tracker msg,ErrCode %u", ret);
	#endif

		return ret;
	}

	//保存sim id
	//gps_tracker_sms_req.sim_id = msg_node->sim_id;//SRV_SMS_SIM_1;//多卡时为 msg_node->sim_id;

	return KAL_SUCCESS;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_update_gps
 * DESCRIPTION
 *  更新gps数据
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
	//1. 如果上一个包没有响应，则将gps数据存入本地文件
	if(gps_tracker_gps_cb.is_responsed != KAL_TRUE)
	{
		kal_uint32 size = 0;

		//获取当前本地文件大小
		FS_GetFileSize(gps_tracker_local_write_file_hdl, &size);

		//文件大小超限，则强行清零
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

		//避免此数据被重复存储，一定要刷新标识
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
		//gps有效
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
				
				//将当前的位置保存
				last_loc.lat = gps_tracker_gps.latitude;
				last_loc.lng = gps_tracker_gps.longitude;
	
				//定位类型 : GPS
				gps_tracker_gps_cb.dev_req_packet.content.gps.loc_type = EN_GT_LT_GPS;

				//纬度
				gps_tracker_gps_cb.dev_req_packet.content.gps.latitude = gps_tracker_gps.latitude;
				//经度
				gps_tracker_gps_cb.dev_req_packet.content.gps.longitude = gps_tracker_gps.longitude;
				
				//速度
				gps_tracker_gps_cb.dev_req_packet.content.gps.speed = gps_tracker_gps.speed;				

#ifdef _WIN32
				gps_tracker_gps.speed = 15000;
#endif
				
				//检测速度告警
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
				
				//航向
				gps_tracker_gps_cb.dev_req_packet.content.gps.course= gps_tracker_gps.course;
				//卫星数 
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

				//获取基站信号强度				
				gps_tracker_gps_cb.dev_req_packet.content.gps.reserv_sigstren = gps_tracker_cell.sig_stren;
				
				//添加基站信息
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
		else//gps 无效
		{	
		#if 0	//zzt.20150915.del for 静止时看到位置
			//获取基站信息			
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

				//定位类型 : 基站
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
 *  更新gps数据
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
		
	//状态	
	status.oil_pwr_state = gps_tracker_state.oil_pwr_state;

	status.sos_state = gps_tracker_state.sos_state;

	//检测震动告警
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

		//连续探测到N次震动，才告警
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

	//获取电压
	bat_level = srv_charbat_get_bat_level();
	status.volt_level = bat_level;
	//todo 电压告警
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
	
	//todo 温度及温度告警
	//暂未实现
	status.temp = 25;

	//断电告警
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
				//发送接收到的控制器数据
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
 *  GPS从串口获取控制器数据
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

		//规定参数长度，如长度不够补足6位
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
					*data = dataTmp[0];		//电量低字节
					*(data+1) = dataTmp[1];	//电量高字节
				}
				return GT_MCU_REPLY_BATTERY_STATUS;
			}
			case GT_MCU_REPLY_FAULT_STATUS:
				if(data)
				{
					*(data+2) = dataTmp[0];	//故障值
				}
				return GT_MCU_REPLY_FAULT_STATUS;
			case GT_MCU_REPLY_LOCK_STATUS:
				if(data)
				{
					*(data+3) = dataTmp[0];	//开关锁状态
				}
				return GT_MCU_REPLY_LOCK_STATUS;
			case GT_MCU_REPLY_HALL_STATUS:	//霍尔里程数
				if(data)
				{
					*(data+4) = dataTmp[0];	//低位
					*(data+5) = dataTmp[1];
					*(data+6) = dataTmp[2];
					*(data+7) = dataTmp[3];	//高位
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
 *  从蓝牙智控器获取数据，打包上传给服务器
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
 *  发送单片机获取的数据给服务器
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
			//发送信息
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
 *  从服务器获取数据，发送给单片机
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
			case 0x01:	//寻车
				gps_tracker_communicate_mcu(GT_MCU_SEND_SEARCH, NULL, 0, NULL);
				break;
			case 0x02:	//锁车
				gps_tracker_communicate_mcu(GT_MCU_SEND_LOCK, mcu_cmd->para, 1, NULL);
				break;
			case 0x03:	//报警开关
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
 *  上报定时器的处理函数，实现gps和status 数据包的上传
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
	
	//获取时间

#ifdef __MTK_TARGET__
	RTC_GetTime_(&rtc);
#else
	RTC_GetTime(&rtc);
#endif
/////////////////////////////////////////////////////////////////
	
	//todo 获取gps数据
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
// 测试时，静止也是要上报的

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
	//判断发送失败次数是否超过最大限
	if(gps_tracker_send_failed_times >= MAX_GT_SEND_FAILED_TIMES)
	{
		if((IsMyTimerExist(GPS_TRACKER_CONN_TIMER) != KAL_TRUE) && (IsMyTimerExist(GPS_TRACKER_DNS_TIMER) != KAL_TRUE))
		{
		#ifdef __GPRS_TRACE__	
			gps_tracker_trace(WARN, MOD_MMI, "send failed %u times, restart conn timer", gps_tracker_send_failed_times);
		#endif
			gps_tracker_conn_times =0 ;
			is_need_dns = KAL_TRUE;//每一次断链，都有一次dns的机会，保证换ip的情况下，能迅速上线
		#ifdef __GPRS_TRACE__	
			gps_tracker_trace(WARN, MOD_MMI,"@@@@@@@@@@@@@@@@@@@@@@@@@@@XXX-1");	
		#endif
			gps_tracker_conn_timer_proc();
		}
		
		gps_tracker_send_failed_times = 0;
	}
	
	//收集数据之前，需要先将响应的cb 的标识都清0
	gps_tracker_gps_cb.is_updated = KAL_FALSE;
	gps_tracker_status_cb.is_updated = KAL_FALSE;
	gps_tracker_alarm_cb.is_updated = KAL_FALSE;

	//1. 处理gps信息	
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
			//发送信息
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
	//处理与单片机的数据
	if(mcu_upload_interval >= 6)
	{
		mcu_upload_interval = 0;
		gps_tracker_send_mcu_data();
	}
#endif


	if(less_upload >= 3)
	{
		less_upload = 0;

		//2. 处理状态信息
		gps_tracker_update_status();

		if(gps_tracker_status_cb.is_updated)
		{
			len = gps_tracker_format_cb_to_buffer(&gps_tracker_status_cb, buffer, MAX_GT_SEND_LEN);

			ret = 0;
			if(gps_tracker_gsm_state == EN_GT_GSMS_WORKING)
			{
				//发送登陆信息
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

		//3. 处理告警信息
		//根据告警开关处理告警信息	
		ind = gps_tracker_alarm.pwr_low_ind << EN_GT_AT_PWR_LOW | (gps_tracker_alarm.pwr_off_ind << EN_GT_AT_PWR_OFF) 
				|(gps_tracker_alarm.vibr_ind << EN_GT_AT_VIBR)|(gps_tracker_alarm.oil_pwr_ind << EN_GT_AT_OIL_PWR)
				|(gps_tracker_alarm.speed_ind << EN_GT_AT_SPEED);
		
		for(i = 0; i < EN_GT_AT_END; i++)
		{	
			//判断当前类型的告警是否有效 1) 告警开关开 2) 有对应的告警类型
			//if((ind & (0x01<<i)) && (*(U8*)&gps_tracker_config.alarm_switch & (0x01<<i)))
			if(ind & (0x01<<i))
			{		
			#ifdef __GPRS_TRACE__
				gps_tracker_trace(ERR, MOD_MMI,"########## [alarm] alarm No:%d", i);
			#endif
				if(i == EN_GT_AT_VIBR)//震动告警
				{
					if(gps_tracker_config.alarm_switch.vibr == EN_GT_SWT_ON)
					{					
						gps_tracker_alarm_cb.dev_req_packet.content.alarm.type = EN_GT_AT_VIBR;
						gps_tracker_alarm_cb.dev_req_packet.content.alarm.value_len = sizeof(gps_tracker_alarm.vibr_value);
						gps_tracker_alarm_cb.dev_req_packet.content.alarm.value.vibr_value = gps_tracker_alarm.vibr_value;
						gps_tracker_alarm_cb.is_updated = KAL_TRUE;
					}
	/*				
					//格式化告警信息，插入告警短信队列
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
							kal_sprintf((char*)tmp_buf, (char*)"<震动告警> 震动级别 %u  %u/%u/%u %u:%u:%u", gps_tracker_alarm.vibr_value, 
								rtc.rtc_year, rtc.rtc_mon, rtc.rtc_day, rtc.rtc_hour, rtc.rtc_min, rtc.rtc_sec);
						}

						app_asc_str_to_ucs2_wcs((kal_uint8 *)content, (kal_uint8 *)tmp_buf);

						gps_tracker_send_sms_to_all_users(content);
					}
	*/
					//本次数据已经处理完毕，清除数据标志
					gps_tracker_alarm.vibr_ind = 0;
					gps_tracker_alarm.vibr_value = 0;
				}	
				else if(i == EN_GT_AT_SPEED)//超速告警
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
						//格式化告警信息，插入告警短信队列
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
								kal_sprintf((char*)tmp_buf, (char*)"<超速告警> 速度 %u  %u/%u/%u %u:%u:%u", gps_tracker_alarm.speed,
									rtc.rtc_year, rtc.rtc_mon, rtc.rtc_day, rtc.rtc_hour, rtc.rtc_min, rtc.rtc_sec);
							}

							app_asc_str_to_ucs2_wcs((kal_uint8 *)content, (kal_uint8 *)tmp_buf);

							gps_tracker_send_sms_to_all_users(content);
						}
					}
	*/
					//本次数据已经处理完毕，清除数据标志
					gps_tracker_alarm.speed_ind = 0;
					gps_tracker_alarm.speed = 0;
				}	
				else if(i == EN_GT_AT_PWR_LOW)//低电
				{
					if(gps_tracker_config.alarm_switch.pwr_low == EN_GT_SWT_ON)
					{					
						gps_tracker_alarm_cb.dev_req_packet.content.alarm.type = EN_GT_AT_PWR_LOW;
						gps_tracker_alarm_cb.dev_req_packet.content.alarm.value_len = sizeof(gps_tracker_alarm.pwr_level);
						gps_tracker_alarm_cb.dev_req_packet.content.alarm.value.volt_level = gps_tracker_alarm.pwr_level;
						gps_tracker_alarm_cb.is_updated = KAL_TRUE;
					}

					//本次数据已经处理完毕，清除数据标志
					gps_tracker_alarm.pwr_low_ind = 0;
					gps_tracker_alarm.pwr_level = 0;
				}
				else if(i == EN_GT_AT_PWR_OFF)//断电
				{
					if(gps_tracker_config.alarm_switch.pwr_off== EN_GT_SWT_ON)
					{					
						gps_tracker_alarm_cb.dev_req_packet.content.alarm.type = EN_GT_AT_PWR_OFF;
						gps_tracker_alarm_cb.dev_req_packet.content.alarm.value_len = 0;
						gps_tracker_alarm_cb.is_updated = KAL_TRUE;
					}

					//本次数据已经处理完毕，清除数据标志
					gps_tracker_alarm.pwr_off_ind= 0;
				}
				else if(i == EN_GT_AT_OIL_PWR)//断油电
				{
					if(gps_tracker_config.alarm_switch.oil_pwr == EN_GT_SWT_ON)
					{					
						gps_tracker_alarm_cb.dev_req_packet.content.alarm.type = EN_GT_AT_OIL_PWR;
						gps_tracker_alarm_cb.dev_req_packet.content.alarm.value_len = 0;
						gps_tracker_alarm_cb.is_updated = KAL_TRUE;
					}

					//本次数据已经处理完毕，清除数据标志
					gps_tracker_alarm.oil_pwr_ind= 0;
				}
				else
				{
					gps_tracker_alarm_cb.is_updated = KAL_FALSE;
				}
				
				if(gps_tracker_alarm_cb.is_updated)
				{
					//格式化告警信息
					len = gps_tracker_format_cb_to_buffer(&gps_tracker_alarm_cb, buffer, MAX_GT_SEND_LEN);
					
					if(gps_tracker_gsm_state >= EN_GT_GSMS_WORKING)
					{
						//发送登陆信息
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

	//继续运行
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
 *  负责定期检查短信队列，发送短信
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
 *  心跳流程函数
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


	//格式化登陆信息
	len = gps_tracker_format_cb_to_buffer(&gps_tracker_hb_cb, buffer, MAX_GT_SEND_LEN);	

	//发送登陆信息
	ret = soc_send(gps_tracker_soc.socketid, buffer, len, 0);		
	if(ret != len)
	{
		sendFail++;
	#ifdef __GPRS_TRACE__		
		gps_tracker_trace(ERR, MOD_MMI,"send hb packet failed...");
	#endif
	}
	
	if(sendFail == 2)   // 2次发送失败 ，则强制重启系统
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

	//心跳计数器
	gps_tracker_hb_failed_times++;
	//gps_tracker_send_failed_times++;	
	
	if(gps_tracker_hb_failed_times > 2)
	{		
		//连续2个心跳没收到，系统重启
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

	//重启定时器
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
 *  格式化cb的内容到发送buffer
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
	
	//1.先将类型和长度格式化进buffer
	buffer[0] = data->type;
	buffer[1] = data->value_len;

	buf = buffer + 2;

	//2. 将value结构体格式化进buf
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
 *  格式化cb的内容到发送buffer
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
	
	//1. 更新时间
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

	//2. 更新数据包的sn号，全局公用一个变量，流水自增
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

	//4. 将 head 格式化进buf
	memcpy(buf, &(cb->dev_req_packet.head), sizeof(gps_tracker_msg_head_struct));
	buf+=sizeof(gps_tracker_msg_head_struct);

	//5.将 content 格式化进 buf
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

	//填充tail
	memcpy(buf, &(cb->dev_req_packet.tail), sizeof(gps_tracker_msg_tail_struct));

	//更新crc
	crc = (U16*)(buffer + 2);

	//指向包长度
	buf = buffer+4;	
			
	*crc = get_crc16(buf, cb->dev_req_packet.head.pack_len + 1+1+2+6);//内容+包头字段
	
	real_len = cb->dev_req_packet.head.pack_len + PACKET_FRAME_LEN;		

	//将buffer 存入cb中，便于保存到本地存储文件
	cb->dev_req_packet.packet_len = real_len;
	memcpy(cb->dev_req_packet.packet_buf, buffer, cb->dev_req_packet.packet_len);
	
	return real_len;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_login_timer_proc
 * DESCRIPTION
 *  登陆处理流程
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
		//登陆失败重新建链
		//@由于ip错误时，也能产生CONNECT 消息，所以login会失败，所以失败时要尝试一次DNS
		if(is_need_dns == KAL_TRUE)
		{
		#ifdef __GPRS_TRACE__		
			gps_tracker_trace(WARN, MOD_MMI,"login failed more than %u times，is_need_dns == KAL_TRUE,so we start dns check timer",MAX_GT_LOGIN_TRY_TIMES);
		#endif

			gps_tracker_gsm_state = EN_GT_GSMS_DNS;

			if(IsMyTimerExist(GPS_TRACKER_DNS_TIMER) != MMI_TRUE)
			{
				//StartTimer(GPS_TRACKER_DNS_TIMER, MAX_GT_DNS_CHECK_INTV, (FuncPtr)gps_tracker_dns_check_timer_proc);	
				gps_tracker_dns_check_timer_proc();
			}

			//下次建链失败，就不再DNS了
			is_need_dns = KAL_FALSE;

			//关闭登陆定时器
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

			
			//格式化登陆信息
			len = gps_tracker_format_cb_to_buffer(&gps_tracker_login_cb, buffer, MAX_GT_SEND_LEN);
			
			//发送登陆信息		
			ret = soc_send(gps_tracker_soc.socketid, buffer, len, 0);
			
			gps_tracker_login_times++;//不管成功或者失败，计数器都加1，成功收到回复后清零
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

			//重启定时器
			if(IsMyTimerExist(GPS_TRACKER_LOGIN_TIMER) != MMI_TRUE)
			{
				StartTimer(GPS_TRACKER_LOGIN_TIMER, MAX_GT_LOGIN_INTV, 
						(FuncPtr)gps_tracker_login_timer_proc);
			}			
		}	
		else
		{
			//登陆成功，有收到响应。登陆结束
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

		    //收到成功事件后，将ip保存到全局变量
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

			//将数据更新进nvram
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
				//建立soc 链接
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
 *  根据 app_id acc_id  hostname 获取 hostname 对应的 ip 地址
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

			//收到成功事件后，将ip保存到全局变量
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

			
			//将数据更新进nvram
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

			// 域名解析失败自动重启，modif by zhangping 20150430
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
 *  获取app 信息
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
 *  开机检查dns的定时器
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
	//判断dns 尝试次数，如果小于门限值，则继续尝试，如果大于则重启
	else
	{
		//dns 失败后立即重启
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
 *  整个系统链接服务器的入口，由不同的开关切换
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
		if(first_fullService)	//为了防止第一次全服务后，立即创建socket可能会失败，所以延时等待下。
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
			//如果是ip模式则直接建链
			gps_tracker_gsm_state = EN_GT_GSMS_CONN;
			gps_tracker_conn_times =0 ;
		#ifdef __GPRS_TRACE__	
			gps_tracker_trace(WARN, MOD_MMI,"@@@@@@@@@@@@@@@@@@@@@@@@@@@XXX-8");
		#endif
			gps_tracker_conn_timer_proc();		
		}
		else if(gps_tracker_config.server.addr_type == EN_GT_ADT_DOMAIN )
		{
			//域名模式，则直接进行dns解析
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
 *  获取app 信息
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
	//1. 设置联网账号信息
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
	char imsi_imei_num[17] ={0};      //定义存放IMSI号的字符数组：IMSI号不多于15位，但是模拟器上会得到16位数字加一位结束符	
	mmi_smu_get_imsi_rsp_struct *local_data = NULL;

	if(info == NULL)
		return;
	
	local_data = (mmi_smu_get_imsi_rsp_struct*) info;   //得到的信息内容
	gps_tracker_trace(INFO,MOD_MMI,"gps_tracker_get_imsi_rsp");     //在Catcher中显示的
	strcpy(imsi_imei_num,(char*)local_data->imsi);     //将SIM存放在数组中 
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
 *  获取设备初始化参数
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
kal_uint32 gps_tracker_config_init(void)
{
	S16 error = NVRAM_READ_FAIL;


	//初始化全局变量
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

	//如果设备类型为0 ，说民系统第一次运行，nvram为空
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

	//密码
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
 *  初始化 tracker 的 定时器
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  kal_uint32 错误码
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
		
		//指定连接事件的回调函数
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
 *  初始化 log 文件
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
	
	//系统日志
	//不加绝对路径默认是创建在c盘根目录的
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
 *  链接定时器流程处理
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
			//ip 连接 失败后，重新尝试dns
			if( is_need_dns == KAL_TRUE )
			{
			#ifdef __GPRS_TRACE__
				gps_tracker_trace(WARN, MOD_MMI,"conn failed %u times，trying dns...",gps_tracker_conn_times);				
			#endif

				//关闭连接定时器
				if(IsMyTimerExist(GPS_TRACKER_CONN_TIMER) == KAL_TRUE)
				{
					StopTimer(GPS_TRACKER_CONN_TIMER);
				}
				
				//下次建链失败，就不再DNS了
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
					
				//启动定时器再次链接
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
						
			//启动定时器再次链接
		#ifdef __GPRS_TRACE__
			gps_tracker_trace(WARN, MOD_MMI,"@@@@@@@@@@@@@@@@@@@@@@@@@@@XXX-11");
		#endif
			StartTimer(GPS_TRACKER_CONN_TIMER, MAX_GT_CONN_INTV, 
				(FuncPtr)gps_tracker_conn_timer_proc);			
		}
	}
	else
	{		
		//超过链接尝试最大次数，重启
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
 *  初始化 系统 io
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
 *  初始化 系统 io
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
 *  初始化 系统 io
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
static U16 gps_tracker_uart_write(U8 *buf, U16 max_len, UART_PORT port, module_type owner)
{
	/*
	向串口写信息

	⑴清空设备输入、输出FIFO（UART_Purge）

	⑵清空发送、接收Buffer（UART_ClrTxBuffer，UART_ClrRxBuffer）

	⑶写入数据UART_PutBytes	 

	关闭串口
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

	// 收取数据，超出最大包长的数据将简单丢弃，这一层需要具体的应用协议做相应处理
	while( (avail_num = U_GetTxRoomLeft(port) > 0  &&  wLenRet < max_len) )
	{
		int i;

		//这里需要延时，否则可能发送失败
		for(i = 0; i< 10000; i++);
		
		if (avail_num + wLenRet > max_len)
		{
			avail_num = max_len - wLenRet;
		}

		wLenWrite = U_PutBytes(port, (kal_uint8 *)(buf + wLenRet), (kal_uint16)avail_num, owner);
		wLenRet += wLenWrite;
	}

	// 读完之后，清除发送Buffer
	U_Purge(port, TX_BUF, owner);
	U_ClrTxBuffer(port, owner);
#endif
	return wLenRet;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_uart_ready_to_read_ind_handler
 * DESCRIPTION
 *  初始化 系统 io
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
	
	//注意时区的正负
	if( (U16)offset < 32768)//正数
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
 *  初始化 系统 io
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

	//1.取时间
	head = buf+6;
	tail = strchr(head, ',');	
	
	//根据时间决定定位信息是否有效
	if(tail-head == 0)//说明2个逗号相邻，数据为空	
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
	
	//2. 定位状态
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
	//3. 纬度
	head = tail+1;
	tail = strchr(head, ',');
	if(tail-head == 0)//说明2个逗号相邻，数据为空	
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

	//4.纬度半球
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
	//5. 经度
	head = tail+1;
	tail = strchr(head, ',');	
	if(tail-head == 0)//说明2个逗号相邻，数据为空	
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
	//6.经度半球
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
	//7.速度	
	head = tail+1;
	tail = strchr(head, ',');	
	memset(tmp,0,16);
	memcpy(tmp, head, tail-head);
	dvalue = atof(tmp);
	gps->speed= dvalue*1.852*100;//knots 转换成kph

	//8.航向 以真北为参考
	head = tail+1;	
	tail = strchr(head, ',');	
	memset(tmp,0,16);
	memcpy(tmp, head, tail-head);
	dvalue = atof(tmp);
	gps->course = dvalue*100;
	//9. 日期
	head = tail+1;
	tail = strchr(head, ',');	
	memcpy(tmp, head, tail-head);
	uvalue = atof(tmp);
	gps->datetime[2]= uvalue/10000;
	gps->datetime[1]= (uvalue%10000)/100;
	gps->datetime[0]= uvalue%100;
	//10 磁偏角
	head = tail+1;
	tail = strchr(head, ',');	
	memset(tmp,0,16);
	memcpy(tmp, head, tail-head);
	dvalue = atof(tmp);
	gps->magnetic_value = dvalue*100;

	//11. 磁偏角方向
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
	//12. 模式 
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
	//第一次成功定位
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
 *  初始化 系统 io
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

	//1.取时间
	head = buf+6;

	//找到第6个位置的值
	for(i = 0; i < 6; i++)
	{
		tail = strchr(head, ',');
		head = tail+1;
	}
	
	//取卫星个数	
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

	//3. 海拔
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
 *  初始化 系统 io
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

	
	//开始解析gps报文
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
 *  初始化 系统 io
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
 *  初始化 系统 io
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
		// 如果底层能做成钩子链，这里就可以调用钩子链的下一个钩子了
		return KAL_SUCCESS;
	}

	//读取gps信息
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
 *  初始化 系统 io
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
/*static kal_int32 gps_tracker_uart_init(void)
{

#ifdef __MTK_TARGET__	
	//初始化串口对应的io，使之工作于 UART 模式
	GPIO_ModeSetup(gpio_uart2_tx, 1);
	GPIO_ModeSetup(gpio_uart2_rx, 1);


	UART_DriverInit(uart_port2);

	uart_cur_handle = L1SM_GetHandle();
	L1SM_SleepDisable(uart_cur_handle);

	// 记录我们要用的串口的当前占有者
	uart_orig_owner = U_GetOwnerID(uart_port2);
	
	// 下面我们申明要占用这个串口了
	U_SetOwner(uart_port2, MOD_MMI);	

	// 设置波特率，缺省的起停位和校验为:8,n,1，即8个数据位，1个停止位，无校验

	U_SetBaudRate(uart_port2, UART_BAUD_9600, MOD_MMI);


	// 其他串口设定(如起停位、校验等)使用函数 UART_ReadDCBConfig 和 UART_SetDCBConfig
	// 详细参数见结构体 UARTDCBStruct

	// 注册一个事件钩子函数，当串口(任何)有数据到达时，我们的钩子函数将被调用
	// 注意，同一种事件同时只能注册一个钩子函数，因此：
	// 如果在我们的程序处理串口的同时还有其他程序要读取和处理(任何)串口数据，
	// 就必须由当前的钩子函数代为处理
	// 实际上我觉得系统底层可以改一下，改成Windows钩子的方式，可以挂多个，能够依次调用或跳过

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

	//串口2 初始化
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
	//串口1 初始化
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
 *  初始化 系统 io
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
 *  获取 震动数据
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

	//进行数组平均
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

	//在第四版中，震动传感器会有问题，所以有下面的代码
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


	//进行数组平均
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
	StartTimer(GPS_TRACKER_SHAKE_DETECT_TIMER, 1000, gps_tracker_shake_eint_callback);	//1//1秒内只响应一次中断
#endif	
	
	//获取震动变化值
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
 *  初始化 加速度传感器
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
	//启动震动状态检测
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
 *  初始化 系统 io
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
void gps_tracker_io_init(void)
{

	//初始化加速度传感器
	gps_tracker_acce_init();

	//初始化串口
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
 *  系统全局变量初始化
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

	//sn mutex 初始化
	gps_tracker_sn_mutex = kal_create_mutex("sn mutex");
	
	//短信队列 mutex 初始化
	sms_send_array_mutex = kal_create_mutex("sms alarm array mutex");

	//接收消息队列 mutex 初始化
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
	//避免第一个数据被存储到本地
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
 *  系统开机获取初始化参数
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
	
    // 处理RR_EM_LAI_INFO
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
 *  系统开机获取初始化参数
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
 *  系统开机获取初始化参数
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
 *  定时重启
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
void gps_tracker_auto_reset(void)
{//	每天晚上3点自动重启 !
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
 *  系统开机获取初始化参数
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  void
 *****************************************************************************/
void gps_tracker_init(void)
{	
	
	//1.初始化log系统
	gps_tracker_files_init();	
	
	//2. 获取设备默认参数
	gps_tracker_config_init();		

	//3. 初始化全局变量
	gps_tracker_variable_init();	

	//5. 初始化外设
	gps_tracker_io_init();	

	//6. 基站初始化
	gps_tracker_cell_init();
	
	//7. 获取服务器信息
	gps_tracker_server_init();	

	//8. 初始化数据采集
//	gps_tracker_task_init();		//zzt.20150730.move login success
#ifdef __GPS_PACKAGE_BY_QUEUE__
	gps_tracker_queue_init();
#endif

	//9. 定时重启函数  moodif by zhangping 20150503
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
 *  生成 gsm 基站请求命令
 * PARAMETERS
 * [IN]mcc
 * [IN]mnc
 * [IN]lac
 * [IN]cellid
 * [OUT]gsm_cell_req
 * RETURNS
 *  错误码
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
 * dev_cfg  配置响应的处理函数，根据不同的消息类型回复不同的响应
 * PARAMETERS
 * RETURNS
 *  错误码
 *****************************************************************************/
kal_int32 gps_tracker_dev_data_rsp_proc(gps_tracker_data_content_struct* data)
{
	//U16 error;
	//U16 send_buf[MAX_GT_SMS_CONTENT_LEN] = {0};
	//U8 tmp_buf[MAX_GT_SMS_CONTENT_LEN*2] = {0};

	if (data == NULL)
		return KAL_ERROR;

	//根据不同的短信类型回复不同的内容
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
				
				//将网络字节序转换成机器字节序
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
 * dev_cfg  配置响应的处理函数，根据不同的消息类型回复不同的响应
 * PARAMETERS
 * RETURNS
 *  错误码
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
			// 更新系统全局变量
			strcpy(gps_tracker_config.admin_num, data->value.admin_number);

			// 更新进 NVRAM 
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
			// 更新系统全局变量
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@ pwd %u", data->value.pwd);
		#endif
			gps_tracker_config.pwd = data->value.pwd;

			// 更新进 NVRAM	
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
			
			// 2 更新nvram里面的user 列表
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
			// 1 更新数据
			#ifdef __GPRS_TRACE__							
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@ up_intv %u", data->value.upload_intv);
			#endif
			gps_tracker_config.up_intv = data->value.upload_intv;
			
			// 2 更新nvram里面的user 列表
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
			// 1 更新数据
			gps_tracker_config.hb_intv = data->value.hb_intv;
			
			// 2 更新nvram里面的user 列表
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
			// 1 更新数据
/*			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@ sms_send_intv %u", data->value.sms_send_intv);
			gps_tracker_config.sms_send_intv = data->value.sms_send_intv;
			
			// 2 更新nvram里面的user 列表
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
			// 1 更新数据
		#ifdef __GPRS_TRACE__			
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@ temp_thr %u", data->value.temp_thr);
		#endif
			gps_tracker_config.temp_thr = data->value.temp_thr;
			
			// 2 更新nvram里面的user 列表
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
			// 1 更新数据
		#ifdef __GPRS_TRACE__				
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@ vibr_thr %u", data->value.vibr_thr);
		#endif
			gps_tracker_config.vibr_thr = data->value.vibr_thr;
			
			// 2 更新nvram里面的user 列表
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
			// 1 更新数据
			#ifdef __GPRS_TRACE__								
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@ speed_thr %u", data->value.speed_thr);
			#endif
			gps_tracker_config.speed_thr = data->value.speed_thr;
			
			// 2 更新nvram里面的user 列表
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
			//目前由于平台此数据的不可配置，暂时屏蔽此数据的更新
			/*
			// 1 更新数据
			gps_tracker_config.lang = data->value.lang;
			
			// 2 更新nvram里面的user 列表
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
			//目前由于平台此数据的不可配置，暂时屏蔽此数据的更新
			/*
			// 1 更新数据
			gps_tracker_config.time_zone = data->value.time_zone;
			
			// 2 更新nvram里面的user 列表
			error = NVRAM_WRITE_FAIL;
			
			WriteRecord(NVRAM_EF_GT_TIME_ZONE_LID, 1, &gps_tracker_config.time_zone, 
				sizeof(gps_tracker_config.time_zone), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
				gps_tracker_trace(ERR, MOD_MMI,
					"write nvram config <time-zone> failed!");	

				return EN_GT_EC_NVRAM_OPT_ERR;
			}
			
			//配置参数重启生效
			if(IsMyTimerExist(GPS_TRACKER_SMS_DELAY_REPLAY_TIMER) != MMI_TRUE)
			{
				gps_tracker_trace(ERR, MOD_MMI, "delay reset...");
				StartTimer(GPS_TRACKER_SMS_DELAY_REPLAY_TIMER, MAX_GT_SMS_DELAY_REPLY_TIME, 
						(FuncPtr)gps_tracker_sms_delay_rst_proc);
			}
			*/
			break;
		case EN_GT_DT_ALARM_SWITCH:
			// 1 更新数据
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
			// 2 更新nvram里面的user 列表
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
			// 1 更新数据
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
			
			// 2 更新nvram里面的user 列表
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
						
				//将网络字节序转换成机器字节序
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
				
				//对于特殊串进行智能替换
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
			// 1 更新数据
			gps_tracker_config.defence =  data->value.defence;
			
			// 2 更新nvram里面的user 列表
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

			//这里不能整体拷贝，因为ip 方式的包里面没有域名，会导致在ip失败时转DNS ，DNS会失败。
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

			// 更新到nvram
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
			//配置参数重启生效
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
			// 1 更新数据
			memcpy(gps_tracker_config.sms_center_num, data->value.sms_center_num, sizeof(gps_tracker_config.sms_center_num));
			// 2 更新nvram里面的user 列表
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

				//油电断开告警
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
			//断油电告警 58 高电平 relay 导通
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
		

	
	// 看本地gps数据需要上传				
	FS_GetFileSize(gps_tracker_local_read_file_hdl, &size);
	
	if(size > 0)
	{					
		S32 read_size;
		S32 want_read_size;
		S32 new_file_size;
		U8 buf[64] = {0};
		U8 i;
		

		//每次读取10个包
		if(size < GPS_PACKET_SIZE*10)
		{
			want_read_size = size;
		}
		else
		{
			want_read_size = GPS_PACKET_SIZE*10;
		}

		//读完后，新文件的大小
		new_file_size = size - want_read_size;

		//从末尾开始读取数据
		FS_Seek( gps_tracker_local_read_file_hdl, new_file_size, FS_FILE_BEGIN );
		
		for(i = 0; i < want_read_size/GPS_PACKET_SIZE; i++)
		{
			FS_Read(gps_tracker_local_read_file_hdl, (void*)buf,GPS_PACKET_SIZE, &read_size);
	
			//更新序列号
			*(kal_uint16*)(buf+6) =  gps_tracker_get_sn();

			//更新crc
			*(kal_uint16*)(buf+2) = get_crc16(buf+4,GPS_PACKET_SIZE-6);//6 =起始长度 + 校验长度 + 结束长度					
			
			ret = soc_send(gps_tracker_soc.socketid, buf, GPS_PACKET_SIZE, 0);
#ifdef __GPRS_TRACE__																				
			gps_tracker_trace(WARN, MOD_MMI,"gps local packet:prot_id %u sn %u",
				buf[5], *(U16*)&buf[6]);
#endif

			//这里的计数本来是要打开的，但因为发送速度很快，所以需要设置的门限计数值就会很高，这会导致正常逻辑需要等
			//待很长的时间才能建链，所以这里不加1. 但收到响应时会减1.因为减1时做了保护，所以不会有问题
			//gps_tracker_send_failed_times ++;

			if(ret != GPS_PACKET_SIZE)
			{
		#ifdef __GPRS_TRACE__																							
				gps_tracker_trace(WARN, MOD_MMI,"send gps local data failed,GPS_PACKET_SIZE %u;send ret %u",
					GPS_PACKET_SIZE, ret);
		#endif

				return ;//发送失败，下一次定时器会重新尝试发送
			}							
		}

		//todo 如果数据读完了，则删除文件
		if(new_file_size == 0)
		{
			//关闭文件
			FS_Close(gps_tracker_local_read_file_hdl);

			//删除文件
			FS_Delete(L"gps_local_read");
		#ifdef __GPRS_TRACE__																							
			gps_tracker_trace(WARN, MOD_MMI,"@@@@@@@@@@@@@@@@delet gps local read file");
		#endif
			
			//停止上传定时器
			if(IsMyTimerExist(GPS_TRACKER_LOCAL_DATA_UPLOAD_TIMER) == MMI_TRUE)
			{
				StopTimer(GPS_TRACKER_LOCAL_DATA_UPLOAD_TIMER);
			}			

			return;
		}
		else//todo 如果没有读完，更新文件长度
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
 * 登陆消息的处理函数，根据不同的消息类型回复不同的响应
 * PARAMETERS
 * RETURNS
 *  错误码
 *****************************************************************************/
kal_int32 gps_tracker_login_rsp_proc()
{	
	kal_uint32 size = 0;
	kal_uint8 buf[MAX_GT_SEND_LEN] = {0};	
	
	

	//关闭登陆定时器进程				
	if(IsMyTimerExist(GPS_TRACKER_LOGIN_TIMER) == MMI_TRUE)
	{
		StopTimer(GPS_TRACKER_LOGIN_TIMER);
	#ifdef __GPRS_TRACE__																								
		gps_tracker_trace(WARN, MOD_MMI,"stop GPS_TRACKER_LOGIN_TIMER timer");	
	#endif
	}	

	//启动本地数据上传定时器
	if(IsMyTimerExist(GPS_TRACKER_LOCAL_DATA_UPLOAD_TIMER) != MMI_TRUE)
	{		
		//StartTimer(GPS_TRACKER_LOCAL_DATA_UPLOAD_TIMER,1000,(FuncPtr)gps_tracker_local_data_upload_timer_proc);
		// todo 关闭写入的文件
		// 重名为读取的文件
		// 重新创建和打开写入的文件			
		FS_GetFileSize(gps_tracker_local_write_file_hdl, &size);

		if(size > 0)
		{
			int ret;
			
			FS_Close(gps_tracker_local_write_file_hdl);

			//先将本地数据文件重命名
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
		//启动心跳定时器
		//StartTimer(GPS_TRACKER_HB_TIMER, gps_tracker_config.hb_intv*1000, (FuncPtr)gps_tracker_hb_timer_proc);		
		gps_tracker_hb_timer_proc();
	}
		
	return KAL_SUCCESS;
}

/*****************************************************************************
 * FUNCTION
 *  gps_tracker_rcv_proc
 * DESCRIPTION
 * 收到网站响应后，根据不同的消息类型，进行消息处理
 * PARAMETERS
 * RETURNS
 *  错误码
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

	
	//处理响应
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
				gps_tracker_login_cb.is_responsed = KAL_TRUE;//收到响应标	

				gps_tracker_task_init();	//zzt.20150730.move from init
				#ifdef __LED_INDICATE_STATE__
				if(!gps_tracker_gsm_led)
				gps_tracker_open_gsm_work_mode();
				#endif

				if(gps_tracker_gsm_state != EN_GT_GSMS_WORKING)
				{
					//t_rtc rtc;					
					

					
					//更新系统时间	
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

						//国外版本这里需要偏移，因为国外版本的服务器时间是格林威治时间

#ifdef OVERSEA
						gps_tracker_datetime_offset(&date, gps_tracker_config.time_zone);
#endif
						
						mmi_dt_set_rtc_dt((MYTIME *)&date);

						is_datetime_updated = KAL_TRUE;
					}
					//获取系统时间	
					/*
					RTC_GetTime(&rtc); 
					
					gps_tracker_trace(WARN, MOD_MMI, 
							"datetime %u %u %u %u %u %u", 
							rtc.rtc_year, rtc.rtc_mon, rtc.rtc_day, rtc.rtc_hour, rtc.rtc_min, rtc.rtc_sec);
					*/
					//在这里开启 working标志，因为在随后的发送操作里面马上就要用working标志了，如果放到最后，会影响随后的操作
					gps_tracker_gsm_state = EN_GT_GSMS_WORKING;
					
					ret = gps_tracker_login_rsp_proc();
					if(KAL_SUCCESS != ret)
					{
					#ifdef __GPRS_TRACE__																								
						gps_tracker_trace(ERR, MOD_MMI,"login rsp proc failed.");
					#endif
						return ret;
					}					
					
					//向服务器发送ver 信息
					{
						U16 len;
						U8 buffer[MAX_GT_SEND_LEN] = {0};
						
						// 更新cb
						gps_tracker_dev_data_cb.dev_req_packet.content.data.type = EN_GT_DT_VER;
						gps_tracker_dev_data_cb.dev_req_packet.content.data.value_len = MAX_GT_VER_STR_LEN;
						strcpy(gps_tracker_dev_data_cb.dev_req_packet.content.data.value.ver, gps_tracker_config.ver);
					#ifdef __GPRS_TRACE__																								
						gps_tracker_trace(INFO, MOD_MMI,"send ver %s to server", gps_tracker_config.ver);		
					#endif	
						//格式化登陆信息
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
				//心跳计数器
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
						case 0x1c:	//蓝牙智控器指令下发处理
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

				//处理数据请求
				ret = gps_tracker_srv_data_req_proc(data);
				if(KAL_SUCCESS != ret)
				{
				#ifdef __GPRS_TRACE__
					gps_tracker_trace(ERR, MOD_MMI,"srv_data rsp proc failed.");
				#endif
					return ret;
				}	
				
				//发送响应
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
 *  soc_connect 事件回调
 * PARAMETERS
 *  msg_ptr 系统回调参数 app_soc_notify_ind_struct类型 
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
						
						//找包头 0xffff
						if(rcv_buf[i]==0xff && rcv_buf[i+1]==0xff)
						{
							head = rcv_buf+i;
						}
						else if(rcv_buf[i]==0x0d && rcv_buf[i+1]==0x0a)
						{
							tail = rcv_buf+i+2;//结尾

							//验证长度合法性
							if(tail - head == head[4] + PACKET_FRAME_LEN)
							{
								//合法
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
								//找到合法结尾
								head = tail;
							}
							//继续找结尾
							i +=1;	//代码加1，for循环自加1 ，等效于向后偏移2个字节（0d0a）			
						}
					}

					//剩下的残余字节在这里处理
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

			//关闭建链定时器进程				
			if(IsMyTimerExist(GPS_TRACKER_CONN_TIMER) == MMI_TRUE)
			{
			    StopTimer(GPS_TRACKER_CONN_TIMER);
			}
			
			//刷新响应的计数器
			//连接计数器
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
			//登陆阶段，由于网关作用，会出现即使ip错误，也会connect 成功，但随后会有close 消息。
			//继续登陆，do nothing
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
 *  pwd 处理函数
 * PARAMETERS
 *  void *  
 * RETURNS
 *  kal_uint32 错误码
 *****************************************************************************/
kal_int32 gps_tracker_send_req(S8* buffer, U16 len)
{
	kal_int32 ret = 0;
	

	if (buffer == NULL)
		return KAL_ERROR;

	
	//发送登陆信息	
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
 *  pwd 处理函数
 * PARAMETERS
 *  void *  
 * RETURNS
 *  kal_uint32 错误码
 *****************************************************************************/
kal_int32 gps_tracker_send_rsp(S8* buffer, U16 len)
{
	kal_int32 ret = 0;

	if (buffer == NULL)
		return KAL_ERROR;
	
	//发送登陆信息	
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
 *  get loc 处理函数
 * PARAMETERS
 *  void
 *  
 * RETURNS
 *  kal_uint32 错误码
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
 
	//出厂默认值
	sprintf(config.ver, "%s", GT_VER);//版本号
	config.dev_type = 0x10;//设备类型
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

	//报警开关
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
	
	//短信中心号码
	strcpy(config.sms_center_num,"");

	//imsi
	memset(config.imsi, 0, sizeof(config.imsi));
	
	gps_tracker_trace(ERR, MOD_MMI,"start resore nvram record"); 
	//////////////////////////更新 NVRAM ///////////////////////////////////////////////////
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
	
	//密码
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
 *  log-level 处理函数
 * PARAMETERS
 *  void
 *  
 * RETURNS
 *  kal_uint32 错误码
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
	//根据不同的短信类型回复不同的内容
	if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
	{
		kal_sprintf(tmp_buf, "<shutdown> success.");
	}
	else
	{
		kal_sprintf((char*)tmp_buf, (char*)"<关闭> 成功。");		
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
 *  联网获取定位地址信息
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
	/*重要 
	用户短信的处理流程 :
	1) 更新本地参数
	2) 上报参数到服务器
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

			//更新本地参数
			// 更新系统全局变量
			strcpy(gps_tracker_config.admin_num, gps_tracker_sms_req.number.num_s8);

			// 更新进 NVRAM 
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
				kal_sprintf((char*)tmp_buf, (char*)"<管理员> 成功。");
			}
			
			// 更新cb
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
			// 更新系统全局变量
			gps_tracker_config.pwd = gps_tracker_sms_req.para.new_pwd;

			// 更新进 NVRAM	
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
				kal_sprintf((char*)tmp_buf, (char*)"<密码> 成功。");
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
			// 1 清除全局变量
			if(0 == gps_tracker_sms_req.para.user_index)
			{		
				memset(&gps_tracker_config.users, 0, sizeof(gps_tracker_config.users));		
			}
			else
			{
				memcpy(gps_tracker_config.users[gps_tracker_sms_req.para.user_index-1],
					gps_tracker_sms_req.para.user, sizeof(gps_tracker_sms_req.para.user));
			}
			
			// 2 更新nvram里面的user 列表
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
				kal_sprintf((char*)tmp_buf, (char*)"<用户> 成功。");
			}
			
			// 更新cb
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
			// 1 更新数据
			gps_tracker_config.up_intv = gps_tracker_sms_req.para.upload_intv;
			
			// 2 更新nvram里面的user 列表
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

			//准备回复短信
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf((char*)tmp_buf, "<up-intv> success.");
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<上报间隔> 成功。");
			}
			
			// 更新cb
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

			// 1 更新数据
			gps_tracker_config.hb_intv = gps_tracker_sms_req.para.hb_intv;
			
			// 2 更新nvram里面的user 列表
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

			//准备回复短信
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf((char*)tmp_buf, "<hb-intv> success.");
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<心跳间隔> 成功。");
			}
			
			// 更新cb
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

			// 1 更新数据
			gps_tracker_config.sms_send_intv = gps_tracker_sms_req.para.sms_send_intv;
			
			// 2 更新nvram里面的user 列表
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
				kal_sprintf((char*)tmp_buf, (char*)"<短信间隔> 成功。");
			}
			
			// 更新cb
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
			// 1 更新数据
			gps_tracker_config.temp_thr = gps_tracker_sms_req.para.temp_thr;
			
			// 2 更新nvram里面的user 列表
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
				kal_sprintf((char*)tmp_buf, (char*)"<温度门限> 成功。");
			}
			
			// 更新cb
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

			// 1 更新数据
			gps_tracker_config.vibr_thr = gps_tracker_sms_req.para.vibr_thr;
			
			// 2 更新nvram里面的user 列表
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
				kal_sprintf((char*)tmp_buf, (char*)"<震动门限> 成功。");
			}
			
			// 更新cb
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
			// 1 更新数据
			gps_tracker_config.speed_thr = gps_tracker_sms_req.para.speed_thr;
			
			// 2 更新nvram里面的user 列表
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
				kal_sprintf((char*)tmp_buf, (char*)"<速度门限> 成功。");
			}
			
			// 更新cb
			gps_tracker_dev_data_cb.dev_req_packet.content.data.type = EN_GT_DT_SPEED_THR;
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value_len = sizeof(gps_tracker_sms_req.para.speed_thr);
			gps_tracker_dev_data_cb.dev_req_packet.content.data.value.speed_thr = gps_tracker_sms_req.para.speed_thr;
			gps_tracker_dev_data_cb.is_updated = KAL_TRUE;	
			break;
		case EN_GT_SMS_CMD_LOC:
		#ifdef __SMS_TRACE__									
			gps_tracker_trace(WARN, MOD_MMI, "EN_GT_SMS_CMD_LOC cmd");	
		#endif
			
			//更新 cb，准备上报
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

			//根据不同的短信类型回复不同的内容
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf(tmp_buf, "<cell> success.mmc %u,mnc %u,lac %u,cell_id %u", 
					gps_tracker_cell.mcc, gps_tracker_cell.mnc, gps_tracker_cell.lac_sid,
					gps_tracker_cell.cellid_nid);
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<基站> 成功。 国家 %u，运营商 %u，小区 %u，基站 %u",
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
			// 1 更新数据
			gps_tracker_config.lang = gps_tracker_sms_req.para.lang;
			
			// 2 更新nvram
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
				kal_sprintf((char*)tmp_buf, (char*)"<语言> 成功。");
			}
			
			// 更新cb
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
	
			//根据不同的短信类型回复不同的内容
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf(tmp_buf, "<log-level> success.");
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<日志级别> 成功。");		
			}	
		#ifdef __SMS_TRACE__							
			gps_tracker_trace(ERR, MOD_MMI, "@@@@@@@@@@@@@@@EN_GT_SMS_CMD_LOG_LEVEL %u OK", EN_GT_SMS_CMD_LOG_LEVEL);
		#endif
			break;			
		case EN_GT_SMS_CMD_TIME_ZONE:
			//更新时区
		#ifdef __SMS_TRACE__									
			gps_tracker_trace(INFO, MOD_MMI,
						"EN_GT_SMS_CMD_TIME_ZONE cmd");
		#endif

			// 1 更新数据
			gps_tracker_config.time_zone = gps_tracker_sms_req.para.time_zone;
			
			// 2 更新nvram里面的user 列表
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
				kal_sprintf((char*)tmp_buf, (char*)"<时区> 成功。");
			}

			if(IsMyTimerExist(GPS_TRACKER_SMS_DELAY_REPLAY_TIMER) != MMI_TRUE)
			{
		#ifdef __SMS_TRACE__												
				gps_tracker_trace(ERR, MOD_MMI, "delay reset...");
		#endif
				
				StartTimer(GPS_TRACKER_SMS_DELAY_REPLAY_TIMER, MAX_GT_SMS_DELAY_REPLY_TIME, 
						(FuncPtr)gps_tracker_sms_delay_rst_proc);
			}
			
			// 更新cb
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
				kal_sprintf((char*)tmp_buf, (char*)"<恢复出厂> 成功。");		
			}
				
			if(IsMyTimerExist(GPS_TRACKER_SMS_DELAY_REPLAY_TIMER) != MMI_TRUE)
			{
		#ifdef __SMS_TRACE__												
				gps_tracker_trace(ERR, MOD_MMI, "delay reset...");
		#endif
				StartTimer(GPS_TRACKER_SMS_DELAY_REPLAY_TIMER, MAX_GT_SMS_DELAY_REPLY_TIME, 
						(FuncPtr)gps_tracker_sms_delay_rst_proc);
			}
			
			// 更新cb
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
			
			// 2 更新nvram里面的user 列表
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
				kal_sprintf((char*)tmp_buf, (char*)"<告警开关> 成功。");
			}
			
			// 更新cb
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
			
			// 2 更新nvram
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
				kal_sprintf((char*)tmp_buf, (char*)"<短信开关> 成功。");
			}
			
			// 更新cb
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
				kal_sprintf((char*)tmp_buf, (char*)"<忽略告警> 成功。");
			}
			
			// 更新cb
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
			//此命令要求及时在没有联网的情况下也成功，所以需要先完成更新操作，再上传配置更新
			//这里不能整体拷贝，因为ip 方式的包里面没有域名，会导致在ip失败时转DNS ，DNS会失败。
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

			// 更新到nvram
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
			//同时更新agps
			memcpy(&gps_tracker_config.agps_server, &gps_tracker_sms_req.para.server, sizeof(gps_tracker_config.agps_server));

			// 更新到nvram
			WriteRecord(NVRAM_EF_GT_AGPS_SRV_LID, 1, &gps_tracker_config.agps_server, 
				sizeof(gps_tracker_config.agps_server), &error);

			if (NVRAM_WRITE_SUCCESS != error)
			{
				gps_tracker_trace(ERR, MOD_MMI,"write nvram record <agps> failed!");	

				return KAL_ERROR;
			}
*/				
			//根据不同的短信类型回复不同的内容
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf(tmp_buf, "<server> success.");
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<服务器> 成功。");		
			}		

			//本地数据更新后，准备重启
			if(IsMyTimerExist(GPS_TRACKER_SMS_DELAY_REPLAY_TIMER) != MMI_TRUE)
			{
			#ifdef __SMS_TRACE__														
				gps_tracker_trace(ERR, MOD_MMI, "delay reset...");
			#endif
				StartTimer(GPS_TRACKER_SMS_DELAY_REPLAY_TIMER, MAX_GT_SMS_DELAY_REPLY_TIME, 
						(FuncPtr)gps_tracker_sms_delay_rst_proc);
			}
			
			///////////////////////////////////////////////////////
			// 准备上传配置更新
			// 更新cb
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

			// 1 更新数据
			gps_tracker_config.defence = gps_tracker_sms_req.para.defence;
			
			// 2 更新nvram里面的user 列表
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
				kal_sprintf((char*)tmp_buf, (char*)"<防护> 成功。");
			}
			
			// 更新cb
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

			// 1 更新数据
			memcpy(gps_tracker_config.sms_center_num, gps_tracker_sms_req.para.sms_center_num, 
				sizeof(gps_tracker_sms_req.para.sms_center_num));
			
			// 2 更新nvram里面的user 列表
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
				kal_sprintf((char*)tmp_buf, (char*)"<短信中心> 成功。");
			}
			
			// 更新cb
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

			//根据不同的短信类型回复不同的内容
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				kal_sprintf(tmp_buf, "<ver> success.ver: %s", gps_tracker_config.ver);
			}
			else
			{
				kal_sprintf((char*)tmp_buf, (char*)"<版本> 成功。版本：%s", gps_tracker_config.ver);		
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
			//这里要返回，因为gps_tracker_sms_para()里面已经回复短信了
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

				//油电断开告警
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
			//准备回复短信
			if(gps_tracker_sms_req.cmd_lang_type == EN_GT_LANG_EN)
			{
				sprintf((char*)tmp_buf, "<pwr-oil> success.");
			}
			else
			{
				sprintf((char*)tmp_buf, (char*)"<油电> 成功。");
			}
			break;
		
		default:
			return EN_GT_EC_INVALID_CMD;
			break;
	}

	//回复短信
	if(strlen(tmp_buf) != 0)
	{
		app_asc_str_to_ucs2_wcs((kal_uint8 *)send_buf, (kal_uint8 *)tmp_buf);

		gps_tracker_send_sms(gps_tracker_sms_req.number.num_u16, send_buf);
	}

	//根据需要，向服务器发送更新请求	
	if( gps_tracker_dev_data_cb.is_updated == KAL_TRUE )
	{
		//格式化登陆信息
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
 *  短信流程错误处理函数
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
 *  短信流程错误处理函数
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

	//解析短信信息获取短信命令类型和命令参数		
	ret = gps_tracker_sms_decode(msg_node); 	

	if(ret != KAL_SUCCESS)
	{
		gps_tracker_sms_err_proc(ret);

		return ret;
	}

	// 根据不同的短信命令进行不同的处理
	//ret = ids_sms_proc();
	ret = gps_tracker_sms_proc();

	// 错误处理函数
	if(ret != KAL_SUCCESS)
	{
		//错误响应在里面已经回复了，这里不需要再短信回复
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
	GPS_BAT_SUM_VOLTAGE,			//总电压
	GPS_BAT_CURRENT,				//充放电电流
	GPS_BAT_DESIGN_CAPABILITY,		//设计容量
	GPS_BAT_DESIGN_VOLITAGE,		//设计电压
	GPS_BAT_FCC,						//电池包最大容量
	GPS_BAT_REMAIN_CAPABILITY,		//剩余容量
	GPS_BAT_PERCENT_CAPABILITY,	//容量百分比
	GPS_BAT_CYCLE_COUNT,			//循环次数
	GPS_BAT_MANUFACTURENAME,		//制造商名称
	GPS_BAT_TEMPERATURE,			//电池温度
	GPS_BAT_STATUS,					//电池包状态
	GPS_BAT_INFO,					//电池包信息
	GPS_BAT_BUNCH_NUM,				//电池串数
	GPS_BAT_BAR_CODE1,				//电池条码1
	GPS_BAT_BAR_CODE2,				//电池条码2
	GPS_BAT_BAR_CODE3,				//电池条码3
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
	
	battery_info.total_voltage = (battery_info.total_voltage*battery_info.bunch_num)/3000;	//总电压=(总电压*总串数)/(3串*1000)

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

