/******************工具转换时，打开屏蔽开关 begin ***********************/
#if 0
typedef unsigned char 		U8;
typedef signed char         S8;
typedef unsigned short      U16;
typedef signed short        S16;
typedef unsigned int        U32;
typedef signed int          S32;
#endif
/******************工具转换时，打开屏蔽开关 end ***********************/

#define MAX_GT_DEV_ID_LEN 		16

//对应上一版的 MAX_GT_DEV_ID_LEN 
#define MAX_GT_DEV_ID_BYTE_LEN 		8

#define MAX_GT_PHONE_NUM_LEN 	16

#define MAX_GT_USER_COUNT 		4

#define MAX_GT_MSG_CMD_LEN 		32

#define MAX_GT_MSG_RSP_LEN 		128

#define MAX_GT_DOMAIN_LEN 		32

#define MAX_GT_IP_ADDR_LEN 		4


#define MAX_GT_SMS_CONTENT_LEN  	140//短信内容是双字节 unicode的

#define MAX_GT_FENCE_NUM 			4

#define MAX_GT_APN_NAME_LEN 		32

#define MAX_GT_APN_USER_NAME_LEN 	32

#define MAX_GT_APN_PWD_LEN 			16

#define MAX_GT_APN_USER_DATA_LEN 	32

#define MAX_GT_VER_STR_LEN 			16

#define MAX_GT_CHECK_NUM	 		48

#pragma pack (1)//强制字节对齐
/********* 包头 包尾 ****************/
//协议包头
typedef struct 
{
	U16 start;     	//起始位
	U16	crc;		//crc校验
	U8	pack_len;	//包长度	---- 内容长度//zzt.20150715.add note
	S8	prot_type;	//协议类型
	U16 sn;	 		//包序列号
	S8 datetime[6];	//包时间
}gps_tracker_msg_head_struct;

typedef struct 
{	
	U16	stop;
}gps_tracker_msg_tail_struct;

typedef struct
{	
	S8 type;
	S8 value;
} gps_tracker_data_switch_struct;

/********* 0x01 登陆包****************/
typedef struct 
{
	U8 	dev_id[MAX_GT_DEV_ID_BYTE_LEN];	//设备id 15位IMEI转化成的8位字节码
	U8 	dev_type;					//设备类型
	S32 auth_code;					//认证码
}gps_tracker_login_req_content_struct;

/********* 0x02 GPS 包****************/
typedef struct 
{	
	U8 lat_ind:1;	//0 南纬 1 北纬
	U8 long_ind:1;	//0  西经1东经
	U8 mode:2;   	//0 实时 1 差分 2 估算 3 无效 ；如果是实时、差分或者估算，说明gps已经成功定位。
}gps_tracker_property_struct;

typedef struct 
{
	S8 loc_type;       	//定位类型
	U8 reserv_satnum;	//前四位gps保留长度 + 后四位卫星数
	gps_tracker_property_struct property;		//gps数据属性
	U32 latitude;		//纬度
	U32 longitude;		//精度
	U16 speed;			//速度
	U16 course;			//航向

	U8 reserv_sigstren;	//前4bit基站数据保留长度 后4bit基站信号强度0-15
	U16 mcc;			//mcc
	U8  mnc;			//mnc
	U16 lac_sid;		//lac
	U16 cellid_nid;		//cellid
	U16 bid;		    //暂保留
}gps_tracker_gps_req_content_struct;

/********* 0x03 status 包****************/
typedef struct 
{	
	U8 oil_pwr_state:1;		//断油电状态 0 油电接通 1 断油断电
	U8 sos_state:1;			//sos 状态 0 无sos  1 有sos
	U8 volt_level:3;        //电压等级 0-6
	
	S8 temp;				//摄氏度温度，暂未实现
}gps_tracker_status_req_content_struct;

/********* 0x04 hb 包****************/
//心跳包无内容
/********* 0x05 告警 包****************/
typedef struct 
{	
	U32 index;		//围栏序列号	
	S8 type;		// 0 无告警 1 出告警  2 入告警 3 出入告警	
}gps_tracker_fence_alarm_struct;

typedef union {
	S8 		volt_level; 				//告警时的电压值
	S8  	vibr_value;					//告警时的震动值
	S8 		temp;						//告警时的温度值
	U16 	speed;						//告警时的速度值
	gps_tracker_fence_alarm_struct fence;
}alarm_union;

typedef struct 
{	
	S8 type;			//告警类型
	U8 value_len;		//值的长度类型
	alarm_union value;	//告警值
}gps_tracker_alarm_req_content_struct;
/********* 0x06 设备服务器数据包****************/
typedef struct 
{	
	S8 			users[MAX_GT_USER_COUNT][MAX_GT_PHONE_NUM_LEN];	//用户列表
	S8 			sms_content[MAX_GT_SMS_CONTENT_LEN];			//短信内容的字节码
}gps_tracker_sms_msg_node_struct;

typedef struct 
{	
	U32 index;		//围栏序列号	
	S8 	alarm_type; // 0 无告警 1 出告警  2 入告警 3 出入告警 4删除
	U32 lat; 		//纬度
	U32 lng;       	//经度
	U32 radius;		//半径，单位 cm
}gps_tracker_fence_struct;

//server
typedef struct
{
	S8	addr_type;					//地址类型  0 ip， 1 域名
	S8	domain[MAX_GT_DOMAIN_LEN];	//域名
	U8	ip[MAX_GT_IP_ADDR_LEN]; 	//ip
	U16	port;						//端口号		
} gps_tracker_server_struct;

typedef struct
{
	S8	apn_name[MAX_GT_APN_NAME_LEN];//cmnet
	S8	user_name[MAX_GT_APN_USER_NAME_LEN];	 
	S8	passwd[MAX_GT_APN_PWD_LEN];	
    U8 	px_addr[MAX_GT_IP_ADDR_LEN];
	U16 px_port;        // proxy port 
	S8	user_data[MAX_GT_APN_USER_DATA_LEN];
} gps_tracker_apn_struct;

typedef struct
{
	S8	check_num;				
	U16 check_sum;
} gps_tracker_srv_check_struct;

typedef struct
{
	S8	check_num;				
	U16 check_sum[MAX_GT_CHECK_NUM];
} gps_tracker_dev_check_struct;

typedef struct
{
	S8	port;				
	S8  state;
} gps_tracker_io_struct;

typedef union {
	S8			admin_number[MAX_GT_PHONE_NUM_LEN];	//管理员号码
	S32 		pwd;								//管理员密码
	S8 			users[MAX_GT_USER_COUNT][MAX_GT_PHONE_NUM_LEN];		//用户列表

	
	U16 		upload_intv;		//时间间隔
	U16 		hb_intv;			//心跳间隔
	U16 		sms_send_intv;		//短信告警间隔
	
	S8			temp_thr; 	//告警温度门限 -128--+127
	S8			vibr_thr; 	//震动门限
	U16			speed_thr;	//超速门限

	S8 			lang; 				//默认短信语言
	S16 		time_zone;			//时区偏移对应的分钟数
	
	gps_tracker_data_switch_struct 	alarm_switch;			//报警开关
	gps_tracker_data_switch_struct 	sms_alarm_switch;		//短信通知开关

	S8			log_level;       			//设备日志级别
	S8 			ignore_alarm;	 			//忽略的告警类型

	S8 			loc_lang_type;				//地址的语言类型

	S8  		datetime[6];
	gps_tracker_sms_msg_node_struct sms; 	//要发送的短信
	
	gps_tracker_fence_struct 		fence[MAX_GT_FENCE_NUM];    //围栏信息
	S8 								defence; 					//0 撤防 1设防
	gps_tracker_server_struct    	server;		//服务器配置
	gps_tracker_server_struct       agps_server;
	gps_tracker_apn_struct 			apn;
	S8 		ver[MAX_GT_VER_STR_LEN];      			//设备版本号
	S8 		sms_center_num[MAX_GT_PHONE_NUM_LEN];	//短信中心号码
	
	gps_tracker_dev_check_struct  dev_check;
	gps_tracker_srv_check_struct  srv_check;

	U32 calling_fence_radius;
	S8 srv_req_data_type;
	U8 oil_pwr_switch;
	gps_tracker_io_struct io;
}data_union;

typedef struct 
{	
	S8 type;			//配置类型
	U8 value_len;		//配置数据类型 长度
	data_union value;	//配置数据值
}gps_tracker_data_content_struct;

/****************************0x07获取控制器数据包**************************/
typedef struct
{
	U8 addr;		//0x1a 电动车控制器，0x1b充电站控制器，0x1c单片机蓝牙智控器
	U8 value_len;
	U8 value[18];
}gps_tracker_control_data_struct;

/******************工具转换时，打开屏蔽开关 begin ***********************/
typedef union {
 	 gps_tracker_login_req_content_struct login;	
	 gps_tracker_gps_req_content_struct gps;
	 gps_tracker_status_req_content_struct status;
	 gps_tracker_alarm_req_content_struct alarm;
	 gps_tracker_data_content_struct dev_data_req;
	 gps_tracker_data_content_struct srv_data_rsp;
}gps_tracker_send_data_content_struct; 
  
typedef struct 
{
	gps_tracker_msg_head_struct head;
	gps_tracker_send_data_content_struct content;
	gps_tracker_msg_tail_struct tail;
}gps_tracker_send_data_struct;

typedef union {
	gps_tracker_data_content_struct dev_data_rsp;
	gps_tracker_data_content_struct srv_data_req;
}gps_tracker_rcv_data_content_struct; 
  
typedef struct 
{
	gps_tracker_msg_head_struct head;
	gps_tracker_rcv_data_content_struct content;
	gps_tracker_msg_tail_struct tail;
}gps_tracker_rcv_data_struct;

/******************工具转换时，打开屏蔽开关 end ***********************/
#pragma pack ()//强制字节对齐


