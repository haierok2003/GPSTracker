/******************����ת��ʱ�������ο��� begin ***********************/
#if 0
typedef unsigned char 		U8;
typedef signed char         S8;
typedef unsigned short      U16;
typedef signed short        S16;
typedef unsigned int        U32;
typedef signed int          S32;
#endif
/******************����ת��ʱ�������ο��� end ***********************/

#define MAX_GT_DEV_ID_LEN 		16

//��Ӧ��һ��� MAX_GT_DEV_ID_LEN 
#define MAX_GT_DEV_ID_BYTE_LEN 		8

#define MAX_GT_PHONE_NUM_LEN 	16

#define MAX_GT_USER_COUNT 		4

#define MAX_GT_MSG_CMD_LEN 		32

#define MAX_GT_MSG_RSP_LEN 		128

#define MAX_GT_DOMAIN_LEN 		32

#define MAX_GT_IP_ADDR_LEN 		4


#define MAX_GT_SMS_CONTENT_LEN  	140//����������˫�ֽ� unicode��

#define MAX_GT_FENCE_NUM 			4

#define MAX_GT_APN_NAME_LEN 		32

#define MAX_GT_APN_USER_NAME_LEN 	32

#define MAX_GT_APN_PWD_LEN 			16

#define MAX_GT_APN_USER_DATA_LEN 	32

#define MAX_GT_VER_STR_LEN 			16

#define MAX_GT_CHECK_NUM	 		48

#pragma pack (1)//ǿ���ֽڶ���
/********* ��ͷ ��β ****************/
//Э���ͷ
typedef struct 
{
	U16 start;     	//��ʼλ
	U16	crc;		//crcУ��
	U8	pack_len;	//������	---- ���ݳ���//zzt.20150715.add note
	S8	prot_type;	//Э������
	U16 sn;	 		//�����к�
	S8 datetime[6];	//��ʱ��
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

/********* 0x01 ��½��****************/
typedef struct 
{
	U8 	dev_id[MAX_GT_DEV_ID_BYTE_LEN];	//�豸id 15λIMEIת���ɵ�8λ�ֽ���
	U8 	dev_type;					//�豸����
	S32 auth_code;					//��֤��
}gps_tracker_login_req_content_struct;

/********* 0x02 GPS ��****************/
typedef struct 
{	
	U8 lat_ind:1;	//0 ��γ 1 ��γ
	U8 long_ind:1;	//0  ����1����
	U8 mode:2;   	//0 ʵʱ 1 ��� 2 ���� 3 ��Ч �������ʵʱ����ֻ��߹��㣬˵��gps�Ѿ��ɹ���λ��
}gps_tracker_property_struct;

typedef struct 
{
	S8 loc_type;       	//��λ����
	U8 reserv_satnum;	//ǰ��λgps�������� + ����λ������
	gps_tracker_property_struct property;		//gps��������
	U32 latitude;		//γ��
	U32 longitude;		//����
	U16 speed;			//�ٶ�
	U16 course;			//����

	U8 reserv_sigstren;	//ǰ4bit��վ���ݱ������� ��4bit��վ�ź�ǿ��0-15
	U16 mcc;			//mcc
	U8  mnc;			//mnc
	U16 lac_sid;		//lac
	U16 cellid_nid;		//cellid
	U16 bid;		    //�ݱ���
}gps_tracker_gps_req_content_struct;

/********* 0x03 status ��****************/
typedef struct 
{	
	U8 oil_pwr_state:1;		//���͵�״̬ 0 �͵��ͨ 1 ���Ͷϵ�
	U8 sos_state:1;			//sos ״̬ 0 ��sos  1 ��sos
	U8 volt_level:3;        //��ѹ�ȼ� 0-6
	
	S8 temp;				//���϶��¶ȣ���δʵ��
}gps_tracker_status_req_content_struct;

/********* 0x04 hb ��****************/
//������������
/********* 0x05 �澯 ��****************/
typedef struct 
{	
	U32 index;		//Χ�����к�	
	S8 type;		// 0 �޸澯 1 ���澯  2 ��澯 3 ����澯	
}gps_tracker_fence_alarm_struct;

typedef union {
	S8 		volt_level; 				//�澯ʱ�ĵ�ѹֵ
	S8  	vibr_value;					//�澯ʱ����ֵ
	S8 		temp;						//�澯ʱ���¶�ֵ
	U16 	speed;						//�澯ʱ���ٶ�ֵ
	gps_tracker_fence_alarm_struct fence;
}alarm_union;

typedef struct 
{	
	S8 type;			//�澯����
	U8 value_len;		//ֵ�ĳ�������
	alarm_union value;	//�澯ֵ
}gps_tracker_alarm_req_content_struct;
/********* 0x06 �豸���������ݰ�****************/
typedef struct 
{	
	S8 			users[MAX_GT_USER_COUNT][MAX_GT_PHONE_NUM_LEN];	//�û��б�
	S8 			sms_content[MAX_GT_SMS_CONTENT_LEN];			//�������ݵ��ֽ���
}gps_tracker_sms_msg_node_struct;

typedef struct 
{	
	U32 index;		//Χ�����к�	
	S8 	alarm_type; // 0 �޸澯 1 ���澯  2 ��澯 3 ����澯 4ɾ��
	U32 lat; 		//γ��
	U32 lng;       	//����
	U32 radius;		//�뾶����λ cm
}gps_tracker_fence_struct;

//server
typedef struct
{
	S8	addr_type;					//��ַ����  0 ip�� 1 ����
	S8	domain[MAX_GT_DOMAIN_LEN];	//����
	U8	ip[MAX_GT_IP_ADDR_LEN]; 	//ip
	U16	port;						//�˿ں�		
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
	S8			admin_number[MAX_GT_PHONE_NUM_LEN];	//����Ա����
	S32 		pwd;								//����Ա����
	S8 			users[MAX_GT_USER_COUNT][MAX_GT_PHONE_NUM_LEN];		//�û��б�

	
	U16 		upload_intv;		//ʱ����
	U16 		hb_intv;			//�������
	U16 		sms_send_intv;		//���Ÿ澯���
	
	S8			temp_thr; 	//�澯�¶����� -128--+127
	S8			vibr_thr; 	//������
	U16			speed_thr;	//��������

	S8 			lang; 				//Ĭ�϶�������
	S16 		time_zone;			//ʱ��ƫ�ƶ�Ӧ�ķ�����
	
	gps_tracker_data_switch_struct 	alarm_switch;			//��������
	gps_tracker_data_switch_struct 	sms_alarm_switch;		//����֪ͨ����

	S8			log_level;       			//�豸��־����
	S8 			ignore_alarm;	 			//���Եĸ澯����

	S8 			loc_lang_type;				//��ַ����������

	S8  		datetime[6];
	gps_tracker_sms_msg_node_struct sms; 	//Ҫ���͵Ķ���
	
	gps_tracker_fence_struct 		fence[MAX_GT_FENCE_NUM];    //Χ����Ϣ
	S8 								defence; 					//0 ���� 1���
	gps_tracker_server_struct    	server;		//����������
	gps_tracker_server_struct       agps_server;
	gps_tracker_apn_struct 			apn;
	S8 		ver[MAX_GT_VER_STR_LEN];      			//�豸�汾��
	S8 		sms_center_num[MAX_GT_PHONE_NUM_LEN];	//�������ĺ���
	
	gps_tracker_dev_check_struct  dev_check;
	gps_tracker_srv_check_struct  srv_check;

	U32 calling_fence_radius;
	S8 srv_req_data_type;
	U8 oil_pwr_switch;
	gps_tracker_io_struct io;
}data_union;

typedef struct 
{	
	S8 type;			//��������
	U8 value_len;		//������������ ����
	data_union value;	//��������ֵ
}gps_tracker_data_content_struct;

/****************************0x07��ȡ���������ݰ�**************************/
typedef struct
{
	U8 addr;		//0x1a �綯����������0x1b���վ��������0x1c��Ƭ�������ǿ���
	U8 value_len;
	U8 value[18];
}gps_tracker_control_data_struct;

/******************����ת��ʱ�������ο��� begin ***********************/
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

/******************����ת��ʱ�������ο��� end ***********************/
#pragma pack ()//ǿ���ֽڶ���


