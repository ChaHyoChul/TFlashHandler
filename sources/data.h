#ifndef _DATA_H_
#define _DATA_H_

#define MAX_VARS		(96)
#define MAX_POINT_DATA	(100)


extern char g_LastCommand[32];
extern char g_MotionCommand;
extern int  g_ErrorCode;
extern char g_OriginCompleted;
extern char g_OriginRunning;
extern char g_Running;
extern char g_Alarm;
extern char g_ResponseSend;
extern int  g_ScanRate;
extern char g_UserStop;
extern char g_GoToError;

extern int  g_OriginNeed[3];
extern int  g_OriginCompletedAxes[3];


typedef struct _POINT_DATA
{
	union
	{
		double axis[3];
		struct 
		{
			 double x;
			 double y;
			 double z;
		};
	};
} POINT_DATA;


// ��� ������ �ʿ��� �Ķ����
typedef struct _MOTION_PARAM
{
	unsigned int 	m_uOrgSLimit;	//00 (4)���� ������ ����Ʈ ����Ʈ (���� : pulse, ���� : 0 ~ 2147483647)
	unsigned char 	m_ucOrgDir;		//01 (1)���� ������ ã�ư� ���� (0 : CW, 1 : CCW)
	unsigned char	m_ucOrgNeed;	//02 (1)���� ���� �ʿ� ����
	unsigned int 	m_uOrgVmax;		//03 (4)���� ������ �ִ� �ӵ� (���� : pps, ���� : 0 ~ 524287)
	unsigned int 	m_uOrgVmin;		//04 (4)���� ������ ���� �ӵ� (���� : pps, ���� : 0 ~ 32767)
	unsigned int 	m_uOrgAcel;		//05 (4)���� ������ ��/�� �ӵ� (���� : ?, ���� : 0 ~ 1023)
	unsigned int 	m_uNorVmax;		//06 (4)Normal ���۽� �ִ� �ӵ� (���� : pps, ���� : 0 ~ 524287)
	unsigned int 	m_uNorVmin;		//07 (4)Normal ���۽� ���� �ӵ� (���� : pps, ���� : 0 ~ 32767)
	unsigned int 	m_uNorAcel;		//08 (4)Normal ���۽� ��/�� �ӵ� (���� : ?, ���� : 0 ~ 1023)
	unsigned int	m_uJogVmax;		//09 (4)JOG ���۽� �ִ� �ӵ� (���� :pps : 0 ~ 524287)
	unsigned int	m_uJogVmin;		//10 (4)JOG ���۽� ���� �ӵ� (���� :pps : 0 ~ 32767)
	unsigned int	m_uJogAcel;		//11 (4)JOG ���۽� ��/�� �ӵ� (���� :pps : 0 ~ 1023)
	int 			m_nHomeOff;		//12 (4)�������� Home ��ġ������ Offset (���� : pulse)
	unsigned char	m_ucMoveDir;	//13 (1)���� �̵� ����
	unsigned char	m_ucEncSign;	//14 (1)���ڴ� ��ȣ
	double			m_fLead;		//15 (8)Lead
	unsigned int	m_ucEncPulse;	//16 (4)���� 1ȸ�� �޽��� (�⺻ : 3200)
	double			m_fScaleFactor;	//17 (8)���� Scale Facotr ( = m_fLead/m_ucEncPulse )
	unsigned char	m_ucOrgSensor;	//18 (1)Origin Sensor
	unsigned char	m_ucNegLimit;	//19 (1)Negative Limit Sensor
	unsigned char	m_ucPosLimit;	//20 (1)Positive Limit Sensor
} MOTION_PARAM;
/*
#define MOTION_PARAM_ORG_SOFT_LIMIT		(0)	//(0)
#define MOTION_PARAM_ORG_SPEED			(3)	//(1)
#define MOTION_PARAM_ORG_LOW_SPEED		(4) //(2)
#define MOTION_PARAM_JOG_SPEED			(9) //(3)
#define MOTION_PARAM_JOG_LOW_SPEED		(10) //(4)
#define MOTION_PARAM_PTP_SPEED			(6) //(5)
#define MOTION_PARAM_PTP_LOW_SPEED		(7) //(6)
#define MOTION_PARAM_ACCEL				(8) //(7)
#define MOTION_PARAM_ORG_DIR			(1) //(8)
#define MOTION_PARAM_PTP_DIR			(13) //(9)
#define MOTION_PARAM_ENC_SIGN			(14) //(10)
#define MOTION_PARAM_ORG_NEED			(2) //(11)
#define MOTION_PARAM_ORG_SENSOR			(18) //(12)
#define MOTION_PARAM_LEAD				(15) //(13)
#define MOTION_PARAM_ENC_PULSE			(16) //(14)
#define MOTION_PARAM_SCALE_FACTOR		(17) //(15)
#define MOTION_PARAM_NEG_LIMIT			(19) //(16)
#define MOTION_PARAM_POS_LIMIT			(20) //(17)
*/
#define MOTION_PARAM_ORG_SOFT_LIMIT		(0)
#define MOTION_PARAM_ORG_SPEED			(1)
#define MOTION_PARAM_ORG_LOW_SPEED		(2)
#define MOTION_PARAM_JOG_SPEED			(3)
#define MOTION_PARAM_JOG_LOW_SPEED		(4)
#define MOTION_PARAM_PTP_SPEED			(5)
#define MOTION_PARAM_PTP_LOW_SPEED		(6)
#define MOTION_PARAM_ACCEL				(7)
#define MOTION_PARAM_ORG_DIR			(8)
#define MOTION_PARAM_PTP_DIR			(9)
#define MOTION_PARAM_ENC_SIGN			(10)
#define MOTION_PARAM_ORG_NEED			(11)
#define MOTION_PARAM_ORG_SENSOR			(12)
#define MOTION_PARAM_LEAD				(13)
#define MOTION_PARAM_ENC_PULSE			(14)
#define MOTION_PARAM_SCALE_FACTOR		(15)
#define MOTION_PARAM_NEG_LIMIT			(16)
#define MOTION_PARAM_POS_LIMIT			(17)

// Command State
#define CMD_READY		(0)
#define CMD_RUNNING		(1)
#define CMD_DONE		(2)
#define CMD_POR_RECV	(3)
#define CMD_POS_RECV	(4)
#define CMD_POF_RECV	(5)


void set_cmd_state(int state);
void set_error(int err);
void clear_error();
 
char get_cmd_state();
int  get_current_error();


int get_org_offset(int axis);
int set_org_offset(int axis, int counter);

int get_var(int no);
int set_var(int no, int val);

POINT_DATA get_point_data(int no);
POINT_DATA set_point_data(int no, POINT_DATA pd);

MOTION_PARAM get_motion_param(int axis);
MOTION_PARAM set_motion_param(int axis, MOTION_PARAM param);


// save data to flash
void init_eeprom();

unsigned char* get_motion_param_ptr(int axis, int type);
void save_motion_param_type(int type);
void save_motion_param();

void load_motion_param();
void save_point_data();
void load_point_data();
void save_var();
void load_var();

void save_all();
void load_all();

void init_data();

#endif //#ifndef _DATA_H_