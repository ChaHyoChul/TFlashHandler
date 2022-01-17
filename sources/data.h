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

extern char g_MoveStartErrorCode[3];
extern int g_MoveStartErrorLine;

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
	unsigned int 	m_uOrgSLimit;	//00 4(4)���� ������ ����Ʈ ����Ʈ (���� : pulse, ���� : 0 ~ 2147483647)
	unsigned char 	m_ucOrgDir;		//01 5(1)���� ������ ã�ư� ���� (0 : CW, 1 : CCW)
	unsigned char	m_ucOrgNeed;	//02 6(1)���� ���� �ʿ� ����
	unsigned int 	m_uOrgVmax;		//03 10(4)���� ������ �ִ� �ӵ� (���� : pps, ���� : 0 ~ 524287)
	unsigned int 	m_uOrgVmin;		//04 14(4)���� ������ ���� �ӵ� (���� : pps, ���� : 0 ~ 32767)
	unsigned int 	m_uOrgAcel;		//05 18(4)���� ������ ��/�� �ӵ� (���� : ?, ���� : 0 ~ 1023)
	unsigned int 	m_uNorVmax;		//06 22(4)Normal ���۽� �ִ� �ӵ� (���� : pps, ���� : 0 ~ 524287)
	unsigned int 	m_uNorVmin;		//07 26(4)Normal ���۽� ���� �ӵ� (���� : pps, ���� : 0 ~ 32767)
	unsigned int 	m_uNorAcel;		//08 30(4)Normal ���۽� ��/�� �ӵ� (���� : ?, ���� : 0 ~ 1023)
	unsigned int	m_uJogVmax;		//09 34(4)JOG ���۽� �ִ� �ӵ� (���� :pps : 0 ~ 524287)
	unsigned int	m_uJogVmin;		//10 38(4)JOG ���۽� ���� �ӵ� (���� :pps : 0 ~ 32767)
	unsigned int	m_uJogAcel;		//11 42(4)JOG ���۽� ��/�� �ӵ� (���� :pps : 0 ~ 1023)
	int 			m_nHomeOff;		//12 46(4)�������� Home ��ġ������ Offset (���� : pulse)
	unsigned char	m_ucMoveDir;	//13 47(1)���� �̵� ����
	unsigned char	m_ucEncSign;	//14 48(1)���ڴ� ��ȣ
	double			m_fLead;		//15 56(8)Lead
	unsigned int	m_ucEncPulse;	//16 60(4)���� 1ȸ�� �޽��� (�⺻ : 3200)
	double			m_fScaleFactor;	//17 68(8)���� Scale Facotr ( = m_fLead/m_ucEncPulse )
	unsigned char	m_ucOrgSensor;	//18 69(1)Origin Sensor
	unsigned char	m_ucNegLimit;	//19 70(1)Negative Limit Sensor
	unsigned char	m_ucPosLimit;	//20 71(1)Positive Limit Sensor
	unsigned char 	m_ucHoldTorque;	//21 72(1)Hold Torque 
	unsigned char 	m_ucMoveTorque;	//22 73(1)Move Torque 
	unsigned char 	m_reversed[7];	// 
} MOTION_PARAM;

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
#define MOTION_PARAM_HOLD_TROQUE		(18)
#define MOTION_PARAM_MOVE_TROQUE		(19)
#define MOTION_PARAM_JOG_ACCEL			(20)

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
void save_motion_param();

void load_motion_param();
void save_point_data();
void load_point_data();
void save_var();
void load_var();

void save_all();
void load_all();

void init_data();

void reset_motion_param();

#endif //#ifndef _DATA_H_
