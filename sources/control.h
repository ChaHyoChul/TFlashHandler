#ifndef _CONTROL_H
#define _CONTROL_H

#include "data.h"


// 모션 제어 명령어
#define	COMM_IDLE			0
#define	COMM_ORIGIN			1	// 
#define	COMM_ALIGN			2	// 
#define COMM_RST			3
#define COMM_PTP			4
#define COMM_JOG_P			5
#define COMM_JOG_N			6
#define COMM_JOG_STOP		7
#define COMM_STOP			8
#define COMM_E_STOP			9
#define COMM_ERROR_STOP		10
#define COMM_ORIGIN_A		18

// speed type at SetSpeed()
#define SPEED_ORG			(0)
#define SPEED_NORMAL		(1)
#define SPEED_JOG			(2)
#define SPEED_SCAN			(3)

// return code
#define NORMAL_RUNNING		(0)
#define NORMAL_FINISHED		(1)
#define ERROR_STOPPED		(2)

// Z-Move : Point Data No
#define Z_DOWN				(1)
#define Z_UP				(2)
#define Z_MID				(3)
#define Z_ALIGN_DOWN		(11)


// =======================================
// 	Axis Sensor
// =======================================
enum AxisSensor
{
	AS_NEG_LIMIT,
	AS_POS_LIMIT,
	AS_HOME,
	AS_MAX
};

unsigned char get_axis_sensor_index(char axis, char type);
unsigned char get_axis_sensor_state(char axis, char type);

#define NEG_LIMIT(axis)		(get_axis_sensor_state(axis, AS_NEG_LIMIT))
#define POS_LIMIT(axis)		(get_axis_sensor_state(axis, AS_POS_LIMIT))
#define HOME_SENSOR(axis)	(get_axis_sensor_state(axis, AS_HOME))


typedef struct _STATUS
{
	char running;
	char stop;
	char error;
	char origin;
} STATUS;

extern STATUS g_Status;
extern char g_LastCommand[32];

typedef char state_t;
typedef char (*sfn_t)(char);

typedef struct _STATE_MAP
{
	sfn_t	sfn;
	state_t	next;
} STATE_MAP;


void SetIdiBuffer(char* str);
void SetIdoBuffer(char* str);
void SendCommResponse();

// New Functions
char IsCommError(int no);
char IsError();
void SetErrorCode(int err);
char SetControlCommand(char cmd);
void SetCommand(char* cmd);
void GoToIdle();
void GoToError();
void SendResponseRaw(char* command, int errorCode);
void SendResponse();
void Delay1ms();
void DelayMoveStart();
void UpdateState(char state);
void MainControl();
void SystemCheck();

// GetInput/GetOutput
// ch : 0-base
// Active High
// *hardware.h functions : Active Low (GetInput16, GetOutput12, SetOutput12)
unsigned char GetInput(char ch);
unsigned char GetOutput(char ch);
void SetOutput(char ch, unsigned char data);

// GetInput2/GetOutput2
// ch : 1~
// reversion flag is applied
// cf. GetInput/GetOutput : ch = 0-base
unsigned char GetInput2(char ch);
unsigned char GetOutput2(char ch);
void SetOutput2(char ch, unsigned char data);

char GetDI(char ch);
char GetDO(char ch);
void SetDO(char ch, char flag);

unsigned char GetDIBit(char ch, char bit);
unsigned char GetDOBit(char ch, char bit);
void SetDOBit(char ch, char bit, char flag);


int  GetResetPos(POINT_DATA* pd);
int  PrepareRST();
char CommErrorStop();
char CommOrigin();
char CommOriginAxis();
char CommMove();


void UpdateOriginCompletedStatus();
void SetOriginCompletedFlag(char axis, char flag);
char IsOriginCompleted();
char IsStopped();
void HoldMotors();
void StopMotors();


// 모션 관련 파라미터 초기화
void InitAxis();

// 모션 제어 루틴 (1ms 타이머 인터럽트)
void MainControl();


void JogStart(char axis, char dir);
void JogStop(char axis);

void MotionStop();

unsigned char GetRawDir(char axis, unsigned char dir);
unsigned char InverseDir(unsigned char dir);

void SetMoveOffset(char axis, int offset);	// set to move distance
void SetSpeed(char axis, int type);			// set speed type


#endif /* _CONTROL_H */
