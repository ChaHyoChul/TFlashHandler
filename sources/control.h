#ifndef _CONTROL_H
#define _CONTROL_H

#include "data.h"


// ��� ���� ���ɾ�
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

#define COMM_HOME			31	// homming 
#define COMM_MGRI			32	// grip					PD1
#define COMM_MUNG			33	// ungrip 				PD2 	
#define COMM_MLOA			34	// movexy(load) 		PD3
#define COMM_MASP			35 	// movexy(aspirate) 	PD4
#define COMM_MDIS			36 	// movexy(dispense) 	PD5  
#define COMM_MSHA 			37 	// shake 				Not used PD VAR6(회수) VAR7(Delay)  
#define COMM_MWAS			38 	// waste 				PD8   VAR8(Delay) 
#define COMM_MSEP			39 	// separate 			PD9-10 
#define COMM_AWAS			40 	// async waste 			PD11 -> PD08 (PD11 이동 -> Output On PD8)
#define COMM_MWRD			41 	// ready waste
#define COMM_MWPR			42 	// pour waste
#define COMM_EQIL			43	// separate long shape
#define COMM_EQIS			44  // separate short shape 
#define COMM_MSHK 			45 	// shake 				PD6 for ready pos. VAR6(회수) VAR7(Delay)  
#define COMM_SWIRL			46 	// shake 
#define COMM_MMLD			47 	// move x,y
#define COMM_MAMV			48	// move x,y 
#define COMM_MRGI			49 	// re-grip 
#define COMM_RASP			50 	// re-grip and aspirate 
#define COMM_RAMV			51 	// regrip and move x,y 

// speed type at SetSpeed()
#define SPEED_ORG			(0)
#define SPEED_NORMAL		(1)
#define SPEED_JOG			(2)
#define SPEED_SEPARATE		(3)

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

/*
#define COMM_HOME			31	// homming 
#define COMM_MGRI			32	// grip					PD1
#define COMM_MUNG			33	// ungrip 				PD2 	
#define COMM_MLOA			34	// movexy(load) 		PD3
#define COMM_MASP			35 	// movexy(aspirate) 	PD4
#define COMM_MDIS			36 	// movexy(dispense) 	PD5  
#define COMM_MSHA 			37 	// shake 				PD6-7 VAR6(회수) VAR7(Delay)  
#define COMM_MWAS			38 	// waste 				PD8   VAR8(Delay) 
*/
char CommHome();
char CommHomeOption1();
char CommGripUngrip();
char CommMoveXY();
char CommShake();
char CommWaste();
char CommAsyncWaste();
char CommReadyWaste();
char CommPourWaste();
char CommSeparate();
char CommSeparateLongSide();
char CommSeparateShortSide();
char CommShakeUsingPD6();
char CommSWIRL();
char CommMMLD();
char CommMAMV();
char CommMRGI();
char CommRASP();
char CommRAMV();

void UpdateOriginCompletedStatus();
void SetOriginCompletedFlag(char axis, char flag);
char IsOriginCompleted();
char IsStopped();
void HoldMotors();
void StopMotors();

// ��� ���� �Ķ���� �ʱ�ȭ
void InitAxis();

// ��� ���� ��ƾ (1ms Ÿ�̸� ���ͷ�Ʈ)
void MainControl();


void JogStart(char axis, char dir);
void JogStop(char axis);

void MotionStop();

unsigned char GetRawDir(char axis, unsigned char dir);
unsigned char InverseDir(unsigned char dir);

void SetMoveOffset(char axis, int offset);	// set to move distance
void SetSpeed(char axis, int type);			// set speed type
void SetSpeedRatio(char axis, int type, int ratio);

void ReleaseBreak();
void HoldBreak();

char IsReleaseBreak();			// 브레이크가 플렸는지 확인 
char IsExistFlask();			// 플라스크가 있는지 확인 
char IsGrip();					// Grip 명령 후 Grip 센서를 확인 한다 

char AsyncWasteOn();
char AsyncWasteOff();

char GetWasteAsyncInput();		// Async Waste 에서 사용하는 input bit를 읽는다 
								// V13번에 읽을 bit 번호를 저장한다 
								// V13=15 일 경우, 보드의 16번 input을 읽는다 
								// bit 번호는 0-base 
								// 보드의 input 은 1-base 

char IsGripperGripPosition();	// Z축의 현재 위치가 Grip 위치이면 1 리턴, 그렇지 않으면 0 리턴 

char IsWasteReadyPos();			// 현재 위치가 Waste Ready 위치이면 1 리턴, 그렇지 않으면 0 리턴 


#endif /* _CONTROL_H */
