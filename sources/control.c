#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "hardware.h"
#include "control.h"
#include "data.h"
#include "function.h"
#include "defines.h"


extern char g_cMotionComm[MAX_AXIS];
extern MOVE_VARIABLE MovVar[MAX_AXIS];
extern MOTION_PARAM g_MotionParam[MAX_AXIS];

extern char str[128];	// �ø��� �۽��� ���� ���ڿ�

extern char g_MotionCommand;
extern int g_ErrorCode;

extern double motor_x_pitch;
extern double motor_y_pitch;


int g_MoveOffset[MAX_AXIS] = { 0, 0, 0 };
int g_OriginAxis = 0;

STATUS g_Status = { 0, };


#define	ADD_F_START_POS_LSB		(*(volatile unsigned short *)0x80000018)
#define	ADD_F_START_POS_MSB		(*(volatile unsigned short *)0x8000001A)

static int uTmp;


//
// Functions
//
unsigned char get_axis_sensor_index(char axis, char type)
{
	unsigned char val = 0;
	
	if (axis < 0 || axis >= MAX_AXIS)
	{
		return 0;
	}
	
	if (type < 0 || type >= AS_MAX)
	{
		return 0;
	}
	
	switch (type)
	{
	case AS_NEG_LIMIT:	val = g_MotionParam[axis].m_ucNegLimit;	break;
	case AS_POS_LIMIT:	val = g_MotionParam[axis].m_ucPosLimit;	break;
	case AS_HOME: val = g_MotionParam[axis].m_ucOrgSensor;	break;
	}
	
	return val;
}

unsigned char get_axis_sensor_state(char axis, char type)
{
	unsigned char index = 0;
	char sz[64];
	
	if (axis < 0 || axis >= MAX_AXIS)
	{
		return 0;
	}

	if (type < 0 || type >= AS_MAX)
	{
		return 0;
	}
	
	index = get_axis_sensor_index(axis, type);

	return ((GetInput16() >> index) & 1);
}


//
// CALL_FUNCTION()
// must be called in CommXXXX function's switch case (step)
//
#define CALL_FUNCTION(FN, ESTEP)	\
		switch (FN())				\
		{							\
		case NORMAL_FINISHED:		\
			++step;					\
			break;					\
									\
		case ERROR_STOPPED:			\
			step = ESTEP;			\
			break;;					\
		}

#define CALL_FUNCTION_A(FN)			\
		switch (FN)					\
		{							\
		case NORMAL_FINISHED:		\
			++step;					\
			break;					\
									\
		case ERROR_STOPPED:			\
			step = 0;				\
			return ERROR_STOPPED;	\
		}

#define CHECK_USER_STOP()			\
		if (g_UserStop > 0)			\
		{							\
			step = 0;				\
			return NORMAL_FINISHED;	\
		}

static volatile char g_idi_send = 0;
static char g_idi_buffer[256] = "";  

static volatile char g_ido_send = 0;
static char g_ido_buffer[256] = "";  


void SetIdiBuffer(char* str)
{
	send(str);
	return;
	
	if (str)
	{
		while (g_idi_send)
		{
//			DelayMS(1);
		}
		
		g_idi_send = 1;
		strcpy(g_idi_buffer, str);
	}
}

void SetIdoBuffer(char* str)
{
	send(str);
	return;
	
	if (str)
	{
		while (g_ido_send)
		{
//			DelayMS(1);
		}

		g_ido_send = 1;
		strcpy(g_ido_buffer, str);
	}
}

void SendCommResponse()
{
	if (g_idi_send)
	{
		send(g_idi_buffer);
		g_idi_send = 0;
	}


	if (g_ido_send)
	{
		send(g_ido_buffer);
		g_ido_send = 0;
	}
}

char IsCommError(int no)
{
	char res = 0;

	if (no >= COMMUNICATION_ERROR_BASE)
	{
		res = 1;
	}
	
	return res;
}

char IsError()
{
	char res = 0;
	
	if (IsCommError(g_ErrorCode))
	{
		return 0;
	}
	
	if (g_ErrorCode != ERR_NO)
	{
		res = 1;
	}
	
	return res;
}

void SetErrorCode(int err)
{
	if (IsCommError(err))
	{
		if (IsCommError(g_ErrorCode))
		{
			g_ErrorCode = err;
		}
	}
	else
	{
		g_ErrorCode = err;
	}
}

char SetControlCommand(char cmd)
{
	char prev = g_MotionCommand;
	if (prev != cmd)
	{
		g_MotionCommand = cmd;
	}
	
	return prev;
}

void SetCommand(char* cmd)
{
	if (cmd && strlen(cmd) > 0)
	{
		strcpy(g_LastCommand, cmd);
	}
}

void GoToIdle()
{
	g_MotionCommand = COMM_IDLE;
}

void GoToError()
{
	//g_MotionCommand = COMM_ERROR_STOP;
	g_GoToError = 1;
	g_UserStop = 10;
}

void SendResponseRaw(char* command, int errorCode)
{
	char str[64] = "";
	
	if (strlen(command) < 2)
	{
		return;
	}
	
	if (errorCode == 0)
	{
		if (strstr(command, "ERR") != 0)
		{
			sprintf(str, "%s 000\r\n", command);	
		}
		else
		{
			sprintf(str, "%s\r\n", command);
		}
	}
	else if (errorCode < 1000)
	{
		sprintf(str, "%s %03d\r\n", command, errorCode);
	}
	else
	{
		sprintf(str, "%s E%02d\r\n", command, errorCode-1000);
	}

	send(str);
}

void SendResponse()
{
	static char str[32] = "";
	
	if (strlen(g_LastCommand) < 2)
	{
		return;
	}
	
	if (g_ErrorCode == 0)
	{
		if (strstr(g_LastCommand, "ERR") != 0)
		{
			sprintf(str, "%s 000\r\n", g_LastCommand);	
		}
		else
		{
			sprintf(str, "%s\r\n", g_LastCommand);
		}
	}
	else if (g_ErrorCode < 1000)
	{
		sprintf(str, "%s %03d\r\n", g_LastCommand, g_ErrorCode);
	}
	else
	{
		sprintf(str, "%s E%02d\r\n", g_LastCommand, g_ErrorCode-1000);
	}

	send(str);
	
	strcpy(g_LastCommand, "");
}


void Delay1ms()
{
	DelayMS(1);
}

//
// MUST delay after MoveStart()
//
void DelayMoveStart()
{
	DelayMS(20);
}

void UpdateState(char state)
{
	switch (state)
	{
	case NORMAL_RUNNING:
		break;
	
	case NORMAL_FINISHED:
		if (g_ResponseSend == 0)
		{
			SendResponse();
		}
		else
		{
			g_ResponseSend = 0;
		}
		GoToIdle();
		break;
	
	case ERROR_STOPPED:
		SendResponse();
		GoToIdle();
		break;
	}
}

///////////////////////////////////////////////////////////
// 1ms Timer Interrupt �� �̿��� ��� ���� Ŀ��� ó�� ��ƾ
///////////////////////////////////////////////////////////
void MainControl()
{
	char res = 0;
	
	switch (g_MotionCommand)
	{
	case COMM_IDLE:		break;
	case COMM_ORIGIN:	res = CommOrigin();	UpdateState(res);	break;
	case COMM_PTP:		res = CommMove();	UpdateState(res);	break;
	case COMM_ORIGIN_A:	res = CommOriginAxis();	UpdateState(res);	break;
	case COMM_ERROR_STOP:	res = CommErrorStop();	UpdateState(res);	break;
	}
	
	if (g_UserStop > 0)
	{
		--g_UserStop;
	}
	
	if (g_GoToError > 0)
	{
		if (g_UserStop < 3)
		{
			g_MotionCommand = COMM_ERROR_STOP;
			g_GoToError = 0;
		}
	}
}

void SystemCheck()
{
	char res = 0;
	char axis = 0;
	char userEmo = 0;
	char is_stopped = 0;
	char use_UserEMO = 0;
	
	if (g_MotionCommand == COMM_IDLE && IsStopped())
	{
		is_stopped = 1;
	}
	
	// system out
	SetDO(0, is_stopped == 0);
	SetDO(1, is_stopped);
	SetDO(2, g_OriginCompleted);
	SetDO(3, g_ErrorCode == ERR_OVER_RUN);
	SetDO(4, g_ErrorCode != 0);
	
	
	if (use_UserEMO)	// currently not used (1.00.016)
	{
		// User Emergency
		userEmo = GetDI(0);
	
		if (userEmo == 0) // reverse, 0=EMO, 1=Normal
		{
			SetErrorCode(ERR_EMERGENCY);
			GoToError();
			for (axis = 0; axis < MAX_AXIS; ++axis)
			{
				MovVar[g_OriginAxis].m_uAcel = EMERGENCY_STOP_DECEL;
				MoveStop(axis);
			}
			g_OriginCompleted = 0;
			return;
		}
	}

	switch (g_MotionCommand)
	{
	case COMM_IDLE:
		if (IsStopped())
		{
			return;
		}
		break;
	case COMM_ORIGIN:
	case COMM_ERROR_STOP:
		return;
	}
	
	// check limit sensor
	
	if (g_MotionCommand == COMM_IDLE)
	{
		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			if (GetMoveStatus(axis) != MOVE_STS_STOP)
			{
				switch (get_axis_sensor(axis))
				{
				case 0:
					break;
					
				case 1:	// POS LIMIT
					if (MovVar[axis].m_ucDir == g_MotionParam[axis].m_ucMoveDir)
					{
						SetErrorCode(ERR_OVER_RUN);
						StopMotors();
					}
					break;
					
				case 2: // NEG LIMIT
					if (MovVar[axis].m_ucDir != g_MotionParam[axis].m_ucMoveDir)
					{
						SetErrorCode(ERR_OVER_RUN);
						StopMotors();
					}
					break;
					
				default:
					SetErrorCode(ERR_OVER_RUN);
					StopMotors();
					break;		
				}
			}
		}
	}
	else
	{
		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			switch (get_axis_sensor(axis))
			{
			case 0:
				break;
				
			case 1:	// POS LIMIT
				if (g_MoveOffset[axis] > 0)
				{
					SetErrorCode(ERR_OVER_RUN);
					GoToError();
				}
				break;
				
			case 2: // NEG LIMIT
				if (g_MoveOffset[axis] < 0)
				{
					SetErrorCode(ERR_OVER_RUN);
					GoToError();
				}
				break;
				
			default:
				SetErrorCode(ERR_OVER_RUN);
				GoToError();
				break;		
			}
		}
	}
}

unsigned char GetInput(char ch)
{
	unsigned char res = 0;
	unsigned short val = 0;
	
	if (ch < 0 || ch > 1)
	{
		return res;
	}
	
	val = ~GetInput16();
	
	res = (val >> (8*ch)) & 0xFF;
	
	return res;
}

unsigned char GetOutput(char ch)
{
	unsigned char res = 0;
	unsigned short val = 0;
	
	if (ch < 0 || ch > 1)
	{
		return res;
	}
	
	//val = ~GetOutput12();
	val = ~GetOutput8();
	
	res = (val >> (8*ch)) & 0xFF;
	
	return res;
}

void SetOutput(char ch, unsigned char data)
{
	//unsigned short cur = GetOutput12();		// active low
	unsigned short cur = GetOutput8();
	unsigned short data2 = 0xFF & (~data);	// active high -> active low
	
	if (ch < 0 || ch > 1)
	{
		return;
	}
	
	cur = cur & (unsigned short)(~(0xFF << (8*ch)));
	data2 = cur | (unsigned short)(data2 << (8*ch));
	
	// SetOutput12(data2);
	SetOutput8(data2);
}

unsigned char GetInput2(char ch)
{
	unsigned char res = 0;

	if (ch < 1 || ch > MAX_DI)
	{
		return res;
	}
	
	res = GetInput(ch-1);	// GetInput() : ch = 0-base
	
	return res;
}

unsigned char GetOutput2(char ch)
{
	unsigned char res = 0;

	if (ch < 1 || ch > MAX_DO)
	{
		return res;
	}
	
	res = GetOutput(ch-1);	// GetOutput() : ch = 0-base

	return res;	
}

void SetOutput2(char ch, unsigned char data)
{
	if (ch < 1 || ch > MAX_DO)
	{
		return;
	}
	
	SetOutput(ch-1, data);
}

char GetDI(char ch)
{
	char res = 0;

	if (ch >= 0 && ch < MAX_DI)
	{
		res = (((GetInput16() >> ch) & 1) == SENS_ON);
	}
	
	return res;
}

char GetDO(char ch)
{
	char res = 0;

	if (ch >= 0 && ch < MAX_DO)
	{
		//res = (((GetOutput12() >> ch) & 1) == SENS_ON);
		res = (((GetOutput8() >> ch) & 1) == SENS_ON);
	}
	
	return res;
}

void SetDO(char ch, char flag)
{
	char res = 0;
	int value = 0;

	if (ch >= 0 && ch < MAX_DO)
	{
		value = (1 << ch);
		if (flag)
		{
			//SetOutput12(GetOutput12() & ~value);
			SetOutput8(GetOutput8() & ~value);
		}
		else
		{
			//SetOutput12(GetOutput12() | value);
			SetOutput8(GetOutput8() | value);
		}
	}
}

unsigned char GetDIBit(char ch, char bit)
{
	unsigned char res = GetInput2(ch);
	
	res = (res >> bit) & 1;
	
	return res;
}

unsigned char GetDOBit(char ch, char bit)
{
	unsigned char res = GetOutput2(ch);
	
	res = (res >> bit) & 1;
	
	return res;	
}

void SetDOBit(char ch, char bit, char flag)
{
	unsigned char res = GetOutput2(ch);
	
	if (flag == 0)
	{
		res = res & (unsigned char)(~(1 << bit));
	}
	else
	{
		res = res | (1 << bit);
	}
	
	SetOutput2(ch, res);
}

void UpdateOriginCompletedStatus()
{
	char axis = 0;
	char flag = 1;

	for (axis = 0; axis < MAX_AXIS; ++axis)
	{
		if (g_OriginNeed[axis] && g_OriginCompletedAxes[axis] == 0)
		{
			flag = 0;
			break;
		}
	}
	
	g_OriginCompleted = flag;
}

void SetOriginCompletedFlag(char axis, char flag)
{
	if (axis < 0 || axis >= MAX_AXIS)
	{
		return;
	}
	
	if (flag < 0 || flag > 1)
	{
		return;
	}
	
	g_OriginCompletedAxes[axis] = flag;
	
	UpdateOriginCompletedStatus();
}

char IsOriginCompleted()
{
	return g_OriginCompleted;
}

char IsStopped()
{
	char axis = 0;
	char moveStatus = 0;
	
	for (axis = 0; axis < MAX_AXIS; ++axis)
	{
		moveStatus = GetMoveStatus(axis);
		if (moveStatus != MOVE_STS_STOP)
		{
			return 0;
		}
	}
	
	return 1;
}

void HoldMotors()
{
	char axis = 0;
	
	for (axis = 0; axis < MAX_AXIS; ++axis)
	{
		SetHoldTorque(axis);
	}
}

void StopMotors()
{
	char axis = 0;
	
	for (axis = 0; axis < MAX_AXIS; ++axis)
	{
		MoveStop(axis);
	}
}


char CommErrorStop()
{
	static int step = 0;
	
	switch (step)
	{
	// Error Handling
	case 0:
		StopMotors();
		++step;
		break;
	
	case 1:
		if (IsStopped())
		{
			++step;
		}
		break;
		
	case 2:
		HoldMotors();
		step = 0;
		return ERROR_STOPPED;
		
	default:
		step = 0;
		return NORMAL_FINISHED;
	}
	
	return NORMAL_RUNNING;
}

char CommOrigin()
{
	static int step = 0;
	static int nNegLimitInPos[MAX_AXIS] = {0,0,0};
	static int nNegLimitOutPos[MAX_AXIS] = {0,0,0};
	static POINT_DATA pd = { 0,	};
	int temp = 0;
	int axis = 0;
	int flag = 1;
	static int vacuum_flag = 0;
	static int timerCount = 0;
	int res = 0;
	char move_status = 0;

	switch (step)
	{
	// Error Handling
	case 91:
		StopMotors();
		++step;
		break;
	
	case 92:
		if (IsStopped())
		{
			++step;
		}
		break;
		
	case 93:
		HoldMotors();
		// VACUUM_OFF;
		step = 0;
		return ERROR_STOPPED;

	// Start
	case 0 :	// Motion ���������� Origin Parameter �� �����Ѵ�.
		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			SetOriginCompletedFlag(axis, 0);
		}

		step = 3;
		break;
		
	case 1:
		++step;
		break;
		
	case 2:
		++step;
		break;
		
	case 3:
		MovVar[Z_AXIS].m_uS = g_MotionParam[Z_AXIS].m_uOrgSLimit;
		MovVar[Z_AXIS].m_ucDir = g_MotionParam[Z_AXIS].m_ucOrgDir;
		SetSpeed(Z_AXIS, SPEED_ORG);
		
		if (NEG_LIMIT(Z_AXIS) != SENS_ON && MoveStart(Z_AXIS))
		{
			step = 91;
			return NORMAL_RUNNING;
		}
		else
		{
			++step;
			debugf("step=%d", step);
		}
		
		DelayMoveStart();
		
		break;
		
		
	case 4 :
		if (HOME_SENSOR(Z_AXIS) == SENS_ON)
		{
			MoveStop(Z_AXIS);
			++step;
		}
		
		break;

	case 5 :
		if (IsStopped())
		{
			++step;
		}

		break;
		

	case 6 :
	for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			MovVar[axis].m_uS = g_MotionParam[axis].m_uOrgSLimit;
			MovVar[axis].m_ucDir = g_MotionParam[axis].m_ucOrgDir;
			SetSpeed(axis, SPEED_ORG);
		}

		if (NEG_LIMIT(X_AXIS) != SENS_ON && MoveStart(X_AXIS))
		{
			step = 91;
			return NORMAL_RUNNING;
		}

		if (NEG_LIMIT(Y_AXIS) != SENS_ON && MoveStart(Y_AXIS))
		{
			step = 91;
		}
		else
		{
			++step;
			debugf("step=%d", step);
		}
		
		DelayMoveStart();
		
		break;
		
	case 7 :
		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			if (HOME_SENSOR(axis) == SENS_ON)	// NegLimit Sensor�� ON �� �� ���� �̵��Ѵ�.
			{
				MoveStop(axis);
			}
			else
			{
				flag = 0;
			}
		}
		
		if (flag == 1)
		{
			++step;
			debugf("step=%d", step);
		}
		
		break;

	case 8 :
		if (IsStopped())
		{
			++step;
		}

		break;
		
	case 9 :
		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			CounterReset(axis);
			MovVar[axis].m_ucDir = (unsigned char)((~g_MotionParam[axis].m_ucOrgDir) & 1);	// Positive ��������
			MovVar[axis].m_uAcel = 0;								// ���� �ӵ��� ����

			if (MoveStart(axis))
			{
				step = 91;
				SetErrorCode(ERR_MOTOR_ERROR);
				return NORMAL_RUNNING;
			}
		}
		
		DelayMoveStart();
		
		++step;		
		debugf("step=%d", step);
		break;

	case 10 :
		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			if (HOME_SENSOR(axis) == SENS_OFF)	// NegLimit Sensor OFF �� �� ���� �̵��Ѵ�.
			{
				MoveStop(axis);
			}
			else
			{
				flag = 0;
			}
		}
		
		if (flag == 1)
		{
			++step;
			debugf("step=%d", step);
		}
		
		break;

	case 11 :
		if (IsStopped())
		{
			++step;
			debugf("step=%d", step);
		}
		
		break;
		
	case 12 :
		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			nNegLimitOutPos[axis] = CounterRead(axis);				// ���������� Positive �������� �̵��Ͽ� ������ OFF �� ��ġ (��ġ ����)
			MovVar[axis].m_ucDir = g_MotionParam[axis].m_ucOrgDir;	// Negative ��������
			MovVar[axis].m_uAcel = 0;								// ���� �ӵ��� ����

			if (MoveStart(axis))
			{
				step = 91;
				SetErrorCode(ERR_MOTOR_ERROR);
				return NORMAL_RUNNING;
			}
		}
		
		DelayMoveStart();
		
		++step;		
		debugf("step=%d", step);
		break;

	case 13 :
		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			if (HOME_SENSOR(axis) == SENS_ON)	// NegLimit Sensor ON �� �� ���� �̵��Ѵ�.
			{
				MoveStop(axis);
			}
			else
			{
				flag = 0;
			}
		}
		
		if (flag == 1)
		{
			++step;
			debugf("step=%d", step);
		}
		
		break;

	case 14 :
		if (IsStopped())
		{
			++step;
			debugf("step=%d", step);
		}
		
		break;
	
	case 15 :
		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			nNegLimitInPos[axis] = CounterRead(axis);	// ���������� Negative �������� �̵��Ͽ� ������ ON �� ��ġ (��ġ ����)

			temp = ((nNegLimitInPos[axis] + nNegLimitOutPos[axis])/2) - nNegLimitInPos[axis]; // (������ġ-������ġ)
			if (temp < 0)
			{
				temp = -temp;
			}
			MovVar[axis].m_uS = temp;
			MovVar[axis].m_ucDir = (unsigned char)((~g_MotionParam[axis].m_ucOrgDir) & 1);

			SetSpeed(axis, SPEED_ORG);

			if (MoveStart(axis))
			{
				step = 91;
				SetErrorCode(ERR_MOTOR_ERROR);
				return NORMAL_RUNNING;					// ���� : ������ ����
			}
		}
		
		DelayMoveStart();
		
		++step;		
		debugf("step=%d", step);
		break;

	case 16 :
		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			move_status = GetMoveStatus(axis);
			switch (move_status)
			{
			case MOVE_STS_INPOS:
				MoveStop(axis);
				break;
			
			case MOVE_STS_STOP:
				break;
			
			default:
				flag = 0;
				break;
			}
		}
		
		if (flag == 1)
		{
			++step;
			debugf("step=%d", step);
		}

		break;

	case 17 :
		if (IsStopped())
		{
			++step;
			debugf("step=%d", step);
		}

		break;
		
	case 18 :
		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			CounterReset(axis);	// �������� �����Ѵ�.
			SetSpeed(axis, SPEED_NORMAL);
		}
		
		++step;
		
	case 19:
		++step;
		break;
		
	case 20:
		// VACUUM_OFF;
		++step;
		timerCount = 0;
		break;
		
	case 21:
		if (timerCount > 300)
		{
			++step;
			break;
		}
		
		++timerCount;
		Delay1ms();
		break;
		
	case 22:
		step = 0;

		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			SetOriginCompletedFlag(axis, 1);
		}

		return NORMAL_FINISHED;	// ���� : Move Origin ���� �Ϸ�

	default:
		return NORMAL_FINISHED;

	}
	
	CHECK_USER_STOP();
	
	return NORMAL_RUNNING;
}

char CommOriginAxis()
{
	static int step = 0;
	static int nNegLimitInPos[MAX_AXIS] = {0,0,0};
	static int nNegLimitOutPos[MAX_AXIS] = {0,0,0};
	static POINT_DATA pd = { 0,	};
	int temp = 0;
	int axis = 0;
	int flag = 1;
	static int vacuum_flag = 0;
	static int timerCount = 0;
	int res = 0;
	char move_status = 0;
	
	if (g_OriginAxis < 0 || g_OriginAxis >= MAX_AXIS)
	{
		SetErrorCode(ERR_WRONG_COMMAND);
		return ERROR_STOPPED;
	}

	switch (step)
	{
	// Error Handling
	case 91:
		StopMotors();
		++step;
		break;
	
	case 92:
		if (IsStopped())
		{
			++step;
		}
		break;
		
	case 93:
		HoldMotors();
		// VACUUM_OFF;
		step = 0;
		return ERROR_STOPPED;

	// Start
	case 0:	// Motion ���������� Origin Parameter �� �����Ѵ�.
		SetOriginCompletedFlag(g_OriginAxis, 0);
		
		MovVar[g_OriginAxis].m_uS = g_MotionParam[g_OriginAxis].m_uOrgSLimit;
		MovVar[g_OriginAxis].m_ucDir = g_MotionParam[g_OriginAxis].m_ucOrgDir;
		SetSpeed(g_OriginAxis, SPEED_ORG);
		
		if (NEG_LIMIT(g_OriginAxis) != SENS_ON && MoveStart(g_OriginAxis))
		{
			step = 91;
			return NORMAL_RUNNING;
		}
		
		DelayMoveStart();
		
		++step;
		debugf("step=%d", step);
		break;
		
	case 1:
		if (HOME_SENSOR(g_OriginAxis) == SENS_ON)	// NegLimit Sensor�� ON �� �� ���� �̵��Ѵ�.
		{
			MoveStop(g_OriginAxis);
			++step;
		}
		
		break;

	case 2:
		if (IsStopped())
		{
			++step;
		}

		break;
		
	case 3:
		CounterReset(g_OriginAxis);
		MovVar[g_OriginAxis].m_ucDir = (unsigned char)((~g_MotionParam[g_OriginAxis].m_ucOrgDir) & 1);	// Positive ��������
		MovVar[g_OriginAxis].m_uAcel = 0;								// ���� �ӵ��� ����

		if (MoveStart(g_OriginAxis))
		{
			step = 91;
			SetErrorCode(ERR_MOTOR_ERROR);
			return NORMAL_RUNNING;
		}
		
		DelayMoveStart();
		
		++step;		
		debugf("step=%d", step);
		break;

	case 4:
		if (HOME_SENSOR(g_OriginAxis) == SENS_OFF)	// NegLimit Sensor OFF �� �� ���� �̵��Ѵ�.
		{
			MoveStop(g_OriginAxis);
			++step;
		}
		
		break;

	case 5:
		if (IsStopped())
		{
			++step;
			debugf("step=%d", step);
		}
		
		break;
		
	case 6:
		nNegLimitOutPos[g_OriginAxis] = CounterRead(g_OriginAxis);				// ���������� Positive �������� �̵��Ͽ� ������ OFF �� ��ġ (��ġ ����)
		MovVar[g_OriginAxis].m_ucDir = g_MotionParam[g_OriginAxis].m_ucOrgDir;	// Negative ��������
		MovVar[g_OriginAxis].m_uAcel = 0;								// ���� �ӵ��� ����

		if (MoveStart(g_OriginAxis))
		{
			step = 91;
			SetErrorCode(ERR_MOTOR_ERROR);
			return NORMAL_RUNNING;
		}
		
		DelayMoveStart();
		
		++step;		
		debugf("step=%d", step);
		break;

	case 7:
		if (HOME_SENSOR(g_OriginAxis) == SENS_ON)	// NegLimit Sensor ON �� �� ���� �̵��Ѵ�.
		{
			MoveStop(g_OriginAxis);
			++step;
		}
		
		break;

	case 8:
		if (IsStopped())
		{
			++step;
			debugf("step=%d", step);
		}
		
		break;
	
	case 9:
		nNegLimitInPos[g_OriginAxis] = CounterRead(g_OriginAxis);	// ���������� Negative �������� �̵��Ͽ� ������ ON �� ��ġ (��ġ ����)

		temp = ((nNegLimitInPos[g_OriginAxis] + nNegLimitOutPos[g_OriginAxis])/2) - nNegLimitInPos[g_OriginAxis]; // (������ġ-������ġ)
		if (temp < 0)
		{
			temp = -temp;
		}
		MovVar[g_OriginAxis].m_uS = temp;
		MovVar[g_OriginAxis].m_ucDir = (unsigned char)((~g_MotionParam[g_OriginAxis].m_ucOrgDir) & 1);

		SetSpeed(g_OriginAxis, SPEED_ORG);

		if (MoveStart(g_OriginAxis))
		{
			step = 91;
			SetErrorCode(ERR_MOTOR_ERROR);
			return NORMAL_RUNNING;					// ���� : ������ ����
		}
		
		DelayMoveStart();
		
		++step;		
		debugf("step=%d", step);
		break;

	case 10:
		move_status = GetMoveStatus(g_OriginAxis);
		switch (move_status)
		{
		case MOVE_STS_INPOS:
			MoveStop(g_OriginAxis);
			++step;
			break;
		
		case MOVE_STS_STOP:
			++step;
			break;
		}

		break;

	case 11:
		if (IsStopped())
		{
			++step;
			debugf("step=%d", step);
		}

		break;
		
	case 12:
		CounterReset(g_OriginAxis);	// �������� �����Ѵ�.
		SetSpeed(g_OriginAxis, SPEED_NORMAL);
		
		++step;
		
	case 13:
		//CALL_FUNCTION(CommRST, 91);
		++step;
		break;
		
	case 14:
		step = 0;

		SetOriginCompletedFlag(g_OriginAxis, 1);

		return NORMAL_FINISHED;	// ���� : Move Origin ���� �Ϸ�

	default:
		return NORMAL_FINISHED;

	}
	
	CHECK_USER_STOP();
	
	return NORMAL_RUNNING;
}

char CommMove()
{
	char axis = 0;
	static int state[MAX_AXIS] = {0,0,0};
	unsigned char dir = 0;
	static int step = 0;
	int flag = 1;
	char move_status = 0;
	char move_start = 0;
	
	switch (step)
	{
	// Error Handling
	case 91:
		StopMotors();
		++step;
		break;
	
	case 92:
		if (IsStopped())
		{
			++step;
		}
		break;
		
	case 93:
		HoldMotors();
		step = 0;

		return ERROR_STOPPED;

	case 0:
		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			if (g_MoveOffset[axis] != 0)
			{
				SetMoveOffset(axis, g_MoveOffset[axis]);
				SetSpeed(axis, SPEED_NORMAL);
				
				debugf("Axis(%d) Distance = %d", axis, MovVar[axis].m_uS);
				debugf("Axis(%d) Direction = %d", axis, MovVar[axis].m_ucDir);
				debugf("Axis(%d) Vmax = %d", axis, MovVar[axis].m_uVmax);
				debugf("Axis(%d) Vmin = %d", axis, MovVar[axis].m_uVmin);
				debugf("Axis(%d) Accel = %d", axis, MovVar[axis].m_uAcel);
				debugf("Axis(%d) Move Torque = %d", axis, MovVar[axis].m_ucMoveTorq);
				debugf("Axis(%d) Hold Torque = %d", axis, MovVar[axis].m_ucHoldTorq);
				
				debugf("MoveStart(%d)", axis);
				
				move_start = MoveStart(axis);
				if (move_start)
				{
					debugf("GetMoveStatus: %d,%d,%d", 
						GetMoveStatus(0),
						GetMoveStatus(1),
						GetMoveStatus(2));
				
					debugf("Error Axis=%d, Code=%d", axis, move_start);
					
					SetErrorCode(ERR_MOTOR_ERROR);

					step = 91;
					return NORMAL_RUNNING;
				}
			}
		}
		
		DelayMoveStart();
		
		++step;
		debugf("step=%d", step);
		break;

	case 1 :
		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			move_status = GetMoveStatus(axis);
			switch (move_status)
			{
			case MOVE_STS_INPOS:
				MoveStop(axis);
				break;

			case MOVE_STS_STOP:
				break;
		
			default:
				flag = 0;
				break;
			}
		}
		
		if (flag == 1)
		{
			++step;
			debugf("step=%d", step);
		}

		break;

	case 2 :
		if (IsStopped())
		{
			++step;
			debugf("step=%d", step);
		}
		
		break;

	case 3 :
		debugf("SetHoldTorque");
		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			SetHoldTorque(axis);
			g_MoveOffset[axis] = 0;
		}

		step = 0;
		
		return NORMAL_FINISHED;	// ���� : Move Origin ���� �Ϸ�

	default:
		step = 0;
		return NORMAL_FINISHED;	// ���� : Move Origin ���� ��?
	}

	CHECK_USER_STOP();
	
	return 0;
}


///////////////////////////////////////////////////////////
// �� ���� ���� ���� �� Counter ������ �ʱ�ȭ �Ѵ�.
///////////////////////////////////////////////////////////
void InitAxis()
{
	char axis;

	for (axis = 0 ; axis < MAX_AXIS ; axis++)
	{
		debugf("SetCounterDirection(%d, %d)", axis, (unsigned char)((~g_MotionParam[axis].m_ucOrgDir) & 1));
		
		SetCounterDirection(axis, (unsigned char)((~g_MotionParam[axis].m_ucOrgDir) & 1));

		MovVar[axis].m_ucHoldTorq = (axis == Z_AXIS) ? HOLD_TORQUE : HOLD_TORQUE_3A;
		SetHoldTorque(axis);

		DriverEnable(axis);
		
		DelayMS(500);
		
		if (DriverErrorCheck(axis) != 0)
		{
			debugf("Axis(%d) DriverError!!! (%d)", axis, DriverErrorCheck(axis));
		}
		
		CounterReset(axis);
	}
}

void JogStart(char axis, char dir)
{
	static char str[128];
	
	sprintf(str, "JogStart(%d, %d)", axis, dir);
	debug(str);
	
	if (dir == 0)	// inverse
	{
		dir = InverseDir(g_MotionParam[axis].m_ucMoveDir);
	}
	else
	{
		dir = g_MotionParam[axis].m_ucMoveDir;
	}
	

	MovVar[axis].m_uS    = g_MotionParam[axis].m_uOrgSLimit;
	MovVar[axis].m_ucDir = dir;
	SetSpeed(axis, SPEED_JOG);
	MovVar[axis].m_ucMoveTorq = (axis == Z_AXIS) ? MOVE_TORQUE : MOVE_TORQUE_3A;
	
	if (MoveStart(axis))
	{
		SetHoldTorque(axis);		// Jog �Լ��� �����ϹǷ� Holding Current�� �����.
	}
	
	DelayMoveStart();
}

void JogStop(char axis)
{
	debug("JogStop()");

	MoveStop(axis);
}

void MotionStop()
{
	char axis = 0;
	
	for (axis = 0; axis < MAX_AXIS; ++axis)
	{
		MoveStop(axis);
		SetHoldTorque(axis);
		g_cMotionComm[axis] = COMM_IDLE;
	}
}

unsigned char GetRawDir(char axis, unsigned char dir)
{
	unsigned char retDir = dir;

	// change the direction for X-axis
	if (axis == X_AXIS)
	{
		retDir = 1 - retDir;
	}
	
	return retDir;
}

unsigned char InverseDir(unsigned char dir)
{
	return 1 - dir;
}

void SetMoveOffset(char axis, int offset)
{
	unsigned char dir = g_MotionParam[axis].m_ucMoveDir;
	if (offset < 0)
	{
		dir = InverseDir(dir);
		offset = -offset;
	}
	
	MovVar[axis].m_uS    = offset;
	MovVar[axis].m_ucDir = dir;
}

void SetSpeed(char axis, int type)
{
	int scan_rate = get_var(VAR_SCAN_SPEED);

	if (scan_rate < 1 || scan_rate > 100)
	{
		scan_rate = 100;
	}
	
	if (axis < 0 || axis >= MAX_AXIS)
	{
		return;
	}
	
	switch (type)
	{
		case SPEED_ORG:
			MovVar[axis].m_uVmax = g_MotionParam[axis].m_uOrgVmax;
			MovVar[axis].m_uVmin = g_MotionParam[axis].m_uOrgVmax;
			MovVar[axis].m_uAcel = g_MotionParam[axis].m_uOrgAcel;
			break;
			
		case SPEED_JOG:
			MovVar[axis].m_uVmax = g_MotionParam[axis].m_uJogVmax;
			MovVar[axis].m_uVmin = g_MotionParam[axis].m_uJogVmin;
			MovVar[axis].m_uAcel = g_MotionParam[axis].m_uJogAcel;
			break;
			
		case SPEED_NORMAL:
			MovVar[axis].m_uVmax = g_MotionParam[axis].m_uNorVmax;
			MovVar[axis].m_uVmin = g_MotionParam[axis].m_uNorVmin;
			MovVar[axis].m_uAcel = g_MotionParam[axis].m_uNorAcel;
			break;

		case SPEED_SCAN:
			MovVar[axis].m_uVmax = ((double)scan_rate/100) * g_MotionParam[axis].m_uNorVmax;
			MovVar[axis].m_uVmin = ((double)scan_rate/100) * g_MotionParam[axis].m_uNorVmin;
			MovVar[axis].m_uAcel = ((double)scan_rate/100) * g_MotionParam[axis].m_uNorAcel;
			break;
	}
	
	MovVar[axis].m_ucMoveTorq = (axis == Z_AXIS) ? MOVE_TORQUE : MOVE_TORQUE_3A;
}


