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

extern char str[128]; // �ø��� �۽��� ���� ���ڿ�

extern char g_MotionCommand;
extern int g_ErrorCode;

extern double motor_x_pitch;
extern double motor_y_pitch;

int g_MovePointDataNo = 0;
int g_MoveRatio = 100;
int g_MoveRatioSeparate = 100;
int g_MoveRatio_Rotation = 100;
int g_ShakeCount = 0;
int g_ShakeAngle = 0;
int g_MoveOffset[MAX_AXIS] = {0, 0, 0};
int g_OriginAxis = 0;
int g_ShakeAngleX = 0;
int g_ShakeAngleY = 0;
int g_ShakeTiltDelay = 0;
double g_fMoveXPos = 0.0;
double g_fMoveYPos = 0.0;
double g_fRegripXPos = 0.0;
double g_fRegripYPos = 0.0;
double g_fRegripZPos = 0.0;
int g_nRegripDelay = 0;
double g_fMASPOffset[2] = {0.0, 0.0};

STATUS g_Status = {
	0,
};

#define ADD_F_START_POS_LSB (*(volatile unsigned short *)0x80000018)
#define ADD_F_START_POS_MSB (*(volatile unsigned short *)0x8000001A)

static int uTmp;

//
//
//
int move_pd(int pd_no, char sel_axis);
int move_pd_with_speed(int pd_no, int sel_axis, int spd_type);
int move_pd_with_speed_ratio(int pd_no, int sel_axis, int spd_type, int spd_ratio);
int move_pd_with_speed_ratio_xy_offset(int pd_no, int sel_axis, int spd_type, int spd_ratio, double xoffset, double yoffset);
int move_inc(int sel_axis, float dist[], int spd_type, int spd_ratio);
int move_abs(int sel_axis, float dist[], int spd_type, int spd_ratio);
char move_done(char sel_axis);
int do_shake_xy(char reset_step, int axis, int angle, int count);

int breakRelease();

//
// Limit Count
// axis 축의 limit count 를 1 증가하고,
// max limit count 보다 크면 1을 리턴,
// 1을 리턴하면, overrun 에러를 발생한다
int g_PosLimitCount[3] = {0, 0, 0};
int g_NegLimitCount[3] = {0, 0, 0};
void reset_limit_count(int axis);
int inc_pos_limit_count(int axis);
int inc_neg_limit_count(int axis);

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
	case AS_NEG_LIMIT:
		val = g_MotionParam[axis].m_ucNegLimit;
		break;
	case AS_POS_LIMIT:
		val = g_MotionParam[axis].m_ucPosLimit;
		break;
	case AS_HOME:
		val = g_MotionParam[axis].m_ucOrgSensor;
		break;
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
#define CALL_FUNCTION(FN, ESTEP) \
	switch (FN())                \
	{                            \
	case NORMAL_FINISHED:        \
		++step;                  \
		break;                   \
                                 \
	case ERROR_STOPPED:          \
		step = ESTEP;            \
		break;                   \
		;                        \
	}

#define CALL_FUNCTION_A(FN)   \
	switch (FN)               \
	{                         \
	case NORMAL_FINISHED:     \
		++step;               \
		break;                \
                              \
	case ERROR_STOPPED:       \
		step = 0;             \
		return ERROR_STOPPED; \
	}

#define CHECK_USER_STOP()       \
	if (g_UserStop > 0)         \
	{                           \
		step = 0;               \
		return NORMAL_FINISHED; \
	}

static volatile char g_idi_send = 0;
static char g_idi_buffer[256] = "";

static volatile char g_ido_send = 0;
static char g_ido_buffer[256] = "";

void SetIdiBuffer(char *str)
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

void SetIdoBuffer(char *str)
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
		g_BreakReleaseStepNo = 0;
		g_MotionCommand = cmd;
	}

	return prev;
}

void SetCommand(char *cmd)
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
	// g_MotionCommand = COMM_ERROR_STOP;
	g_GoToError = 1;
	g_UserStop = 10;
}

void SendResponseRaw(char *command, int errorCode)
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
		sprintf(str, "%s E%02d\r\n", command, errorCode - 1000);
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
		sprintf(str, "%s E%02d\r\n", g_LastCommand, g_ErrorCode - 1000);
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
	case COMM_IDLE:
		CheckEncoder();
		break;
	case COMM_ORIGIN:
		if (breakRelease() == 0)
			break;
		res = CommOrigin(); // Encoder 상시 체크
		UpdateState(res);
		break;
	case COMM_PTP:
		if (breakRelease() == 0)
			break;
		res = CommMove(); // Encoder 체크 X
		UpdateState(res);
		break;
	case COMM_ORIGIN_A:
		if (breakRelease() == 0)
			break;
		res = CommOriginAxis(); // Encoder 상시 체크
		UpdateState(res);
		break;
	case COMM_ERROR_STOP:
		res = CommErrorStop(); // Encoder 체크 X
		UpdateState(res);
		break;

	case COMM_HOME:
		if (breakRelease() == 0)
			break;
		res = CommHome(); // Encoder 상시 체크
		UpdateState(res);
		break;
	case COMM_MMLD:
		if (breakRelease() == 0)
			break;
		res = CommMMLD(); // Encoder 부분 체크
		UpdateState(res);
		break;
	case COMM_MGRI:
		res = CommGripUngrip(); // Encoder 체크 X
		UpdateState(res);
		break;
	case COMM_MUNG:
		res = CommGripUngrip(); // Encoder 체크 X
		UpdateState(res);
		break;
	case COMM_MLOA:
		if (breakRelease() == 0)
			break;
		res = CommMLOA(); // Encoder 부분 체크
		UpdateState(res);
		break;
	case COMM_MASP:
		if (breakRelease() == 0)
			break;
		res = CommMoveXY_With_Offset(); // Encoder 부분 체크
		UpdateState(res);
		break;
	case COMM_MDIS:
		if (breakRelease() == 0)
			break;
		res = CommMoveXY(); // Encoder 부분 체크
		UpdateState(res);
		break;
	case COMM_MSHA:
		if (breakRelease() == 0)
			break;
		res = CommShake(); // Encoder 부분 체크
		UpdateState(res);
		break;
	case COMM_MWAS:
		if (breakRelease() == 0)
			break;
		res = CommWaste(); // Encoder 부분 체크
		UpdateState(res);
		break;
	case COMM_AWAS:
		if (breakRelease() == 0)
			break;
		res = CommAsyncWaste(); // Encoder 부분 체크
		UpdateState(res);
		break;
	case COMM_MWRD:
		if (breakRelease() == 0)
			break;
		res = CommReadyWaste(); // Encoder 부분 체크
		UpdateState(res);
		break;
	case COMM_MWPR:
		if (breakRelease() == 0)
			break;
		res = CommPourWaste(); // Encoder 부분 체크
		UpdateState(res);
		break;
	case COMM_MSEP:
		if (breakRelease() == 0)
			break;
		res = CommSeparate(); // Encoder 부분 체크
		UpdateState(res);
		break;
	case COMM_EQIL:
		if (breakRelease() == 0)
			break;
		res = CommSeparateLongSide(); // Encoder 부분 체크
		UpdateState(res);
		break;
	case COMM_EQIS:
		if (breakRelease() == 0)
			break;
		res = CommSeparateShortSide(); // Encoder 부분 체크
		UpdateState(res);
		break;
	case COMM_MSHK:
		if (breakRelease() == 0)
			break;
		res = CommShakeUsingPD6(); // Encoder 부분 체크
		UpdateState(res);
		break;
	case COMM_SWIRL:
		if (breakRelease() == 0)
			break;
		res = CommSWIRL(); // Encoder 부분 체크
		UpdateState(res);
		break;
	case COMM_MAMV:
		if (breakRelease() == 0)
			break;
		res = CommMAMV(); // Encoder 부분 체크
		UpdateState(res);
		break;
	case COMM_MRGI:
		if (breakRelease() == 0)
			break;
		res = CommMRGI(); // Encoder 부분 체크
		UpdateState(res);
		break;
	case COMM_RASP:
		if (breakRelease() == 0)
			break;
		res = CommRASP(); // Encoder 부분 체크
		UpdateState(res);
		break;
	case COMM_RAMV:
		if (breakRelease() == 0)
			break;
		res = CommRAMV(); // Encoder 부분 체크
		UpdateState(res);
		break;
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

// 명령어가 바뀌었을 때, break release 후 일정 시간 대기
// 0이면 대기 중
// 1이면 대기 종료
// 2025.11.04 step 변수를 전역으로 만들고, 외부에서 실행 전 초기화 한다
int breakRelease()
{
	static int prevMotionCommand = -1;
	static int tickCount = 0;
	static int ret = 0;

	switch (g_BreakReleaseStepNo)
	{
	case 0:
		tickCount = get_var(90);
		ReleaseBreak();
		g_BreakReleaseStepNo = 1;
		break;
	case 1:
		tickCount -= 1;
		if (tickCount <= 0)
		{
			g_BreakReleaseStepNo = 2;
		}
		break;
	case 2:
		g_BreakReleaseStepNo = 0;
		ret = 1;
		break;
	}

	return ret;
}

void BreakControl()
{
	unsigned long break_hold_count = 0;

	break_hold_count = get_var(90);

	if (break_hold_count == 0)
	{
		// break 항상 release
		ReleaseBreak();
	}
	else
	{
		// macro run 중 release
		if (g_MotionCommand == COMM_IDLE)
		{
			// break hold
			HoldBreak();
		}
		else
		{
			// break release - delay time 때문에 각 이동 매크로에서 release 한다
			// ReleaseBreak();
		}
	}
}

int CheckEncoder()
{
	int tolerance_x = get_var(19);
	int tolerance_y = get_var(20);
	signed int counter = 0;
	signed int encoder = 0;
	int error_count_x = 0;
	int error_count_y = 0;
	char msg[128];
	int ret = 1;

	//	if (g_MotionCommand == COMM_IDLE)
	// 모터 에러 상태가 아닐때만
	if (IsError() == 0)
	{
		if (tolerance_x > 0)
		{
			// error_count_x = (int)CounterRead(0) - (int)(EncoderRead(0) * g_EncoderScaleX);
			counter = (signed int)CounterRead(0);
			encoder = (signed int)EncoderRead(0); // * g_EncoderScaleX);
			encoder = encoder * g_EncoderScaleX;

			if (counter != encoder)
			{
				if (counter > encoder)
					error_count_x = counter - encoder;
				else
					error_count_x = encoder - counter;

				if (error_count_x > tolerance_x)
				{
					// 에러
					SetErrorCode(ERR_ENCODER_ERROR_X);
					// g_OriginCompletedAxes[X_AXIS] = 0;
					SetOriginCompletedFlag(X_AXIS, FALSE);
					ret = 0;
				}
			}
		}
		if (tolerance_y > 0)
		{
			counter = (signed int)CounterRead(1);
			encoder = (signed int)EncoderRead(1); // * g_EncoderScaleY);
			encoder = encoder * g_EncoderScaleY;

			if (counter != encoder)
			{
				if (counter > encoder)
					error_count_y = counter - encoder;
				else
					error_count_y = encoder - counter;

				if (error_count_y > tolerance_y)
				{
					// 에러
					SetErrorCode(ERR_ENCODER_ERROR_Y);
					// g_OriginCompletedAxes[Y_AXIS] = 0;
					SetOriginCompletedFlag(Y_AXIS, FALSE);
					ret = 0;
				}
			}
		}
	}

	return ret;
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

	if (use_UserEMO) // currently not used (1.00.016)
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
	case COMM_ORIGIN_A:
	case COMM_HOME:
		return;
	}

	// check limit sensor

	if (g_MotionCommand == COMM_IDLE)
	{
		// IDLE 상태에서, 모터가 이동중일 경우 Limit 센서 확인
		// MMA, MMI, JOG 명령등
		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			if (GetMoveStatus(axis) != MOVE_STS_STOP)
			{
				switch (get_axis_sensor(axis))
				{
				case 0:
					reset_limit_count(axis);
					break;

				case 1: // POS LIMIT
					if (MovVar[axis].m_ucDir == g_MotionParam[axis].m_ucMoveDir)
					{
						// g_OverRun_Command = g_MotionCommand;
						// g_OverRun_LimitSensor = 1;
						// g_OverRun_AxisNo = axis + 1;
						if (inc_pos_limit_count(axis))
						{
							SetErrorCode(ERR_OVER_RUN);
							StopMotors();
						}
					}
					break;

				case 2: // NEG LIMIT
					if (MovVar[axis].m_ucDir != g_MotionParam[axis].m_ucMoveDir)
					{
						// g_OverRun_Command = g_MotionCommand;
						// g_OverRun_LimitSensor = 2;
						// g_OverRun_AxisNo = axis + 1;
						if (inc_neg_limit_count(axis))
						{
							SetErrorCode(ERR_OVER_RUN);
							StopMotors();
						}
					}
					break;

				default:
					// g_OverRun_Command = g_MotionCommand;
					// g_OverRun_LimitSensor = 3;
					// g_OverRun_AxisNo = axis + 1;
					if (inc_pos_limit_count(axis) || inc_neg_limit_count(axis))
					{
						SetErrorCode(ERR_OVER_RUN);
						StopMotors();
					}
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
				reset_limit_count(axis);
				break;

			case 1: // POS LIMIT
				if (g_MoveOffset[axis] > 0)
				{
					// g_OverRun_Command = g_MotionCommand;
					// g_OverRun_LimitSensor = 1;
					// g_OverRun_AxisNo = axis + 1;
					if (inc_pos_limit_count(axis))
					{
						SetErrorCode(ERR_OVER_RUN);
						GoToError();
					}
				}
				break;

			case 2: // NEG LIMIT
				if (g_MoveOffset[axis] < 0)
				{
					// g_OverRun_Command = g_MotionCommand;
					// g_OverRun_LimitSensor = 2;
					// g_OverRun_AxisNo = axis + 1;
					if (inc_neg_limit_count(axis))
					{
						SetErrorCode(ERR_OVER_RUN);
						GoToError();
					}
				}
				break;

			default:
				// g_OverRun_Command = g_MotionCommand;
				// g_OverRun_LimitSensor = 1;
				// g_OverRun_AxisNo = axis + 1;
				if (inc_pos_limit_count(axis) || inc_neg_limit_count(axis))
				{
					SetErrorCode(ERR_OVER_RUN);
					GoToError();
				}
				break;
			}
		}
	}
}

void reset_limit_count(int axis)
{
	g_PosLimitCount[axis] = 0;
	g_NegLimitCount[axis] = 0;
}

int inc_pos_limit_count(int axis)
{
	// 에러 정보가 있으면, 업데이트 하지 않는다
	if (g_OverRun_AxisNo == 0 && g_OverRun_LimitSensor == 0)
	{
		g_PosLimitCount[axis] += 1;
		if (g_PosLimitCount[axis] > get_var(1))
		{
			g_OverRun_Command = g_MotionCommand;
			g_OverRun_AxisNo = axis + 1;
			g_OverRun_LimitSensor = 1;
			g_OverRun_LimitCount = g_PosLimitCount[axis];
			return 1;
		}
		return 0;
	}
	return 1;
}

int inc_neg_limit_count(int axis)
{
	// 에러 정보가 있으면, 업데이트 하지 않는다
	if (g_OverRun_AxisNo == 0 && g_OverRun_LimitSensor == 0)
	{
		g_NegLimitCount[axis] += 1;
		if (g_NegLimitCount[axis] > get_var(1))
		{
			g_OverRun_Command = g_MotionCommand;
			g_OverRun_AxisNo = axis + 1;
			g_OverRun_LimitSensor = 2;
			g_OverRun_LimitCount = g_NegLimitCount[axis];
			return 1;
		}
		return 0;
	}
	return 1;
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

	res = (val >> (8 * ch)) & 0xFF;

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

	// val = ~GetOutput12();
	val = ~GetOutput8();

	res = (val >> (8 * ch)) & 0xFF;

	return res;
}

void SetOutput(char ch, unsigned char data)
{
	// unsigned short cur = GetOutput12();		// active low
	unsigned short cur = GetOutput8();
	unsigned short data2 = 0xFF & (~data); // active high -> active low

	if (ch < 0 || ch > 1)
	{
		return;
	}

	cur = cur & (unsigned short)(~(0xFF << (8 * ch)));
	data2 = cur | (unsigned short)(data2 << (8 * ch));

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

	res = GetInput(ch - 1); // GetInput() : ch = 0-base

	return res;
}

unsigned char GetOutput2(char ch)
{
	unsigned char res = 0;

	if (ch < 1 || ch > MAX_DO)
	{
		return res;
	}

	res = GetOutput(ch - 1); // GetOutput() : ch = 0-base

	return res;
}

void SetOutput2(char ch, unsigned char data)
{
	if (ch < 1 || ch > MAX_DO)
	{
		return;
	}

	SetOutput(ch - 1, data);
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
		// res = (((GetOutput12() >> ch) & 1) == SENS_ON);
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
			// SetOutput12(GetOutput12() & ~value);
			SetOutput8(GetOutput8() & ~value);
		}
		else
		{
			// SetOutput12(GetOutput12() | value);
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
	static int nNegLimitInPos[MAX_AXIS] = {0, 0, 0};
	static int nNegLimitOutPos[MAX_AXIS] = {0, 0, 0};
	static POINT_DATA pd = {
		0,
	};
	int temp = 0;
	int axis = 0;
	int flag = 1;
	static int vacuum_flag = 0;
	static int timerCount = 0;
	int res = 0;
	char move_status = 0;

	if (CheckEncoder() == 0)
	{
		step = 91;
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
	case 0: // Motion ���������� Origin Parameter �� �����Ѵ�.
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

		g_MoveStartErrorCode[Z_AXIS] = MoveStart(Z_AXIS);
		g_MoveStartErrorLine = __LINE__;
		if (NEG_LIMIT(Z_AXIS) != SENS_ON && g_MoveStartErrorCode[Z_AXIS]) // MoveStart(Z_AXIS))
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

	case 4:
		if (HOME_SENSOR(Z_AXIS) == SENS_ON)
		{
			MoveStop(Z_AXIS);
			++step;
		}

		break;

	case 5:
		if (IsStopped())
		{
			++step;
		}

		break;

	case 6:
		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			MovVar[axis].m_uS = g_MotionParam[axis].m_uOrgSLimit;
			MovVar[axis].m_ucDir = g_MotionParam[axis].m_ucOrgDir;
			SetSpeed(axis, SPEED_ORG);
		}

		g_MoveStartErrorCode[X_AXIS] = MoveStart(X_AXIS);
		g_MoveStartErrorLine = __LINE__;
		if (NEG_LIMIT(X_AXIS) != SENS_ON && g_MoveStartErrorCode[X_AXIS]) // MoveStart(X_AXIS))
		{
			step = 91;
			return NORMAL_RUNNING;
		}

		g_MoveStartErrorCode[Y_AXIS] = MoveStart(Y_AXIS);
		g_MoveStartErrorLine = __LINE__;
		if (NEG_LIMIT(Y_AXIS) != SENS_ON && g_MoveStartErrorCode[Y_AXIS]) // MoveStart(Y_AXIS))
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

	case 7:
		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			if (HOME_SENSOR(axis) == SENS_ON) // NegLimit Sensor�� ON �� �� ���� �̵��Ѵ�.
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

	case 8:
		if (IsStopped())
		{
			++step;
		}

		break;

	case 9:
		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			CounterReset(axis);
			EncoderReset(axis);
			MovVar[axis].m_ucDir = (unsigned char)((~g_MotionParam[axis].m_ucOrgDir) & 1); // Positive ��������
			MovVar[axis].m_uAcel = 0;													   // ���� �ӵ��� ����

			if (MoveStart(axis))
			{
				g_MoveStartErrorLine = __LINE__;
				step = 91;
				SetErrorCode(ERR_MOTOR_ERROR);
				return NORMAL_RUNNING;
			}
		}

		DelayMoveStart();

		++step;
		debugf("step=%d", step);
		break;

	case 10:
		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			if (HOME_SENSOR(axis) == SENS_OFF) // NegLimit Sensor OFF �� �� ���� �̵��Ѵ�.
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

	case 11:
		if (IsStopped())
		{
			++step;
			debugf("step=%d", step);
		}

		break;

	case 12:
		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			nNegLimitOutPos[axis] = CounterRead(axis);			   // ���������� Positive �������� �̵��Ͽ� ������ OFF �� ��ġ (��ġ ����)
			MovVar[axis].m_ucDir = g_MotionParam[axis].m_ucOrgDir; // Negative ��������
			MovVar[axis].m_uAcel = 0;							   // ���� �ӵ��� ����

			if (MoveStart(axis))
			{
				g_MoveStartErrorLine = __LINE__;
				step = 91;
				SetErrorCode(ERR_MOTOR_ERROR);
				return NORMAL_RUNNING;
			}
		}

		DelayMoveStart();

		++step;
		debugf("step=%d", step);
		break;

	case 13:
		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			if (HOME_SENSOR(axis) == SENS_ON) // NegLimit Sensor ON �� �� ���� �̵��Ѵ�.
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

	case 14:
		if (IsStopped())
		{
			++step;
			debugf("step=%d", step);
		}

		break;

	case 15:
		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			nNegLimitInPos[axis] = CounterRead(axis); // ���������� Negative �������� �̵��Ͽ� ������ ON �� ��ġ (��ġ ����)

			temp = ((nNegLimitInPos[axis] + nNegLimitOutPos[axis]) / 2) - nNegLimitInPos[axis]; // (������ġ-������ġ)
			if (temp < 0)
			{
				temp = -temp;
			}
			MovVar[axis].m_uS = temp;
			MovVar[axis].m_ucDir = (unsigned char)((~g_MotionParam[axis].m_ucOrgDir) & 1);

			SetSpeed(axis, SPEED_ORG);

			if (MoveStart(axis))
			{
				g_MoveStartErrorLine = __LINE__;
				step = 91;
				SetErrorCode(ERR_MOTOR_ERROR);
				return NORMAL_RUNNING; // ���� : ������ ����
			}
		}

		DelayMoveStart();

		++step;
		debugf("step=%d", step);
		break;

	case 16:
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

	case 17:
		if (IsStopped())
		{
			++step;
			debugf("step=%d", step);
		}

		break;

	case 18:
		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			CounterReset(axis); // �������� �����Ѵ�.
			EncoderReset(axis);
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

		return NORMAL_FINISHED; // ���� : Move Origin ���� �Ϸ�

	default:
		return NORMAL_FINISHED;
	}

	CHECK_USER_STOP();

	return NORMAL_RUNNING;
}

char CommOriginAxis()
{
	static int step = 0;
	static int nNegLimitInPos[MAX_AXIS] = {0, 0, 0};
	static int nNegLimitOutPos[MAX_AXIS] = {0, 0, 0};
	static POINT_DATA pd = {
		0,
	};
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

	if (CheckEncoder() == 0)
	{
		step = 91;
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
		step = 0;
		return ERROR_STOPPED;

	// Start
	case 0:
		SetOriginCompletedFlag(g_OriginAxis, 0);

		MovVar[g_OriginAxis].m_uS = g_MotionParam[g_OriginAxis].m_uOrgSLimit; // 이동 거리를 길게
		MovVar[g_OriginAxis].m_ucDir = g_MotionParam[g_OriginAxis].m_ucOrgDir;
		SetSpeed(g_OriginAxis, SPEED_ORG);

		g_MoveStartErrorCode[g_OriginAxis] = MoveStart(g_OriginAxis);
		g_MoveStartErrorLine = __LINE__;
		if (NEG_LIMIT(g_OriginAxis) != SENS_ON && g_MoveStartErrorCode[g_OriginAxis]) // MoveStart(g_OriginAxis))
		{
			step = 91;
			return NORMAL_RUNNING;
		}

		DelayMoveStart();

		++step;
		debugf("step=%d", step);
		break;

		// Home 센서가 ON 되면 멈춘다
	case 1:
		if (HOME_SENSOR(g_OriginAxis) == SENS_ON) // NegLimit Sensor�� ON �� �� ���� �̵��Ѵ�.
		{
			MoveStop(g_OriginAxis);
			++step;
		}
		break;

	case 2:
		if (IsStopped())
		{
			Delay1ms();
			++step;
		}
		break;

		// 반대 방향으로 이동 시작
	case 3:
		// CounterReset(g_OriginAxis);
		MovVar[g_OriginAxis].m_ucDir = (unsigned char)((~g_MotionParam[g_OriginAxis].m_ucOrgDir) & 1); // Positive ��������
		MovVar[g_OriginAxis].m_uAcel = 0;															   // ���� �ӵ��� ����

		g_MoveStartErrorCode[g_OriginAxis] = MoveStart(g_OriginAxis);
		g_MoveStartErrorLine = __LINE__;
		if (g_MoveStartErrorCode[g_OriginAxis])
		{
			step = 91;
			SetErrorCode(ERR_MOTOR_ERROR);
			return NORMAL_RUNNING;
		}

		DelayMoveStart();

		++step;
		debugf("step=%d", step);
		break;

		//  홈 센서가 OFF 되면 멈춘다
	case 4:
		if (HOME_SENSOR(g_OriginAxis) == SENS_OFF) // NegLimit Sensor OFF �� �� ���� �̵��Ѵ�.
		{
			MoveStop(g_OriginAxis);
			++step;
		}
		break;

	case 5:
		if (IsStopped())
		{
			Delay1ms();
			++step;
		}
		break;

		// Offset(0.2) 만큼 이동
	case 6:
	{
		POINT_DATA pd = get_point_data(12);
		double org_offset = 0.2;

		CounterReset(g_OriginAxis);
		EncoderReset(g_OriginAxis);

		switch (g_OriginAxis)
		{
		case X_AXIS:
			org_offset = pd.x;
			break;
		case Y_AXIS:
			org_offset = pd.y;
			break;
		case Z_AXIS:
			org_offset = pd.z;
			break;
		}
		g_MoveOffset[g_OriginAxis] = (org_offset - get_motor_pos(g_OriginAxis)) / g_MotionParam[g_OriginAxis].m_fScaleFactor; // g_MotionParam[g_OriginAxis].fScaleFactor;SetMoveOffset(g_OriginAxis, g_MoveOffset[g_OriginAxis]);
		SetMoveOffset(g_OriginAxis, g_MoveOffset[g_OriginAxis]);

		SetSpeed(g_OriginAxis, SPEED_ORG);
		g_MoveStartErrorCode[g_OriginAxis] = MoveStart(g_OriginAxis);
		g_MoveStartErrorLine = __LINE__;
		if (g_MoveStartErrorCode[g_OriginAxis])
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
	}
	break;
	case 7:
		move_status = GetMoveStatus(g_OriginAxis);
		switch (move_status)
		{
		case MOVE_STS_INPOS:
			MoveStop(g_OriginAxis);
			step++;
			break;
		case MOVE_STS_STOP:
			step++;
			break;
		}
		break;
	case 8:
		if (IsStopped())
		{
			Delay1ms();
			step++;
		}
		break;

	case 9:
		CounterReset(g_OriginAxis); // �������� �����Ѵ�.
		EncoderReset(g_OriginAxis);
		SetSpeed(g_OriginAxis, SPEED_NORMAL);
		SetOriginCompletedFlag(g_OriginAxis, 1);
		timerCount = 20;
		++step;

	case 10:
		if (--timerCount > 0)
		{
			Delay1ms();
			break;
		}
		++step;
		break;

	case 11:
		step = 0;
		return NORMAL_FINISHED; // ���� : Move Origin ���� �Ϸ�

	default:
		return NORMAL_FINISHED;
	}

	CHECK_USER_STOP();

	return NORMAL_RUNNING;
}

char CommMove()
{
	char axis = 0;
	static int state[MAX_AXIS] = {0, 0, 0};
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

				g_MoveStartErrorCode[axis] = MoveStart(axis);
				g_MoveStartErrorLine = __LINE__;
				if (g_MoveStartErrorCode[axis])
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

	case 1:
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

	case 2:
		if (IsStopped())
		{
			++step;
			debugf("step=%d", step);
		}

		break;

	case 3:
		debugf("SetHoldTorque");
		for (axis = 0; axis < MAX_AXIS; ++axis)
		{
			SetHoldTorque(axis);
			g_MoveOffset[axis] = 0;
		}

		step = 0;

		return NORMAL_FINISHED; // ���� : Move Origin ���� �Ϸ�

	default:
		step = 0;
		return NORMAL_FINISHED; // ���� : Move Origin ���� ��?
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

	for (axis = 0; axis < MAX_AXIS; axis++)
	{
		debugf("SetCounterDirection(%d, %d)", axis, (unsigned char)((~g_MotionParam[axis].m_ucOrgDir) & 1));

		SetCounterDirection(axis, (unsigned char)((~g_MotionParam[axis].m_ucOrgDir) & 1));

		MovVar[axis].m_ucHoldTorq = g_MotionParam[axis].m_ucHoldTorque; //(axis == Z_AXIS) ? HOLD_TORQUE : HOLD_TORQUE_3A;

		// MovVar[axis].m_ucHoldTorq = 20;
		SetHoldTorque(axis);

		DriverEnable(axis);

		DelayMS(500);

		if (DriverErrorCheck(axis) != 0)
		{
			debugf("Axis(%d) DriverError!!! (%d)", axis, DriverErrorCheck(axis));
		}

		CounterReset(axis);
		// Encoder 초기화
		SetEncoderDir(axis, 0);
		EncoderReset(axis);
		EncoderWrite(axis, 0);
	}
}

void JogStart(char axis, char dir)
{
	static char str[128];

	sprintf(str, "JogStart(%d, %d)", axis, dir);
	debug(str);

	if (dir == 0) // inverse
	{
		dir = InverseDir(g_MotionParam[axis].m_ucMoveDir);
	}
	else
	{
		dir = g_MotionParam[axis].m_ucMoveDir;
	}

	MovVar[axis].m_uS = g_MotionParam[axis].m_uOrgSLimit;
	MovVar[axis].m_ucDir = dir;
	SetSpeed(axis, SPEED_JOG);
	MovVar[axis].m_ucMoveTorq = g_MotionParam[axis].m_ucMoveTorque; //(axis == Z_AXIS) ? MOVE_TORQUE : MOVE_TORQUE_3A;

	g_MoveStartErrorLine = __LINE__;
	if (MoveStart(axis))
	{
		SetHoldTorque(axis); // Jog �Լ��� �����ϹǷ� Holding Current�� �����.
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

	MovVar[axis].m_uS = offset;
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
		// MovVar[axis].m_uVmin = g_MotionParam[axis].m_uOrgVmax;		// 속도가 느려서 max 값을 사용해도 됨
		MovVar[axis].m_uVmin = g_MotionParam[axis].m_uOrgVmin; // min 값을 사용하도록 하고 min 값을 max의 50%
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

	case SPEED_SEPARATE:
		switch (axis)
		{
		case X_AXIS:
			MovVar[axis].m_uVmax = get_var(11);
			break;
		default:
			MovVar[axis].m_uVmax = g_MotionParam[axis].m_uNorVmax;
			break;
		}
		MovVar[axis].m_uVmin = g_MotionParam[axis].m_uNorVmin;
		MovVar[axis].m_uAcel = g_MotionParam[axis].m_uNorAcel;
		break;
	}

	MovVar[axis].m_ucMoveTorq = g_MotionParam[axis].m_ucMoveTorque; //(axis == Z_AXIS) ? MOVE_TORQUE : MOVE_TORQUE_3A;
}

void SetSpeedRatio(char axis, int type, int ratio)
{
	float fRatio = (float)(ratio / 100.0);
	float fAcelRatio = 1.0;

	if (axis < 0 || axis >= MAX_AXIS)
	{
		return;
	}

	if (fRatio < 0.0)
	{
		fRatio = 0.01;
	}

	fAcelRatio = fRatio;
	if (fAcelRatio < 0.1)
	{
		fAcelRatio = 0.1;
	}

	switch (type)
	{
	case SPEED_ORG:
		MovVar[axis].m_uVmax = (unsigned int)(g_MotionParam[axis].m_uOrgVmax); // * fRatio);
		MovVar[axis].m_uVmin = (unsigned int)(g_MotionParam[axis].m_uOrgVmin); // * fRatio);
		MovVar[axis].m_uAcel = (unsigned int)(g_MotionParam[axis].m_uOrgAcel); // * fRatio);
		break;

	case SPEED_JOG:
		MovVar[axis].m_uVmax = (unsigned int)(g_MotionParam[axis].m_uJogVmax); // * fRatio);
		MovVar[axis].m_uVmin = (unsigned int)(g_MotionParam[axis].m_uJogVmin); // * fRatio);
		MovVar[axis].m_uAcel = (unsigned int)(g_MotionParam[axis].m_uJogAcel); // * fRatio);
		break;

	case SPEED_NORMAL:
		MovVar[axis].m_uVmax = (unsigned int)(g_MotionParam[axis].m_uNorVmax * fRatio);
		MovVar[axis].m_uVmin = (unsigned int)(g_MotionParam[axis].m_uNorVmin * fRatio);
		MovVar[axis].m_uAcel = (unsigned int)(g_MotionParam[axis].m_uNorAcel * fAcelRatio);
		break;
	}

	MovVar[axis].m_ucMoveTorq = g_MotionParam[axis].m_ucMoveTorque; //(axis == Z_AXIS) ? MOVE_TORQUE : MOVE_TORQUE_3A;
}

/////////////////////////////////////////////////////////////
// Home 동작 기능
// - Home 센서를 감지 하고 있으면, + 방향으로 빠져 나온다
// - - 방향으로 이동해서 Home 센서를 찾는다
// - + 방향으로 이동해서 Home 센서를 빠져 나온다
// - 현재 위치를 0으로 셋팅
// - Offset 만큼 이동 한다
// - 현재 위치를 0으로 셋팅
// Gripper(3) -> Rotation(2) -> Tile(1) 순서로 Origin 한다
// v1.2.5 에서 수정
//	Rotation(2) -> Title(1) -> Load위치로 이동 -> Gripper(3)
// v1.2.6 에서 수정
//  Flask 센서 확인 (On이면, Grip 센서 On 될때까지 Grip)
char CommHome()
{
	static int origin_axis[3] = {Y_AXIS, X_AXIS, Z_AXIS}; // 원점복귀 순서
	static int origin_index = 0;						  // 원점복귀 동작 축 인덱스

	static int nNegLimitOutPos[MAX_AXIS] = {0, 0, 0};
	static int nNegLimitInPos[MAX_AXIS] = {0, 0, 0};
	// static int org_axis = 0;	//  원점복귀 하는 축 번호 Z->Y->X 순서
	static int step = 0;
	static int delay_count = 0;

	int axis = 0;
	int temp = 0;
	char move_status = 0;
	char move_start = 0;
	double org_offset = 0;
	int home_x_offset = 0;
	int home_y_offset = 0;
	POINT_DATA pd_load;
	POINT_DATA pd_org_offset;

	char sz[64];

	if (CheckEncoder() == 0)
	{
		step = 91;
	}

	switch (step)
	{
		// Error Handling
	case 91:
		StopMotors();
		step++;
		break;
	case 92:
		if (IsStopped())
		{
			step++;
		}
		break;
	case 93:
		HoldMotors();
		step = 0;
		return ERROR_STOPPED;

		// Start
	case 0:
		// g_OriginAxis = Z_AXIS;
		origin_index = 0;
		for (axis = 0; axis < MAX_AXIS; axis++)
		{
			SetOriginCompletedFlag(axis, 0);
		}
		// step = 1;
		step = 110; // v1.2.6 Flask가 있으면 Grip 한다 => Celltrio 확인 후 기능 테스트 예정
		break;

		// + 방향으로, VAR12의 위치 만큼 X축 회피
	case 1:
		home_x_offset = get_var(12);
		if (home_x_offset <= 0)
		{
			step = 10;
			break;
		}
		if (POS_LIMIT(X_AXIS) == SENS_ON)
		{
			step = 5;
			break;
		} // +limit 감지시 5번 step

		g_MoveOffset[X_AXIS] = (int)(home_x_offset / g_MotionParam[X_AXIS].m_fScaleFactor);
		SetMoveOffset(X_AXIS, g_MoveOffset[X_AXIS]);
		SetSpeed(X_AXIS, SPEED_ORG);
		g_MoveStartErrorCode[X_AXIS] = MoveStart(X_AXIS);
		g_MoveStartErrorLine = __LINE__;
		if (g_MoveStartErrorCode[X_AXIS])
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step = 2;
		break;
	case 2:
		if (POS_LIMIT(X_AXIS) == SENS_ON)
		{
			MoveStop(X_AXIS);
			step = 5;
			break; // +limit 감지시 5번 step
		}
		move_status = GetMoveStatus(X_AXIS);
		switch (move_status)
		{
		case MOVE_STS_INPOS:
			MoveStop(X_AXIS);
			step = 3;
			break;
		case MOVE_STS_STOP:
			step = 3;
			break;
		}
		break;
	case 3:
		if (IsStopped())
		{
			Delay1ms();
			step = 10;
		}
		break;

		// POS_LIMIT 이 감지 되었기 때문에, -방향으로 회피
	case 5:
		if (!IsStopped())
		{
			break;
		}
		Delay1ms();
		home_x_offset = get_var(12) * -1;
		g_MoveOffset[X_AXIS] = (int)(home_x_offset / g_MotionParam[X_AXIS].m_fScaleFactor);
		SetMoveOffset(X_AXIS, g_MoveOffset[X_AXIS]);
		SetSpeed(X_AXIS, SPEED_ORG);
		g_MoveStartErrorCode[X_AXIS] = MoveStart(X_AXIS);
		g_MoveStartErrorLine = __LINE__;
		if (g_MoveStartErrorCode[X_AXIS])
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;
	case 6:
		move_status = GetMoveStatus(X_AXIS);
		switch (move_status)
		{
		case MOVE_STS_INPOS:
			MoveStop(X_AXIS);
			step++;
			break;
		case MOVE_STS_STOP:
			step++;
			break;
		}
		break;
	case 7:
		if (IsStopped())
		{
			Delay1ms();
			step = 10;
		}
		break;

		///////////////////////////////////////////////////////
		// g_OriginAxis 축의 Origin 시작
		// - 방향으로 이동
	case 10: // 3:
		MovVar[origin_axis[origin_index]].m_uS = g_MotionParam[origin_axis[origin_index]].m_uOrgSLimit;
		MovVar[origin_axis[origin_index]].m_ucDir = g_MotionParam[origin_axis[origin_index]].m_ucOrgDir;
		SetSpeed(origin_axis[origin_index], SPEED_ORG); // torque 설정

		g_MoveStartErrorCode[origin_axis[origin_index]] = MoveStart(origin_axis[origin_index]);
		g_MoveStartErrorLine = __LINE__;
		if (NEG_LIMIT(origin_axis[origin_index]) != SENS_ON && g_MoveStartErrorCode[origin_axis[origin_index]]) // MoveStart(g_OriginAxis))
		{
			step = 91;
			return NORMAL_RUNNING;
		}
		else
		{
			step++;
		}

		DelayMoveStart();
		break;

		// Home 센서가 ON 되면 멈춘다
	case 11: // 4:
		if (HOME_SENSOR(origin_axis[origin_index]) == SENS_ON)
		{
			MoveStop(origin_axis[origin_index]);
			step++;
		}
		break;

	case 12: // 5:
		if (IsStopped())
		{
			Delay1ms();
			step++;
		}
		break;

		// + 방향으로 이동 (반대 방향으로 이동 시작)
	case 13: // 6:
		MovVar[origin_axis[origin_index]].m_ucDir = (unsigned char)((~g_MotionParam[origin_axis[origin_index]].m_ucOrgDir) & 1);
		MovVar[origin_axis[origin_index]].m_uAcel = 0;

		// 2021.11.22
		g_MoveStartErrorCode[origin_axis[origin_index]] = MoveStart(origin_axis[origin_index]);
		g_MoveStartErrorLine = __LINE__;
		if (g_MoveStartErrorCode[origin_axis[origin_index]])
		{
			step = 91;
			SetErrorCode(ERR_MOTOR_ERROR);
			return NORMAL_RUNNING;
		}

		DelayMoveStart();
		step++;
		break;

		// HOME 센서가 OFF 되면 멈춘다
	case 14: // 7:
		if (HOME_SENSOR(origin_axis[origin_index]) == SENS_OFF)
		{
			MoveStop(origin_axis[origin_index]);
			step++;
		}
		break;

	case 15: // 8:
		if (IsStopped())
		{
			Delay1ms();
			step++;
		}
		break;

	case 16: // 9:
		CounterReset(origin_axis[origin_index]);
		EncoderReset(origin_axis[origin_index]);
		step++;
		break;

		// Offset(0.2) 만큼 이동
	case 17: // 10:
		pd_org_offset = get_point_data(12);
		switch (origin_axis[origin_index])
		{
		case X_AXIS:
			org_offset = pd_org_offset.x;
			break;
		case Y_AXIS:
			org_offset = pd_org_offset.y;
			break;
		case Z_AXIS:
			org_offset = pd_org_offset.z;
			break;
		default:
			org_offset = 0.2;
			break;
		}
		g_MoveOffset[origin_axis[origin_index]] = (org_offset - get_motor_pos(origin_axis[origin_index])) / g_MotionParam[origin_axis[origin_index]].m_fScaleFactor;
		SetMoveOffset(origin_axis[origin_index], g_MoveOffset[origin_axis[origin_index]]);
		// 2021.11.22
		SetSpeed(origin_axis[origin_index], SPEED_ORG);
		g_MoveStartErrorCode[origin_axis[origin_index]] = MoveStart(origin_axis[origin_index]);
		g_MoveStartErrorLine = __LINE__;
		if (g_MoveStartErrorCode[origin_axis[origin_index]])
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;
	case 18: // 11:
		move_status = GetMoveStatus(origin_axis[origin_index]);
		switch (move_status)
		{
		case MOVE_STS_INPOS:
			MoveStop(origin_axis[origin_index]);
			step++;
			break;
		case MOVE_STS_STOP:
			step++;
			break;
		}
		break;
	case 19: // 12:
		if (IsStopped())
		{
			Delay1ms();
			step++;
		}
		break;
		//

	case 20: // 13:
		CounterReset(origin_axis[origin_index]);
		EncoderReset(origin_axis[origin_index]);
		SetSpeed(origin_axis[origin_index], SPEED_NORMAL);
		SetOriginCompletedFlag(origin_axis[origin_index], 1);
		delay_count = 20;
		step++;
		break;

	case 21: // 14:
		if (--delay_count > 0)
		{
			Delay1ms();
			break;
		}
		if (origin_axis[origin_index] == X_AXIS)
		{
			// X축 원점복귀 했을 경우, Load 위치로 이동한다
			step = 100;
		}
		else
		{
			step = 30;
		}
		break;

	case 30: // 15:
		// if (--g_OriginAxis >= X_AXIS) { step = 10; }
		// else { step++; }
		origin_index += 1;
		if (origin_index >= MAX_AXIS)
		{
			step++;
		}
		else
		{
			step = 10;
		}
		break;

	case 31: // 16:
		HoldMotors();
		step = 0;
		return NORMAL_FINISHED;

	default:
		HoldMotors();
		step = 0;
		return NORMAL_FINISHED;

		///////////////////////////////////////////////////////
		// X, Y축을 Load 위치(PD3)로 이동한다 (상대 이동). 완료 후 step=30
		// v2.0.0 PD WaitPios(PD16)로 이동 한다
	case 100:
		pd_load = get_point_data(15);
		if (fabs(pd_load.x) < 0.01)
		{
			step = 102;
			break;
		}

		g_MoveOffset[X_AXIS] = (int)(pd_load.x / g_MotionParam[X_AXIS].m_fScaleFactor);
		SetMoveOffset(X_AXIS, g_MoveOffset[X_AXIS]);
		// SetSpeed(X_AXIS, SPEED_ORG);
		SetSpeedRatio(X_AXIS, SPEED_NORMAL, 80);
		g_MoveStartErrorCode[X_AXIS] = MoveStart(X_AXIS);
		g_MoveStartErrorLine = __LINE__;
		if (g_MoveStartErrorCode[X_AXIS])
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step = 101;
		break;
	case 101:
		if (move_done(0x01))
		{
			step = 102;
			delay_count = 20;
		}
		break;

	case 102:
		if (--delay_count > 0)
		{
			Delay1ms();
			break;
		}
		pd_load = get_point_data(15);
		if (fabs(pd_load.y) < 0.01)
		{
			step = 104;
			break;
		}

		g_MoveOffset[Y_AXIS] = (int)(pd_load.y / g_MotionParam[Y_AXIS].m_fScaleFactor);
		SetMoveOffset(Y_AXIS, g_MoveOffset[Y_AXIS]);
		// SetSpeed(Y_AXIS, SPEED_ORG);
		SetSpeedRatio(Y_AXIS, SPEED_NORMAL, 80);
		g_MoveStartErrorCode[Y_AXIS] = MoveStart(Y_AXIS);
		g_MoveStartErrorLine = __LINE__;
		if (g_MoveStartErrorCode[Y_AXIS])
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step = 103;
		break;
	case 103:
		if (move_done(0x02))
		{
			step = 104;
		}
		break;

	case 104:
		step = 30;
		break;

		///////////////////////////////////////////////////////
		// v1.2.6 Flask가 있으면 Grip 한다. Demo mode와 관련 없이 동작 한다
	case 110:
		if (!IsExistFlask())
		{
			step = 120;
		}
		else
		{
			step = 111;
		}
		break;
	case 111:
		if (IsGrip())
		{
			step = 120;
		}
		else
		{
			step = 112;
		}
	case 112:
		// Z축을 Grip 될때까지 이동한다. 15mm 이동 후 감지 되면 멈춘다
		// 감지되지 않으면???
		g_MoveOffset[Z_AXIS] = (int)(15 / g_MotionParam[Z_AXIS].m_fScaleFactor);
		SetMoveOffset(Z_AXIS, g_MoveOffset[Z_AXIS]);
		SetSpeedRatio(Z_AXIS, SPEED_NORMAL, 60);
		g_MoveStartErrorCode[Z_AXIS] = MoveStart(Z_AXIS);
		g_MoveStartErrorLine = __LINE__;
		if (g_MoveStartErrorCode[Z_AXIS])
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step = 113;
		break;
	case 113:
		if (IsGrip())
		{
			MoveStop(Z_AXIS);
			step = 114;
			break;
		}
		if (move_done(0x04))
		{
			step = 116;
			break;
		}
		break;
	case 114:
		if (IsStopped())
		{
			Delay1ms();
			step = 116;
		}
		break;
	case 116:
		step = 120;
		break;
	case 120:
		step = 1;
		break;
	}

	CHECK_USER_STOP();

	return NORMAL_RUNNING;
}

// LOAD 기준으로 이동한다
// - g_MoveRatio, g_fMoveXPos, g_fMoveYPos
// - Load 기준으로 이동하기 전에 스크로크에 들어오는지 확인
char CommMMLD()
{
	static char step = 0;
	float pos[3];
	int move_start = 0;

	switch (step)
	{
		// Error handling
	case 91:
		StopMotors();
		step++;
		break;
	case 92:
		if (IsStopped())
		{
			step++;
		}
		break;
	case 93:
		HoldMotors();
		step = 0;
		return ERROR_STOPPED;

		// y축 위치를 확인 한다
	case 0:
		if (IsYAxisPositionMovable())
		{
			step = 10;
		}
		else
		{
			step = 1;
		}
		break;

		// y축을 PD15번 위치로 이동 한다
	case 1:
		move_start = move_pd(15, 0x02);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;
	case 2:
		if (move_done(0x02) && IsStopped())
		{
			step = 3;
		}
		break;
	case 3:
		step = 10;
		if (CheckEncoder() == 0)
		{
			step = 91;
			return NORMAL_RUNNING;
		}
		break;

	case 10:
		pos[0] = g_fMoveXPos;
		pos[1] = g_fMoveYPos;
		pos[2] = 0.0;
		move_start = move_abs(0x03, pos, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 11:
		if (move_done(0x03))
		{
			step++;
		}
		break;

	case 12:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 13:
		// SetHoldTorque(Z_AXIS);
		HoldMotors();
		step++;
		break;

	default:
		step = 0;
		return NORMAL_FINISHED;
	}

	CHECK_USER_STOP();

	return 0;
}

// g_MoveOffset[] 변수에 위치를 저장해 놓고, 그 위치로 이동한다
char CommGripUngrip()
{
	static char step = 0;
	int varGripOption = 6;
	int axis = 0;
	char flag = 1;
	char move_start = 0;
	char move_status = 0;

	switch (step)
	{
		// Error handling
	case 91:
		StopMotors();
		step++;
		break;
	case 92:
		if (IsStopped())
		{
			step++;
		}
		break;
	case 93:
		HoldMotors();
		step = 0;
		return ERROR_STOPPED;

		// Start
		// move_offset 값이 0이면 MoveStart() 함수에서 3번 에러 발생
		// offset = target_pos - current_pos
		// point data 2번. Z축 위치 사용
	case 0:
		if (g_MovePointDataNo != 0)
		{
			move_start = move_pd(g_MovePointDataNo, 0x04);
			g_MoveStartErrorLine = __LINE__;
			if (move_start)
			{
				SetErrorCode(ERR_MOTOR_ERROR);
				step = 91;
				return NORMAL_RUNNING;
			}
			DelayMoveStart();
			step++;
		}
		else
		{
			step = 4;
		}
		break;

	case 1:
		// 이 옵션에서는 91번 변수를 사용하지 않는다 (Grip 센서 사용 유무)
		// 옵션을 사용하지 않아도 센서가 고장났을 경우 티칭 위치까지만 이동하며,
		// 이 후 Grip 센서가 들어오지 않기 때문임
		if (g_MovePointDataNo == 1 && get_var(varGripOption) != 0)
		{
			// Grip일 경우, Grip 센서가 들어오면 바로 멈춘다
			if (IsGrip())
			{
				MoveStop(Z_AXIS);
				step = 2;
				break;
			}
		}
		if (move_done(0x04))
		{
			step++;
		}
		break;

	case 2:
		if (IsStopped())
		{
			step++;
		}
		break;

	case 3:
		if (g_MovePointDataNo == 1)
		{
			// send("case 3\r\n");
			// Grip일 때, Grip 센서가 감지되는지 확인
			if ((get_var(91) == 0) && !IsGrip())
			{
				// send("case 3 : error routine\r\n");
				//  Error
				SetErrorCode(ERR_GRIP_ERROR);
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		step++;
		break;

	case 4:
		SetHoldTorque(Z_AXIS);
		g_MovePointDataNo = 0;
		g_MoveOffset[Z_AXIS] = 0;
		step++;
		break;

	default:
		step = 0;
		return NORMAL_FINISHED;
	}

	CHECK_USER_STOP();

	return 0;
}

//
char CommMoveXY()
{
	static char step = 0;
	int axis = 0;
	char flag = 1;
	char move_start = 0;
	char move_status = 0;
	POINT_DATA pd;

	switch (step)
	{
		// Error handling
	case 91:
		StopMotors();
		step++;
		break;
	case 92:
		if (IsStopped())
		{
			step++;
		}
		break;
	case 93:
		HoldMotors();
		step = 0;
		return ERROR_STOPPED;

		// y축 위치를 확인 한다
	case 0:
		if (g_MovePointDataNo == 0)
		{
			step = 10;
			break;
		}
		if (IsYAxisPositionMovable())
		{
			step = 10;
		}
		else
		{
			step = 1;
		}
		break;
	case 1:
		move_start = move_pd(15, 0x02);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;
	case 2:
		if (move_done(0x02) && IsStopped())
		{
			step = 10;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 10:
		if (g_MovePointDataNo != 0)
		{
			// move_start = move_pd(g_MovePointDataNo, 0x03);
			move_start = move_pd_with_speed_ratio(g_MovePointDataNo, 0x03, SPEED_NORMAL, g_MoveRatio);
			g_MoveStartErrorLine = __LINE__;
			if (move_start)
			{
				SetErrorCode(ERR_MOTOR_ERROR);
				step = 91;
				return NORMAL_RUNNING;
			}
			DelayMoveStart();
			step++;
		}
		else
		{
			step = 13;
		}
		break;

	case 11:
		if (move_done(0x03))
		{
			step++;
		}
		break;

	case 12:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 13:
		for (axis = 0; axis < 2; axis++)
		{
			SetHoldTorque(axis);
			g_MovePointDataNo = 0;
			g_MoveOffset[axis] = 0;
		}
		step = 0;
		return NORMAL_FINISHED;

	default:
		step = 0;
		return NORMAL_FINISHED;
	}

	CHECK_USER_STOP();

	return 0;
}

char CommMoveXY_With_Offset()
{
	static char step = 0;
	int axis = 0;
	char flag = 1;
	char move_start = 0;
	char move_status = 0;
	POINT_DATA pd;

	switch (step)
	{
		// Error handling
	case 91:
		StopMotors();
		step++;
		break;
	case 92:
		if (IsStopped())
		{
			step++;
		}
		break;
	case 93:
		HoldMotors();
		step = 0;
		return ERROR_STOPPED;

		// y축 위치를 확인 한다
	case 0:
		if (g_MovePointDataNo == 0)
		{
			step = 10;
			break;
		}
		if (IsYAxisPositionMovable())
		{
			step = 10;
		}
		else
		{
			step = 1;
		}
		break;
		// y축을 PD15번 위치로 이동 한다
	case 1:
		move_start = move_pd(15, 0x02);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;
	case 2:
		if (move_done(0x02) && IsStopped())
		{
			step = 10;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 10:
		if (g_MovePointDataNo != 0)
		{
			move_start = move_pd_with_speed_ratio_xy_offset(
				g_MovePointDataNo,
				0x03,
				SPEED_NORMAL,
				g_MoveRatio,
				g_fMASPOffset[0],
				g_fMASPOffset[1]);
			g_MoveStartErrorLine = __LINE__;
			if (move_start)
			{
				SetErrorCode(ERR_MOTOR_ERROR);
				step = 91;
				return NORMAL_RUNNING;
			}
			DelayMoveStart();
			step++;
		}
		else
		{
			step = 13;
		}
		break;

	case 11:
		if (move_done(0x03))
		{
			step++;
		}
		break;

	case 12:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 13:
		for (axis = 0; axis < 2; axis++)
		{
			SetHoldTorque(axis);
			g_MovePointDataNo = 0;
			g_MoveOffset[axis] = 0;
		}
		step = 0;
		return NORMAL_FINISHED;

	default:
		step = 0;
		return NORMAL_FINISHED;
	}

	CHECK_USER_STOP();
	return 0;
}

char CommMLOA()
{
	static char step = 0;
	int axis = 0;
	char flag = 1;
	char move_start = 0;
	char move_status = 0;
	POINT_DATA pd;

	switch (step)
	{
		// Error handling
	case 91:
		StopMotors();
		step++;
		break;
	case 92:
		if (IsStopped())
		{
			step++;
		}
		break;
	case 93:
		HoldMotors();
		step = 0;
		return ERROR_STOPPED;

	case 0:
		// move_start = move_pd(g_MovePointDataNo, 0x03);
		move_start = move_pd_with_speed_ratio(3, 0x01, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 1:
		if (move_done(0x01))
		{
			step++;
		}
		break;

	case 2:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 3:
		move_start = move_pd_with_speed_ratio(3, 0x02, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;
	case 4:
		if (move_done(0x02))
		{
			step++;
		}
		break;
	case 5:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 6:
		for (axis = 0; axis < 2; axis++)
		{
			SetHoldTorque(axis);
			g_MovePointDataNo = 0;
			g_MoveOffset[axis] = 0;
		}
		step = 0;
		return NORMAL_FINISHED;

	default:
		step = 0;
		return NORMAL_FINISHED;
	}

	CHECK_USER_STOP();

	return 0;
}

// pd6, pd7번을 이동 -> pd3번으로 이동
// var6. 이동 회수
// var7. 이동 delay (ms)
// version. 1.01.000 수정
// - load 위치로 이동
// - count 회수 만큼 X축을 +/- angle 만큼 이동
char CommShake()
{
	const int POINT_SHAKE_1 = 6;
	const int POINT_SHAKE_2 = 7;
	const int POINT_NEW_LOAD = 15;
	const int VAR_NUM_MOVE = 6;
	const int VAR_DELAY_MOVE = 7;

	static char step = 0;
	static float angle_1[3] = {0.0, 0.0, 0.0};
	static float angle_2[3] = {0.0, 0.0, 0.0};
	static int delay_count = 0;
	static int shake_count = 0;

	int move_start = 0;
	POINT_DATA pd;

	switch (step)
	{
		// Error handling
	case 91:
		StopMotors();
		step++;
		break;
	case 92:
		if (IsStopped())
		{
			step++;
		}
		break;
	case 93:
		HoldMotors();
		step = 0;
		return ERROR_STOPPED;

		// y축 위치를 확인 한다
	case 0:
		if (IsYAxisPositionMovable())
		{
			step = 10;
		}
		else
		{
			step = 1;
		}
		break;
		// y축을 PD15번 위치로 이동 한다
	case 1:
		move_start = move_pd(15, 0x02);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;
	case 2:
		if (move_done(0x02) && IsStopped())
		{
			step = 10;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		//
	case 10:
		delay_count = 0;
		shake_count = 0; // g_ShakeCount; //g_get_var(VAR_NUM_MOVE);
		pd = get_point_data(POINT_NEW_LOAD);
		angle_1[0] = pd.x + g_ShakeAngle;
		angle_2[0] = pd.x - g_ShakeAngle;
		step++;
		break;

		// pd15으로 이동 (load point)
	case 11:
		move_start = move_pd_with_speed_ratio(POINT_NEW_LOAD, 0x03, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 12:
		if (move_done(0x03))
		{
			step++;
		}
		break;

	case 13:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// shake. X축을 +/-방향으로 이동
	case 14:
		if (shake_count >= g_ShakeCount)
		{
			step = 20;
			break;
		}

		if (shake_count % 2 == 0)
		{
			move_start = move_abs(0x01, angle_1, SPEED_NORMAL, g_MoveRatio);
		}
		else
		{
			move_start = move_abs(0x01, angle_2, SPEED_NORMAL, g_MoveRatio);
		}
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 15:
		if (move_done(0x01))
		{
			delay_count = g_ShakeTiltDelay; // get_var(VAR_DELAY_MOVE);
			step++;
		}
		break;

	case 16:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 17:
		if (--delay_count > 0)
		{
			Delay1ms();
		}
		else
		{
			shake_count += 1;
			step = 14;
		}
		break;

		// new load 위치(15)로 이동
	case 20:
		// move_start = move_pd(POINT_LOAD, 0x03);
		move_start = move_pd_with_speed_ratio(POINT_NEW_LOAD, 0x03, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 21:
		if (move_done(0x03))
		{
			step++;
		}
		break;

	case 22:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}

	case 23:
		HoldMotors();
		g_MovePointDataNo = 0;
		g_MoveOffset[X_AXIS] = g_MoveOffset[Y_AXIS] = g_MoveOffset[Z_AXIS] = 0;
		step = 0;
		return NORMAL_FINISHED;

	default:
		step = 0;
		return NORMAL_FINISHED;
	}

	CHECK_USER_STOP();

	return 0;
}

// pd6 으로 이동
// var6. 이동 회수
// var7. 이동 delay (ms)
// version. 1.01.012 수정
// - pd6 위치로 이동
// - count 회수 만큼 X축을 +/- angle 만큼 이동
// - load 위치로 이동
char CommShakeUsingPD6()
{
	const int POINT_READY = 6;
	const int POINT_NEW_LOAD = 15;
	const int VAR_NUM_MOVE = 6;
	const int VAR_DELAY_MOVE = 7;

	static char step = 0;
	static float angle_1[3] = {0.0, 0.0, 0.0};
	static float angle_2[3] = {0.0, 0.0, 0.0};
	static int delay_count = 0;
	static int shake_count = 0;

	int move_start = 0;
	POINT_DATA pd;

	switch (step)
	{
		// Error handling
	case 91:
		StopMotors();
		step++;
		break;
	case 92:
		if (IsStopped())
		{
			step++;
		}
		break;
	case 93:
		HoldMotors();
		step = 0;
		return ERROR_STOPPED;

	// y축 위치를 확인 한다
	case 0:
		if (IsYAxisPositionMovable())
		{
			step = 10;
		}
		else
		{
			step = 1;
		}
		break;
		// y축을 PD15번 위치로 이동 한다
	case 1:
		move_start = move_pd(15, 0x02);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;
	case 2:
		if (move_done(0x02) && IsStopped())
		{
			step = 10;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		//
	case 10:
		delay_count = 0;
		shake_count = 0; // g_ShakeCount; //g_get_var(VAR_NUM_MOVE);
		pd = get_point_data(POINT_READY);
		angle_1[0] = pd.x + g_ShakeAngle;
		angle_2[0] = pd.x - g_ShakeAngle;
		step = 11;
		break;

		// ready 위치로 이동 (x->y)
	case 11:
		move_start = move_pd_with_speed_ratio(POINT_READY, 0x01, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 12:
		if (move_done(0x01))
		{
			step++;
		}
		break;

	case 13:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 14:
		move_start = move_pd_with_speed_ratio(POINT_READY, 0x02, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 15:
		if (move_done(0x02))
		{
			step++;
		}
		break;

	case 16:
		if (IsStopped())
		{
			step = 20;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// shake. X축을 +/-방향으로 이동
	case 20:
		if (shake_count >= g_ShakeCount)
		{
			step = 30;
			break;
		}

		if (shake_count % 2 == 0)
		{
			move_start = move_abs(0x01, angle_1, SPEED_NORMAL, g_MoveRatio);
		}
		else
		{
			move_start = move_abs(0x01, angle_2, SPEED_NORMAL, g_MoveRatio);
		}
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 21:
		if (move_done(0x01))
		{
			delay_count = g_ShakeTiltDelay; // get_var(VAR_DELAY_MOVE);
			step++;
		}
		break;

	case 22:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 23:
		if (--delay_count > 0)
		{
			Delay1ms();
		}
		else
		{
			shake_count += 1;
			step = 20;
		}
		break;

		// new load 위치(15)로 이동 (x->y)
	case 30:
		move_start = move_pd_with_speed_ratio(POINT_NEW_LOAD, 0x01, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 31:
		if (move_done(0x01))
		{
			step++;
		}
		break;

	case 32:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 33:
		move_start = move_pd_with_speed_ratio(POINT_NEW_LOAD, 0x02, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;
	case 34:
		if (move_done(0x02))
		{
			step++;
		}
		break;
	case 35:
		if (IsStopped())
		{
			step = 40;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// 종료
	case 40:
		HoldMotors();
		g_MovePointDataNo = 0;
		g_MoveOffset[X_AXIS] = g_MoveOffset[Y_AXIS] = g_MoveOffset[Z_AXIS] = 0;
		step = 0;
		return NORMAL_FINISHED;

	default:
		step = 0;
		return NORMAL_FINISHED;
	}

	CHECK_USER_STOP();

	return 0;
}

char CommWaste()
{
	const int POINT_NEW_LOAD = 15;
	const int POINT_WASTE = 8;
	const int VAR_NUM_WASTE_DELAY = 8;
	static char step = 0;
	static int delay_count = 0;

	int move_start = 0;

	switch (step)
	{
		// Error handling
	case 91:
		StopMotors();
		step++;
		break;
	case 92:
		if (IsStopped())
		{
			step++;
		}
		break;
	case 93:
		HoldMotors();
		step = 0;
		return ERROR_STOPPED;

		// y축 위치를 확인 한다
	case 0:
		if (IsYAxisPositionMovable())
		{
			step = 10;
		}
		else
		{
			step = 1;
		}
		break;
		// y축을 PD15번 위치로 이동 한다
	case 1:
		move_start = move_pd(15, 0x02);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;
	case 2:
		if (move_done(0x02) && IsStopped())
		{
			step = 10;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 10:
		delay_count = get_var(VAR_NUM_WASTE_DELAY);
		step++;
		break;

		// pd8로 이동 (x축)
	case 11:
		move_start = move_pd_with_speed_ratio(POINT_WASTE, 0x01, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 12:
		if (move_done(0x01))
		{
			step++;
		}
		break;

	case 13:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// pd8로 이동 (y축)
	case 14:
		move_start = move_pd_with_speed_ratio(POINT_WASTE, 0x02, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 15:
		if (move_done(0x02))
		{
			delay_count = get_var(VAR_NUM_WASTE_DELAY);
			step++;
		}
		break;

	case 16:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// delay
	case 17:
		if (--delay_count > 0)
		{
			Delay1ms();
		}
		else
		{
			step++;
		}
		break;

		// new load 위치(pd15)로 이동. y축 부터 이동
	case 18:
		move_start = move_pd_with_speed_ratio(POINT_NEW_LOAD, 0x02, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 19:
		if (move_done(0x02))
		{
			step++;
		}
		break;

	case 20:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// new load 위치(pd15)로 이동. x축
	case 21:
		move_start = move_pd_with_speed_ratio(POINT_NEW_LOAD, 0x01, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 22:
		if (move_done(0x01))
		{
			step++;
		}
		break;

	case 23:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 24:
		HoldMotors();
		g_MovePointDataNo = 0;
		g_MoveOffset[X_AXIS] = g_MoveOffset[Y_AXIS] = g_MoveOffset[Z_AXIS] = 0;
		step = 0;
		return NORMAL_FINISHED;

	default:
		step = 0;
		return NORMAL_FINISHED;
	}

	CHECK_USER_STOP();

	return 0;
}

// PD11 이동 -> Output ON -> PD8 이동 -> Output OFF
// Sequence 수정 예정
//	- SCARA 출력 신호 확인 -> ON 이면 에러
//	- 대기 위치로 이동
//	- Output ON
//	- SCARA 출력 신호 확인
//		- 일정시간 ON 되지 않으면 에러
//		- ON 이면 Waste 동작 시작
//	- Waste 동작이 끝나면 Output OFF
//	- 대기 위치로 이동
char CommAsyncWaste()
{
	const int POINT_NEW_LOAD = 15;
	const int POINT_WASTE = 8;
	const int POINT_ASYNC_WASTE = 11;
	const int VAR_NUM_WASTE_DELAY = 8;
	const int VAR_NUM_ASYNC_DELAY = 14;
	const int VAR_NUM_ASYNC_INPUT_NO = 13;
	const int VAR_NUM_NO_USE_ASYNC_INPUT = 15;
	static char step = 0;
	static int delay_count = 0;

	int move_start = 0;

	switch (step)
	{
		// Error handling
	case 91:
		StopMotors();
		step++;
		break;
	case 92:
		if (IsStopped())
		{
			step++;
		}
		break;
	case 93:
		HoldMotors();
		step = 0;
		return ERROR_STOPPED;

	case 0:
		AsyncWasteOff();
		// SCARA의 Input이 ON 되어 있으면, 에러
		if (get_var(VAR_NUM_NO_USE_ASYNC_INPUT) == 0 &&
			GetWasteAsyncInput())
		{
			SetErrorCode(ERR_ASYNC_WASTE_OUTPUT_ON);
			step = 91;
			break;
		}
		//
		delay_count = get_var(VAR_NUM_WASTE_DELAY);
		step = 1;
		break;

	// y축 위치를 확인 한다
	case 1:
		if (IsYAxisPositionMovable())
		{
			step = 11;
		}
		else
		{
			step = 2;
		}
		break;

		// y축을 PD15번 위치로 이동 한다
	case 2:
		move_start = move_pd(15, 0x02);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;
	case 3:
		if (move_done(0x02) && IsStopped())
		{
			step = 11;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// pd8로 이동 (x축)
	case 11:
		move_start = move_pd(POINT_WASTE, 0x01);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 12:
		if (move_done(0x01))
		{
			step++;
		}
		break;

	case 13:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// pd11로 이동 (y축)
	case 14:
		move_start = move_pd(POINT_ASYNC_WASTE, 0x02);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;
	case 15:
		if (move_done(0x02))
		{
			delay_count = get_var(VAR_NUM_ASYNC_DELAY);
			step++;
		}
		break;
	case 16:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;
		// y축 이동 후 Async signal on
	case 17:
		AsyncWasteOn();
		step++;
		if (get_var(VAR_NUM_NO_USE_ASYNC_INPUT) != 0)
		{
			// SCARA의 Input 신호를 사용하지 않을 경우, step 8을 skip. step 9로 이동
			step++;
		}
		break;
		// Async Delay 시간 동안 대기
		// SCARA의 출력 신호를 기다린다 (V14)
	case 18:
		if (--delay_count > 0)
		{
			if (GetWasteAsyncInput())
			{
				step++;
				break;
			}
			Delay1ms();
		}
		else
		{
			// Timeout 에러
			SetErrorCode(ERR_ASYNC_WASTE_TIMEOUT);
			step = 91;
			// 신호를 끈다
			AsyncWasteOff();
			return NORMAL_RUNNING;
		}
		break;

	case 19:
		step++;
		break;

		// pd8로 이동 (y축)
	case 20:
		move_start = move_pd(POINT_WASTE, 0x02);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 21:
		if (move_done(0x02))
		{
			delay_count = get_var(VAR_NUM_WASTE_DELAY);
			step++;
		}
		break;

	case 22:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// delay
	case 23:
		if (--delay_count > 0)
		{
			Delay1ms();
		}
		else
		{
			step++;
		}
		break;

		// load 위치(pd3)로 이동. y축 부터 이동 -> pd15
	case 24:
		move_start = move_pd(POINT_NEW_LOAD, 0x02);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 25:
		if (move_done(0x02))
		{
			step++;
		}
		break;

	case 26:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// load 위치(pd3)로 이동. x축 -> pd15
	case 27:
		AsyncWasteOff();
		move_start = move_pd(POINT_NEW_LOAD, 0x01);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 28:
		if (move_done(0x01))
		{
			step++;
		}
		break;

	case 29:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 30:
		HoldMotors();
		g_MovePointDataNo = 0;
		g_MoveOffset[X_AXIS] = g_MoveOffset[Y_AXIS] = g_MoveOffset[Z_AXIS] = 0;
		step = 0;
		return NORMAL_FINISHED;

	default:
		step = 0;
		return NORMAL_FINISHED;
	}

	CHECK_USER_STOP();

	return 0;
}

// X(PD8) -> Y(PD11)
char CommReadyWaste()
{
	const int POINT_WASTE = 8;	// X축 이동
	const int POINT_READY = 11; // Y축 이동
	static int step = 0;
	int move_start = 0;

	switch (step)
	{
		// Error handling
	case 91:
		StopMotors();
		step++;
		break;
	case 92:
		if (IsStopped())
		{
			step++;
		}
		break;
	case 93:
		HoldMotors();
		step = 0;
		return ERROR_STOPPED;

		// y축 위치를 확인 한다
	case 0:
		if (IsYAxisPositionMovable())
		{
			step = 10;
		}
		else
		{
			step = 1;
		}
		break;

		// y축을 PD15번 위치로 이동 한다
	case 1:
		move_start = move_pd(15, 0x02);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;
	case 2:
		if (move_done(0x02) && IsStopped())
		{
			step = 10;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// X축 (PD8) 이동
	case 10:
		move_start = move_pd_with_speed_ratio(POINT_WASTE, 0x01, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;
	case 11:
		if (move_done(0x01))
		{
			step++;
		}
		break;
	case 12:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// Y축 (PD11) 이동
	case 13:
		move_start = move_pd_with_speed_ratio(POINT_READY, 0x02, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;
	case 14:
		if (move_done(0x02))
		{
			step++;
		}
		break;
	case 15:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 16:
		HoldMotors();
		g_MovePointDataNo = 0;
		g_MoveOffset[X_AXIS] = g_MoveOffset[Y_AXIS] = g_MoveOffset[Z_AXIS] = 0;
		step = 0;
		return NORMAL_FINISHED;

	default:
		step = 0;
		return NORMAL_FINISHED;
	}

	CHECK_USER_STOP();

	return 0;
}

// 실행하기 전에 현재 x, y 위치를 먼저 확인한다
// Y(PD8) -> Delay -> Y(PD3) -> X(PD3)
char CommPourWaste()
{
	const int POINT_NEW_LOAD = 15;
	const int POINT_WASTE = 8;
	const int VAR_NUM_WASTE_DELAY = 8;
	static char step = 0;
	static int delay_count = 0;

	int move_start = 0;

	switch (step)
	{
		// Error handling
	case 91:
		StopMotors();
		step++;
		break;
	case 92:
		if (IsStopped())
		{
			step++;
		}
		break;
	case 93:
		HoldMotors();
		step = 0;
		return ERROR_STOPPED;

		// pd8로 이동 (y축)
	case 0: // 4:
		move_start = move_pd_with_speed_ratio(POINT_WASTE, 0x02, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 1: // 5:
		if (move_done(0x02))
		{
			delay_count = get_var(VAR_NUM_WASTE_DELAY);
			step++;
		}
		break;

	case 2: // 6:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// delay
	case 3: // 7:
		if (--delay_count > 0)
		{
			Delay1ms();
		}
		else
		{
			step++;
		}
		break;

		// load 위치(pd3)로 이동. y축 부터 이동
	case 4: // 8:
		move_start = move_pd_with_speed_ratio(POINT_NEW_LOAD, 0x02, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 5: // 9:
		if (move_done(0x02))
		{
			step++;
		}
		break;

	case 6: // 10:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// load 위치(pd3)로 이동. x축
	case 7: // 11:
		move_start = move_pd_with_speed_ratio(POINT_NEW_LOAD, 0x01, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 8: // 12:
		if (move_done(0x01))
		{
			step++;
		}
		break;

	case 9: // 13:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 10: // 14:
		HoldMotors();
		g_MovePointDataNo = 0;
		g_MoveOffset[X_AXIS] = g_MoveOffset[Y_AXIS] = g_MoveOffset[Z_AXIS] = 0;
		step = 0;
		return NORMAL_FINISHED;

	default:
		step = 0;
		return NORMAL_FINISHED;
	}

	CHECK_USER_STOP();

	return 0;
}

// pd9.  수직으로 세워서 용액을 나누는 위치 (x,y 동시 이동)
// pd10. 옆으로 뉘어서 용액을 분리 (y축만 이동)
// pd3.  Load 위치로 이동 (x->y 순서로 이동)
// var9. 용액을 나누는 위치에서, 용액이 분리 될때까지 대기 시간
// var10. 나머지 모터 이동 delay
char CommSeparate()
{
	const int POINT_SEP = 9;
	const int POINT_SEL = 10;
	const int POINT_NEW_LOAD = 15;
	const int VAR_DELAY_SEP = 9;
	const int VAR_DELAY_MOV = 10;

	static char step = 0;
	static int delay_count = 0;

	int move_start = 0;

	switch (step)
	{
		// Error handling
	case 91:
		StopMotors();
		step++;
		break;
	case 92:
		if (IsStopped())
		{
			step++;
		}
		break;
	case 93:
		HoldMotors();
		step = 0;
		return ERROR_STOPPED;

	case 0:
		step = 1;
		break;

		// y축 위치를 확인 한다
	case 1:
		if (IsYAxisPositionMovable())
		{
			step = 11;
		}
		else
		{
			step = 2;
		}
		break;
		// y축을 PD15번 위치로 이동 한다
	case 2:
		move_start = move_pd(15, 0x02);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;
	case 3:
		if (move_done(0x02) && IsStopped())
		{
			step = 10;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// pd9 로 이동 (x,y 동시)
	case 11:
		move_start = move_pd(POINT_SEP, 0x03);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 12:
		if (move_done(0x03))
		{
			step++;
		}
		break;

	case 13:
		if (IsStopped())
		{
			delay_count = get_var(VAR_DELAY_SEP);
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 14:
		if (--delay_count > 0)
		{
			Delay1ms();
		}
		else
		{
			step++;
		}
		break;

		// pd10 으로 이동 (y축만 90도 회전)
		// var12가 1이면 y축 이동을 하지 않는다
	case 15:
		if (get_var(12) == 1)
		{
			step = 19;
			break;
		}

		move_start = move_pd(POINT_SEL, 0x02);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 16:
		if (move_done(0x02))
		{
			step++;
		}
		break;

	case 17:
		if (IsStopped())
		{
			delay_count = get_var(VAR_DELAY_MOV);
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 18:
		if (--delay_count > 0)
		{
			Delay1ms();
		}
		else
		{
			step++;
		}
		break;

		// pd3 으로 이동 (x->y 순서)
	case 19:
		// move_start = move_pd(POINT_LOAD, 0x01);
		move_start = move_pd_with_speed(POINT_NEW_LOAD, 0x01, SPEED_SEPARATE);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 20:
		if (move_done(0x01))
		{
			step++;
		}
		break;

	case 21:
		if (IsStopped())
		{
			delay_count = get_var(VAR_DELAY_MOV);
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 22:
		if (--delay_count > 0)
		{
			Delay1ms();
		}
		else
		{
			step++;
		}
		break;

	case 23:
		move_start = move_pd(POINT_NEW_LOAD, 0x02);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 24:
		if (move_done(0x02))
		{
			step++;
		}
		break;

	case 25:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 26:
		HoldMotors();
		g_MovePointDataNo = 0;
		g_MoveOffset[X_AXIS] = g_MoveOffset[Y_AXIS] = g_MoveOffset[Z_AXIS] = 0;
		step = 0;
		return NORMAL_FINISHED;

	default:
		step = 0;
		return NORMAL_FINISHED;
	}

	CHECK_USER_STOP();

	return 0;
}

// EQIL
// Flask를 수직으로 세워 용액을 나눈 후 바로 올린다
// PD 9번 X축 데이터만 사용
// VAR9 용액을 나누는 위치에서, 용액이 분리될때 까지 대기 시간
char CommSeparateLongSide()
{
	const int POINT_NEW_LOAD = 15;
	const int POINT_SEP = 9;
	const int POINT_MORE_TILT = 13;
	const int VAR_DELAY_SEP = 9;
	const int VAR_DELAY_SEP2 = 10;
	const int VAR_HW_TYPE = 11;

	static int step = 0;
	static int delay_count = 0;
	int move_start = 0;

	switch (step)
	{
		// Error handling
	case 91:
		StopMotors();
		step++;
		break;
	case 92:
		if (IsStopped())
		{
			step++;
		}
		break;
	case 93:
		HoldMotors();
		step = 0;
		return ERROR_STOPPED;

	case 0:
		step = 1;
		break;

		// y축 위치를 확인 한다
	case 1:
		if (IsYAxisPositionMovable())
		{
			step = 11;
		}
		else
		{
			step = 2;
		}
		break;

		// y축을 PD15번 위치로 이동 한다
	case 2:
		move_start = move_pd(15, 0x02);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;
	case 3:
		if (move_done(0x02) && IsStopped())
		{
			step = 11;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// pd9로 이동. xy 동시
	case 11:
		move_start = move_pd_with_speed_ratio(POINT_SEP, 0x03, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 12:
		if (move_done(0x03))
		{
			step++;
		}
		break;

	case 13:
		if (IsStopped())
		{
			delay_count = get_var(VAR_DELAY_SEP);
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// 용액이 나누어 질때까지 대기
	case 14:
		if (--delay_count > 0)
		{
			Delay1ms();
		}
		else
		{
			step += 1;
		}
		break;

		//
	case 15:
		if (get_var(VAR_HW_TYPE) == 0)
		{
			step = 30;
		}
		else
		{
			step = 20;
		}
		break;

		// PD 13 (Tilt) 위치로 이동
	case 20:
		move_start = move_pd_with_speed_ratio(POINT_MORE_TILT, 0x01, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step += 1;
		break;

	case 21:
		if (move_done(0x01))
		{
			step += 1;
		}
		break;

	case 22:
		if (IsStopped())
		{
			delay_count = get_var(VAR_DELAY_SEP2);
			step += 1;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 23:
		if (--delay_count > 0)
		{
			Delay1ms();
		}
		else
		{
			step = 30;
		}
		break;

		// pd3(load) 으로 이동 (xy 동시)
	case 30:
		move_start = move_pd_with_speed_ratio(POINT_NEW_LOAD, 0x03, SPEED_NORMAL, g_MoveRatioSeparate);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 31:
		if (move_done(0x03))
		{
			step++;
		}
		break;

	case 32:
		if (IsStopped())
		{
			delay_count = get_var(VAR_DELAY_SEP);
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 33:
		HoldMotors();
		g_MovePointDataNo = 0;
		g_MoveOffset[X_AXIS] = g_MoveOffset[Y_AXIS] = g_MoveOffset[Z_AXIS] = 0;
		step = 0;
		return NORMAL_FINISHED;

	default:
		step = 0;
		return NORMAL_FINISHED;
	}

	CHECK_USER_STOP();

	return 0;
}

// Flask를 수직으로 세워 용액은 나눈 후 Y축으로 이동후 X축을 올린다
// 그 후 Y축을 대기 위치로 이동한다
// PD 10번 X, Y축 데이터 사용
char CommSeparateShortSide()
{
	const int POINT_NEW_LOAD = 15;
	const int POINT_SEP = 10;
	const int POINT_MORE_TILE = 14;
	const int VAR_DELAY_SEP = 9;
	// const int VAR_DELAY_MOV = 10;
	const int VAR_DELAY_SEP2 = 10;
	const int VAR_HW_TYPE = 11;

	static int step = 0;
	static int delay_count = 0;
	int move_start = 0;

	switch (step)
	{
		// Error handling
	case 91:
		StopMotors();
		step++;
		break;
	case 92:
		if (IsStopped())
		{
			step++;
		}
		break;
	case 93:
		HoldMotors();
		step = 0;
		return ERROR_STOPPED;

	case 0:
		step = 1;
		break;

		// y축 위치를 확인 한다
	case 1:
		if (IsYAxisPositionMovable())
		{
			step = 11;
		}
		else
		{
			step = 2;
		}
		break;

		// y축을 PD15번 위치로 이동 한다
	case 2:
		move_start = move_pd(15, 0x02);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;
	case 3:
		if (move_done(0x02) && IsStopped())
		{
			step = 11;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// pd10로 이동. x 축만
	case 11:
		move_start = move_pd_with_speed_ratio(POINT_SEP, 0x01, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 12:
		if (move_done(0x03))
		{
			step++;
		}
		break;

	case 13:
		if (IsStopped())
		{
			delay_count = get_var(VAR_DELAY_SEP);
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// 용액이 나누어 질때까지 대기
	case 14:
		if (--delay_count > 0)
		{
			Delay1ms();
		}
		else
		{
			step += 1;
		}
		break;

		// pd10 으로 이동. y 축만
	case 15:
		move_start = move_pd_with_speed_ratio(POINT_SEP, 0x02, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 16:
		if (move_done(0x02))
		{
			step++;
		}
		break;

	case 17:
		if (IsStopped())
		{
			if (get_var(VAR_HW_TYPE) == 0)
			{
				step = 30;
			}
			else
			{
				step = 20;
			}
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// PD 14 (Tilt) 위치로 이동
	case 20:
		move_start = move_pd_with_speed_ratio(POINT_MORE_TILE, 0x01, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step += 1;
		break;

	case 21:
		if (move_done(0x01))
		{
			step += 1;
		}
		break;

	case 22:
		if (IsStopped())
		{
			delay_count = get_var(VAR_DELAY_SEP2);
			step += 1;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 23:
		if (--delay_count > 0)
		{
			Delay1ms();
		}
		else
		{
			step = 30;
		}
		break;

		//
	case 30:
		if (IsStopped())
		{
			delay_count = get_var(VAR_DELAY_SEP2);
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;
	case 31:
		if (--delay_count > 0)
		{
			Delay1ms();
		}
		else
		{
			step++;
		}
		break;

		// pd3으로 이동. x축만 빠르게
	case 32:
		move_start = move_pd_with_speed_ratio(POINT_NEW_LOAD, 0x01, SPEED_NORMAL, g_MoveRatioSeparate);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 33:
		if (move_done(0x01))
		{
			step++;
		}
		break;

	case 34:
		if (IsStopped())
		{
			delay_count = 100; // get_var(VAR_DELAY_MOV);
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 35:
		if (--delay_count > 0)
		{
			Delay1ms();
		}
		else
		{
			step++;
		}
		break;

		// pd3으로 이동. y축만
		// pd10 으로 이동. y 축만
	case 36:
		move_start = move_pd_with_speed_ratio(POINT_NEW_LOAD, 0x02, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 37:
		if (move_done(0x02))
		{
			step++;
		}
		break;

	case 38:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 39:
		HoldMotors();
		g_MovePointDataNo = 0;
		g_MoveOffset[X_AXIS] = g_MoveOffset[Y_AXIS] = g_MoveOffset[Z_AXIS] = 0;
		step = 0;
		return NORMAL_FINISHED;

	default:
		step = 0;
		return NORMAL_FINISHED;
	}

	CHECK_USER_STOP();

	return 0;
}

// X축 방향 Shake (front-back-angle),
// Y축 방향 Shake (side-by-side-angle)
// g_MoveRatio = ints.val[0];
// g_MoveRatio_Rotation = ints.val[1]
// g_ShakeCount = ints.val[2];
// g_ShakeAngleX = ints.val[3];
// g_ShakeAngleY = ints.val[4];
// g_ShakeTiltDelay = ints.val[5];
char CommSWIRL()
{
	const int POINT_NEW_LOAD = 15;
	static char step = 0;
	static int delay_count = 0;
	static int shake_count = 0;

	float angle[3] = {0.0, 0.0, 0.0};
	int move_start = 0;
	POINT_DATA pd;

	switch (step)
	{
		// Error Handling
	case 91:
		StopMotors();
		step++;
		break;
	case 92:
		if (IsStopped())
		{
			step++;
		}
		break;
	case 93:
		HoldMotors();
		step = 0;
		return ERROR_STOPPED;

	case 0:
		delay_count = 0;
		shake_count = 0;
		pd = get_point_data(POINT_NEW_LOAD);
		step = 1;
		break;

		// y축 위치를 확인 한다
	case 1:
		if (IsYAxisPositionMovable())
		{
			step = 11;
		}
		else
		{
			step = 2;
		}
		break;

		// y축을 PD15번 위치로 이동 한다
	case 2:
		move_start = move_pd(15, 0x02);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;
	case 3:
		if (move_done(0x02) && IsStopped())
		{
			step = 11;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// ready 위치로 이동
	case 11:
		move_start = move_pd_with_speed_ratio(POINT_NEW_LOAD, 0x01, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step = 12;
		break;
	case 12:
		if (move_done(0x01))
		{
			step = 13;
		}
		break;
	case 13:
		if (IsStopped())
		{
			step = 14;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 14:
		move_start = move_pd_with_speed_ratio(POINT_NEW_LOAD, 0x02, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step = 15;
		break;
	case 15:
		if (move_done(0x02))
		{
			step = 16;
		}
		break;
	case 16:
		if (IsStopped())
		{
			step = 20;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		///////////////////////////////////////////////
		// SHAEK
		// - shake count 회수 만큼
		// - x 축 (g_ShakeAngleX)
		// - Y 축 (g_ShakeAngleY)
		///////////////////////////////////////////////
	case 20:
		if (shake_count++ >= g_ShakeCount)
		{
			step = 100;
		}
		else
		{
			step = 21;
		}
		break;

		// X축 Shake
	case 21:
		do_shake_xy(1, 0x01, g_ShakeAngleX, 1);
		step = 22;
		break;
	case 22:
		move_start = do_shake_xy(0, 0x01, g_ShakeAngleX, 1);
		if (move_start == 0)
		{
			step = step;
		}
		else if (move_start == 1)
		{
			step = 31;
		}
		else if (move_start == 2)
		{
			g_MoveStartErrorLine = __LINE__;
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		else
		{
			step = 91;
		}
		break;

		// Y축 Shake
	case 31:
		do_shake_xy(1, 0x02, g_ShakeAngleY, 1);
		step = 32;
		break;
	case 32:
		move_start = do_shake_xy(0, 0x02, g_ShakeAngleY, 1);
		if (move_start == 0)
		{
			step = step;
		}
		else if (move_start == 1)
		{
			step = 33;
		}
		else if (move_start == 2)
		{
			g_MoveStartErrorLine = __LINE__;
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		else
		{
			step = 91;
		}
		break;
	case 33:
		step = 11;
		break;

		//
	case 100:
		HoldMotors();
		g_MovePointDataNo = 0;
		g_MoveOffset[X_AXIS] = g_MoveOffset[Y_AXIS] = g_MoveOffset[Z_AXIS] = 0;
		step = 0;
		return NORMAL_FINISHED;

	default:
		step = 0;
		return NORMAL_FINISHED;
	}

	CHECK_USER_STOP();

	return 0;
}

//
char CommMAMV()
{
	static char step = 0;
	float pos[3];
	int move_start = 0;

	switch (step)
	{
		// Error handling
	case 91:
		StopMotors();
		step++;
		break;
	case 92:
		if (IsStopped())
		{
			step++;
		}
		break;
	case 93:
		HoldMotors();
		step = 0;
		return ERROR_STOPPED;

		// y축 위치를 확인 한다
	case 0:
		if (IsYAxisPositionMovable())
		{
			step = 10;
		}
		else
		{
			step = 1;
		}
		break;

		// y축을 PD15번 위치로 이동 한다
	case 1:
		move_start = move_pd(15, 0x02);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;
	case 2:
		if (move_done(0x02) && IsStopped())
		{
			step = 10;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 10:
		pos[0] = g_fMoveXPos;
		pos[1] = g_fMoveYPos;
		pos[2] = 0.0;
		move_start = move_abs(0x03, pos, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;

	case 11:
		if (move_done(0x03))
		{
			step++;
		}
		break;

	case 12:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 13:
		// SetHoldTorque(Z_AXIS);
		HoldMotors();
		step++;
		break;

	default:
		step = 0;
		return NORMAL_FINISHED;
	}

	CHECK_USER_STOP();

	return 0;
}

//
char CommRAMV()
{
	const int POINT_GRIP = 1;
	static char step = 0;
	static int delay_count = 0;
	int move_start = 0;
	float pos[3];

	switch (step)
	{
		// Error handling
	case 91:
		StopMotors();
		step++;
		break;
	case 92:
		if (IsStopped())
		{
			step++;
		}
		break;
	case 93:
		HoldMotors();
		step = 0;
		return ERROR_STOPPED;

		// y축 위치를 확인 한다
	case 0:
		if (IsYAxisPositionMovable())
		{
			step = 10;
		}
		else
		{
			step = 1;
		}
		break;

		// y축을 PD15번 위치로 이동 한다
	case 1:
		move_start = move_pd(15, 0x02);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;
	case 2:
		if (move_done(0x02) && IsStopped())
		{
			step = 10;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// REGRIP 위치로 X축 이동
	case 10:
		pos[0] = g_fRegripXPos;
		pos[1] = g_fRegripYPos;
		pos[2] = 0.0;
		move_start = move_abs(0x03, pos, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step += 1;
		break;

	case 11:
		if (move_done(0x03))
		{
			step++;
		}
		break;

	case 12:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// Gripper Ungrip. PD7번의 Z축 위치로 이동
	case 13:
		pos[0] = pos[1] = 0.0;
		pos[2] = g_fRegripZPos;
		move_start = move_abs(0x04, pos, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step += 1;
		break;

	case 14:
		if (move_done(0x04))
		{
			step++;
		}
		break;

	case 15:
		if (IsStopped())
		{
			delay_count = g_nRegripDelay;
			step += 1;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// Wait (PD2)
	case 16:
		if (--delay_count > 0)
		{
			Delay1ms();
		}
		else
		{
			step += 1;
		}
		break;

		// GRIP
	case 17:
		move_start = move_pd_with_speed_ratio(POINT_GRIP, 0x04, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step += 1;
		break;

	case 18:
		if (move_done(0x04))
		{
			step += 1;
		}
		break;

	case 19:
		if (IsStopped())
		{
			step += 1;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// x, y 위치로 이동
	case 20:
		pos[0] = g_fMoveXPos;
		pos[1] = g_fMoveYPos;
		pos[2] = 0.0;
		move_start = move_abs(0x03, pos, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step += 1;
		break;

	case 21:
		if (move_done(0x03))
		{
			step += 1;
		}
		break;

	case 22:
		if (IsStopped())
		{
			step += 1;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 23:
		// SetHoldTorque(Z_AXIS);
		HoldMotors();
		step += 1;
		break;

	default:
		step = 0;
		return NORMAL_FINISHED;
	}

	CHECK_USER_STOP();

	return 0;
}

//
char CommMRGI()
{
	const int POINT_GRIP = 1;
	const int POINT_NEW_LOAD = 15;
	const int POINT_REGRIP = 7;
	static char step = 0;
	static int delay_count = 0;
	int move_start = 0;

	switch (step)
	{
		// Error handling
	case 91:
		StopMotors();
		step++;
		break;
	case 92:
		if (IsStopped())
		{
			step++;
		}
		break;
	case 93:
		HoldMotors();
		step = 0;
		return ERROR_STOPPED;

		// y축 위치를 확인 한다
	case 0:
		if (IsYAxisPositionMovable())
		{
			step = 10;
		}
		else
		{
			step = 1;
		}
		break;

		// y축을 PD15번 위치로 이동 한다
	case 1:
		move_start = move_pd(15, 0x02);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;
	case 2:
		if (move_done(0x02) && IsStopped())
		{
			step = 10;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// REGRIP 위치로 X, Y축 이동
	case 10:
		move_start = move_pd_with_speed_ratio(POINT_REGRIP, 0x03, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step += 1;
		break;

	case 11:
		if (move_done(0x03))
		{
			step++;
		}
		break;

	case 12:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// Gripper Ungrip. PD7번의 Z축 위치로 이동
	case 13:
		move_start = move_pd_with_speed_ratio(POINT_REGRIP, 0x04, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step += 1;
		break;

	case 14:
		if (move_done(0x04))
		{
			step++;
		}
		break;

	case 15:
		if (IsStopped())
		{
			delay_count = get_var(2);
			step += 1;
			// sendf("dc = %d\r\n", delay_count);
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// Wait (PD2)
	case 16:
		if (--delay_count > 0)
		{
			Delay1ms();
		}
		else
		{
			step += 1;
		}
		break;

		// GRIP
	case 17:
		move_start = move_pd_with_speed_ratio(POINT_GRIP, 0x04, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step += 1;
		break;

	case 18:
		if (move_done(0x04))
		{
			step += 1;
		}
		break;

	case 19:
		if (IsStopped())
		{
			step += 1;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// LOAD 위치로 이동
	case 20:
		move_start = move_pd_with_speed_ratio(POINT_NEW_LOAD, 0x03, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step += 1;
		break;

	case 21:
		if (move_done(0x03))
		{
			step += 1;
		}
		break;

	case 22:
		if (IsStopped())
		{
			step += 1;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 23:
		// SetHoldTorque(Z_AXIS);
		HoldMotors();
		step += 1;
		break;

	default:
		step = 0;
		return NORMAL_FINISHED;
	}

	CHECK_USER_STOP();

	return 0;
}

//
char CommRASP()
{
	const int POINT_GRIP = 1;
	const int POINT_ASP = 4;
	static char step = 0;
	static int delay_count = 0;
	int move_start = 0;
	float pos[3];

	switch (step)
	{
		// Error handling
	case 91:
		StopMotors();
		step++;
		break;
	case 92:
		if (IsStopped())
		{
			step++;
		}
		break;
	case 93:
		HoldMotors();
		step = 0;
		return ERROR_STOPPED;

		// y축 위치를 확인 한다
	case 0:
		if (IsYAxisPositionMovable())
		{
			step = 10;
		}
		else
		{
			step = 1;
		}
		break;

		// y축을 PD15번 위치로 이동 한다
	case 1:
		move_start = move_pd(15, 0x02);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step++;
		break;
	case 2:
		if (move_done(0x02) && IsStopped())
		{
			step = 10;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// REGRIP 위치로 X축 이동
	case 10:
		pos[0] = g_fRegripXPos;
		pos[1] = g_fRegripYPos;
		pos[2] = 0.0;
		move_start = move_abs(0x03, pos, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step += 1;
		break;

	case 11:
		if (move_done(0x03))
		{
			step++;
		}
		break;

	case 12:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// Gripper Ungrip. PD7번의 Z축 위치로 이동
	case 13:
		pos[0] = pos[1] = 0.0;
		pos[2] = g_fRegripZPos;
		move_start = move_abs(0x04, pos, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step += 1;
		break;

	case 14:
		if (move_done(0x04))
		{
			step++;
		}
		break;

	case 15:
		if (IsStopped())
		{
			delay_count = g_nRegripDelay;
			step += 1;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// Wait (PD2)
	case 16:
		if (--delay_count > 0)
		{
			Delay1ms();
		}
		else
		{
			step += 1;
		}
		break;

		// GRIP
	case 17:
		move_start = move_pd_with_speed_ratio(POINT_GRIP, 0x04, SPEED_NORMAL, g_MoveRatio);
		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step += 1;
		break;

	case 18:
		if (move_done(0x04))
		{
			step += 1;
		}
		break;

	case 19:
		if (IsStopped())
		{
			step += 1;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

		// ASP 위치로 이동
	case 20:
		//	move_start = move_pd_with_speed_ratio(POINT_ASP, 0x03, SPEED_NORMAL, g_MoveRatio);
		move_start = move_pd_with_speed_ratio_xy_offset(
			POINT_ASP,
			0x03,
			SPEED_NORMAL,
			g_MoveRatio,
			g_fMASPOffset[0],
			g_fMASPOffset[1]);

		g_MoveStartErrorLine = __LINE__;
		if (move_start)
		{
			SetErrorCode(ERR_MOTOR_ERROR);
			step = 91;
			return NORMAL_RUNNING;
		}
		DelayMoveStart();
		step += 1;
		break;

	case 21:
		if (move_done(0x03))
		{
			step += 1;
		}
		break;

	case 22:
		if (IsStopped())
		{
			step += 1;
			if (CheckEncoder() == 0)
			{
				step = 91;
				return NORMAL_RUNNING;
			}
		}
		break;

	case 23:
		// SetHoldTorque(Z_AXIS);
		HoldMotors();
		step += 1;
		break;

	default:
		step = 0;
		return NORMAL_FINISHED;
	}

	CHECK_USER_STOP();

	return 0;
}

// return
// - 0 : continue
// - 1 : end
// - 2 : motor error
// - 3 : encoder error
int do_shake_xy(char reset_step, int axis, int angle, int count)
{
	const int POINT_NEW_LOAD = 15;
	const int POINT_YPOS = 6;
	const int VAR_DELAY_MOVE = 7;
	static int step = 0;
	static int shake_count = 0;
	static int delay_count = 0;

	int move_start = 0;
	float move_angle[3] = {0.0, 0.0, 0.0};
	POINT_DATA pd;

	step = (reset_step == 1) ? 1 : step;

	switch (step)
	{
	case 1:
		shake_count = 0;
		step++;
		break;

	case 2:
		move_start = move_pd_with_speed_ratio(POINT_NEW_LOAD, 0x03, SPEED_NORMAL, g_MoveRatio);
		if (move_start)
		{
			return 2;
		}
		DelayMoveStart();
		step++;
	case 3:
		if (move_done(0x03))
		{
			step++;
		}
		break;
	case 4:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
				return 3;
		}
		break;
	case 5:
		if ((axis & 0x02) == 0x02)
		{
			pd = get_point_data(POINT_YPOS);
			move_angle[0] = 0.0;
			move_angle[1] = pd.y;
			move_angle[2] = 0.0;
			move_start = move_abs(0x02, move_angle, SPEED_NORMAL, g_MoveRatio_Rotation);
			if (move_start)
			{
				return 2;
			}
			DelayMoveStart();
			step++;
		}
		else
		{
			step = 10;
		}
		break;
	case 6:
		if (move_done(0x02))
		{
			step++;
		}
		break;
	case 7:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
				return 3;
		}
		break;
	case 8:
		step = 10;
		break;

	case 10:
		if (shake_count++ >= count)
		{
			step = 30;
		}
		else
		{
			step++;
		}
		break;

	case 11:
		pd = get_point_data(POINT_NEW_LOAD);
		move_angle[0] = pd.x + angle;
		move_start = move_abs(0x01, move_angle, SPEED_NORMAL, g_MoveRatio);
		if (move_start)
		{
			return 2;
		}
		DelayMoveStart();
		step++;
		break;
	case 12:
		if (move_done(0x01))
		{
			delay_count = g_ShakeTiltDelay; // get_var(VAR_DELAY_MOVE);
			step++;
		}
		break;
	case 13:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
				return 3;
		}
		break;
	case 14:
		if (--delay_count > 0)
		{
			Delay1ms();
		}
		else
		{
			step++;
		}
		break;

	case 15:
		pd = get_point_data(POINT_NEW_LOAD);
		move_angle[0] = pd.x - angle;
		move_start = move_abs(0x01, move_angle, SPEED_NORMAL, g_MoveRatio);
		if (move_start)
		{
			return 2;
		}
		DelayMoveStart();
		step++;
		break;

	case 16:
		if (move_done(0x01))
		{
			delay_count = g_ShakeTiltDelay; // get_var(VAR_DELAY_MOVE);
			step++;
		}
		break;
	case 17:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
				return 3;
		}
		break;
	case 18:
		if (--delay_count > 0)
		{
			Delay1ms();
		}
		else
		{
			step++;
		}
		break;
	case 19:
		step = 10;
		break;

	case 30:
		move_start = move_pd_with_speed_ratio(POINT_NEW_LOAD, 0x01, SPEED_NORMAL, g_MoveRatio);
		if (move_start)
		{
			return 2;
		}
		DelayMoveStart();
		step++;
		break;
	case 31:
		if (move_done(0x01))
		{
			delay_count = 1; // g_ShakeTiltDelay; //get_var(VAR_DELAY_MOVE);
			step++;
		}
		break;
	case 32:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
				return 3;
		}
		break;
	case 33:
		if (--delay_count > 0)
		{
			Delay1ms();
		}
		else
		{
			step++;
		}
		break;

	case 34:
		if ((axis & 0x02) == 0x02)
		{
			step++;
		}
		else
			step = 0;
		break;
	case 35:
		move_start = move_pd_with_speed_ratio(POINT_NEW_LOAD, 0x03, SPEED_NORMAL, g_MoveRatio_Rotation);
		if (move_start)
		{
			return 2;
		}
		step++;
		break;
	case 36:
		if (move_done(0x03))
		{
			step++;
		}
		break;
	case 37:
		if (IsStopped())
		{
			step++;
			if (CheckEncoder() == 0)
				return 3;
		}
		break;
	case 38:
		step = 0;
		break;
	}

	return (step == 0) ? 1 : 0;
}

// pd_no 위치로 이동
// axis 는 이동할 축 번호 비트만 1
// - 1번 축 => 0x01
// - 2번 축 => 0x02
// - 1,2번 축 => 0x03
// return
// - MoveStart 값을 리턴한다
// - 0: ok
// - n: error code
int move_pd(int pd_no, char sel_axis)
{
	POINT_DATA pd;
	// int  move_start = 0;
	char axis;
	char mask = 1;

	pd = get_point_data(pd_no);

	for (axis = 0; axis < MAX_AXIS; axis++)
	{
		if ((sel_axis & mask) == mask)
		{
			switch (axis)
			{
			case X_AXIS:
				g_MoveOffset[axis] = (pd.x - get_motor_pos(axis)) / g_MotionParam[axis].m_fScaleFactor;
				break;
			case Y_AXIS:
				g_MoveOffset[axis] = (pd.y - get_motor_pos(axis)) / g_MotionParam[axis].m_fScaleFactor;
				break;
			case Z_AXIS:
				g_MoveOffset[axis] = (pd.z - get_motor_pos(axis)) / g_MotionParam[axis].m_fScaleFactor;
				break;
			}
			// g_MoveOffset[axis] = (pd.x - get_motor_pos(axis)) / g_MotionParam[axis].m_fScaleFactor;

			if (g_MoveOffset[axis] != 0)
			{
				SetMoveOffset(axis, g_MoveOffset[axis]);
				SetSpeed(axis, SPEED_NORMAL);
				g_MoveStartErrorCode[axis] = MoveStart(axis);
				if (g_MoveStartErrorCode[axis])
				{
					return g_MoveStartErrorCode[axis];
				}
			}
		}
		else
		{
			g_MoveOffset[axis] = 0;
		}
		mask <<= 1;
	}

	return 0;
}

int move_pd_with_speed(int pd_no, int sel_axis, int spd_type)
{
	POINT_DATA pd;
	// int  move_start = 0;
	char axis;
	char mask = 1;

	pd = get_point_data(pd_no);

	for (axis = 0; axis < MAX_AXIS; axis++)
	{
		if ((sel_axis & mask) == mask)
		{
			switch (axis)
			{
			case X_AXIS:
				g_MoveOffset[axis] = (pd.x - get_motor_pos(axis)) / g_MotionParam[axis].m_fScaleFactor;
				break;
			case Y_AXIS:
				g_MoveOffset[axis] = (pd.y - get_motor_pos(axis)) / g_MotionParam[axis].m_fScaleFactor;
				break;
			case Z_AXIS:
				g_MoveOffset[axis] = (pd.z - get_motor_pos(axis)) / g_MotionParam[axis].m_fScaleFactor;
				break;
			}
			// g_MoveOffset[axis] = (pd.x - get_motor_pos(axis)) / g_MotionParam[axis].m_fScaleFactor;

			if (g_MoveOffset[axis] != 0)
			{
				SetMoveOffset(axis, g_MoveOffset[axis]);
				SetSpeed(axis, spd_type);
				g_MoveStartErrorCode[axis] = MoveStart(axis);
				if (g_MoveStartErrorCode[axis])
				{
					return g_MoveStartErrorCode[axis];
				}
			}
		}
		else
		{
			g_MoveOffset[axis] = 0;
		}
		mask <<= 1;
	}

	return 0;
}

int move_pd_with_speed_ratio(int pd_no, int sel_axis, int spd_type, int spd_ratio)
{
	POINT_DATA pd;
	// int  move_start = 0;
	char axis;
	char mask = 1;

	pd = get_point_data(pd_no);

	for (axis = 0; axis < MAX_AXIS; axis++)
	{
		if ((sel_axis & mask) == mask)
		{
			switch (axis)
			{
			case X_AXIS:
				g_MoveOffset[axis] = (pd.x - get_motor_pos(axis)) / g_MotionParam[axis].m_fScaleFactor;
				break;
			case Y_AXIS:
				g_MoveOffset[axis] = (pd.y - get_motor_pos(axis)) / g_MotionParam[axis].m_fScaleFactor;
				break;
			case Z_AXIS:
				g_MoveOffset[axis] = (pd.z - get_motor_pos(axis)) / g_MotionParam[axis].m_fScaleFactor;
				break;
			}
			// g_MoveOffset[axis] = (pd.x - get_motor_pos(axis)) / g_MotionParam[axis].m_fScaleFactor;

			if (g_MoveOffset[axis] != 0)
			{
				SetMoveOffset(axis, g_MoveOffset[axis]);
				// SetSpeed(axis, spd);
				SetSpeedRatio(axis, spd_type, spd_ratio);
				g_MoveStartErrorCode[axis] = MoveStart(axis);
				if (g_MoveStartErrorCode[axis])
				{
					return g_MoveStartErrorCode[axis];
				}
			}
		}
		else
		{
			g_MoveOffset[axis] = 0;
		}
		mask <<= 1;
	}

	return 0;
}

int move_pd_with_speed_ratio_xy_offset(int pd_no, int sel_axis, int spd_type, int spd_ratio, double xoffset, double yoffset)
{
	POINT_DATA pd;
	char axis;
	char mask = 1;

	pd = get_point_data(pd_no);
	pd.x += xoffset;
	pd.y += yoffset;

	for (axis = 0; axis < MAX_AXIS; axis++)
	{
		if ((sel_axis & mask) == mask)
		{
			switch (axis)
			{
			case X_AXIS:
				g_MoveOffset[axis] = (pd.x - get_motor_pos(axis)) / g_MotionParam[axis].m_fScaleFactor;
				break;
			case Y_AXIS:
				g_MoveOffset[axis] = (pd.y - get_motor_pos(axis)) / g_MotionParam[axis].m_fScaleFactor;
				break;
			case Z_AXIS:
				g_MoveOffset[axis] = (pd.z - get_motor_pos(axis)) / g_MotionParam[axis].m_fScaleFactor;
				break;
			}
			// g_MoveOffset[axis] = (pd.x - get_motor_pos(axis)) / g_MotionParam[axis].m_fScaleFactor;

			if (g_MoveOffset[axis] != 0)
			{
				SetMoveOffset(axis, g_MoveOffset[axis]);
				// SetSpeed(axis, spd);
				SetSpeedRatio(axis, spd_type, spd_ratio);
				g_MoveStartErrorCode[axis] = MoveStart(axis);
				if (g_MoveStartErrorCode[axis])
				{
					return g_MoveStartErrorCode[axis];
				}
			}
		}
		else
		{
			g_MoveOffset[axis] = 0;
		}
		mask <<= 1;
	}

	return 0;
}

// 선택된 축에 대해서, dist 거리 만큼 이동 한다
int move_inc(int sel_axis, float dist[], int spd_type, int spd_ratio)
{
	char axis;
	char mask = 1;

	for (axis = 0; axis < MAX_AXIS; axis++)
	{
		if ((sel_axis & mask) == mask)
		{
			g_MoveOffset[axis] = (dist[axis]) / g_MotionParam[axis].m_fScaleFactor;

			if (g_MoveOffset[axis] != 0)
			{
				SetMoveOffset(axis, g_MoveOffset[axis]);
				SetSpeedRatio(axis, spd_type, spd_ratio);
				g_MoveStartErrorCode[axis] = MoveStart(axis);
				if (g_MoveStartErrorCode[axis])
				{
					return g_MoveStartErrorCode[axis];
				}
			}
		}
		else
		{
			g_MoveOffset[axis] = 0;
		}

		mask = mask << 1;
	}

	return 0;
}

// 선택된 축에 대해서, dist 거리 만큼 이동 한다
int move_abs(int sel_axis, float dist[], int spd_type, int spd_ratio)
{
	char axis;
	char mask = 1;

	for (axis = 0; axis < MAX_AXIS; axis++)
	{
		if ((sel_axis & mask) == mask)
		{
			g_MoveOffset[axis] = (dist[axis] - get_motor_pos(axis)) / g_MotionParam[axis].m_fScaleFactor;

			if (g_MoveOffset[axis] != 0)
			{
				SetMoveOffset(axis, g_MoveOffset[axis]);
				SetSpeedRatio(axis, spd_type, spd_ratio);
				g_MoveStartErrorCode[axis] = MoveStart(axis);
				if (g_MoveStartErrorCode[axis])
				{
					return g_MoveStartErrorCode[axis];
				}
			}
		}
		else
		{
			g_MoveOffset[axis] = 0;
		}

		mask = mask << 1;
	}

	return 0;
}

// 선택된 축의 이동이 끝났는지 확인
// return
//	1: done
//	0: moving
char move_done(char sel_axis)
{
	char axis = 0;
	char move_status = 0;
	char flag = 1;
	char mask = 1;

	for (axis = 0; axis < MAX_AXIS; axis++)
	{
		if ((sel_axis & mask) == mask)
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
			}
		}
		mask <<= 1;
	}

	return flag;
}

void ReleaseBreak()
{
	SetDOBit(1, 7, 1);
}

void HoldBreak()
{
	SetDOBit(1, 7, 0);
}

char IsReleaseBreak()
{
	unsigned char b = GetDOBit(1, 7);
	// return b;
	return 1;
}

// Grip 센서 확인
char IsGrip()
{
	int bit_no = get_var(4);
	unsigned char b = GetDIBit(1, bit_no);
	return b;
}

// 플라스크가 있는지 확인
char IsExistFlask()
{
	int bit_no = get_var(5);
	// unsigned char b = GetDIBit(1, 6);
	unsigned char b = GetDIBit(1, bit_no);
	return b;
}

char AsyncWasteOn()
{
	SetDOBit(1, 6, 1);
}

char AsyncWasteOff()
{
	SetDOBit(1, 6, 0);
}

// VAR13 번에 bit 번호를 입력 하고, 그 Bit를 사용한다
// - bit 번호는 0번부터 시작
// - V13 = 0 이면 0번 bit
// - 전장 배선은 1번부터 시작. 16번에 연결 => V13=15 입력
char GetWasteAsyncInput()
{
	char bitno = get_var(13);
	char ch = (bitno / 8) + 1;
	char bit = bitno % 8;

	return GetDIBit(ch, bit);
}

//
char IsGripperGripPosition()
{
	double curr_z = get_motor_pos(Z_AXIS);
	POINT_DATA pd = get_point_data(1);
	char ret = 0;

	if (fabs(pd.z - curr_z) < 2.0)
	{
		ret = 1;
	}

	return ret;
}

// 현재 위치가 Waste Ready 위치이면 1 리턴, 그렇지 않으면 0 리턴
// X(P08), Y(P11)
char IsWasteReadyPos()
{
	double curr_x = get_motor_pos(X_AXIS);
	double curr_y = get_motor_pos(Y_AXIS);
	POINT_DATA pd08 = get_point_data(8);
	POINT_DATA pd11 = get_point_data(11);
	char ret = 0;

	if (fabs(pd08.x - curr_x) < 2.0 &&
		fabs(pd11.y - curr_y) < 2.0)
	{
		ret = 1;
	}

	return ret;
}

char IsYAxisPositionMovable()
{
	if (get_motor_pos(Y_AXIS) < get_var(16))
	{
		return 1;
	}
	return 0;
}