//
#include <string.h>
#include <stdio.h>
#include "hardware.h"
#include "data.h"
#include "function.h"
#include "defines.h"

#define ADDR_EEP_VAR			0x1400
#define ADDR_EEP_MOTION_PARAM	0x1600
#define ADDR_EEP_POINT_DATA		0x1800

// global data

char g_LastCommand[32] = "";
char g_MotionCommand = 0;
int  g_ErrorCode = 0;
char g_OriginCompleted = 0;
char g_OriginRunning = 0;
char g_Running = 0;
char g_Alarm = 0;
char g_ResponseSend = 0;
int  g_ScanRate = 32;
char g_UserStop = 0;
char g_GoToError = 0;
char g_MoveStartErrorCode[MAX_AXIS] = {0, 0, 0};		// 2021.11.23 bychul2 MoveStart() 함수 리턴값 저장  
int  g_MoveStartErrorLine = 0;

int g_OverRun_Command = 0;								// 2022.10.11 bychul2 OverRun 정보 저장 
int g_OverRun_LimitSensor = 0;
int g_OverRun_AxisNo = 0;
int g_OverRun_LimitCount = 0;

char g_PointDataCommandState = CMD_READY;

int  g_OriginNeed[MAX_AXIS] = { 0, 1, 1 };
int  g_OriginCompletedAxes[MAX_AXIS] = { 0, 0, 0 };

static int g_OriginOffset[MAX_AXIS] = { 0, 0, 0 };
static int g_Vars[MAX_VARS] = { 0, };
static POINT_DATA g_PointData[MAX_POINT_DATA] = { 0, };


MOTION_PARAM g_MotionParam[MAX_AXIS] = {
					// X_AXIS
						{2147483647,	// m_uOrgSLimit
						0,				// m_ucOrgDir
						0,				// m_ucOrgNeed
						5333,           // m_uOrgVmax	// 150RPM
						533,			// m_uOrgVmin	// 10RPM
						48,				// m_uOrgAcel
						26666,			// m_uNorVmax	// 500RPM
						2666,			// m_uNorVmin	// 50RPM
						256,			// m_uNorAcel
						2666,			// m_uJogVmax	// 500RPM
						533,			// m_uJogVmin	// 50RPM
						6555,			// m_uJogAcel
						0,				// m_nHomeOff
						1,				// m_nMoveDir
						1,				// m_ucEncSign
						1.0,			// m_fLead
						3200,			// m_ucEncPulse
						1.0/3200,		// m_fScaleFactor
						10,				// m_ucOrgSensor
						11,				// m_ucNegLimit
						12,				// m_ucPosLimit
						5,				// m_ucHoldTorque
						10,				// m_ucMoveTroque
						//0,0,0,0,0,0,0,
						},
					// Y_AXIS
						{2147483647,	// m_uOrgSLimit
						0,				// m_ucOrgDir
						1,				// m_ucOrgNeed
						5333,           // m_uOrgVmax	// 150RPM
						533,            // m_uOrgVmin	// 10RPM
						48,             // m_uOrgAcel
						26666,			// m_uNorVmax	// 500RPM
						2666,			// m_uNorVmin	// 50RPM
						256,			// m_uNorAcel
						2665,			// m_uJogVmax	// 500RPM
						533,			// m_uJogVmin	// 50RPM
						6555,			// m_uJogAcel
						-16000,			// m_nHomeOff	// 10mm
						0,				// m_nMoveDir
						0,				// m_ucEncSign
						1.0,			// m_fLead
						3200,			// m_ucEncPulse
						1.0/3200,		// m_fScaleFactor
						10,				// m_ucOrgSensor
						11,				// m_ucNegLimit
						12,				// m_ucPosLimit
						5,				// m_ucHoldTorque
						10,				// m_ucMoveTroque
						},
					// Z_AXIS
						{2147483647,	// m_uOrgSLimit
						1,				// m_ucOrgDir	// 0 -> 1
						1,				// m_ucOrgNeed
						5333,           // m_uOrgVmax	// 100RPM
						533,            // m_uOrgVmin	// 10RPM
						48,             // m_uOrgAcel
						26666,			// m_uNorVmax	// 500RPM
						2666,			// m_uNorVmin	// 50RPM
						256,			// m_uNorAcel
						2665,			// m_uJogVmax	// 500RPM
						533,			// m_uJogVmin	// 50RPM
						6555,			// m_uJogAcel
						16000,			// m_nHomeOff	// 10mm
						0,				// m_nMoveDir	// 1->0
						1,				// m_ucEncSign
						1.0,			// m_fLead
						3200,			// m_ucEncPulse
						1.0/3200,		// m_fScaleFactor
						14,				// m_ucOrgSensor
						11,				// m_ucNegLimit
						12,				// m_ucPosLimit
						5,				// m_ucHoldTorque
						10,				// m_ucMoveTroque
						}
						};

MOVE_VARIABLE MovVar[MAX_AXIS];


char is_system_setting_var(int no);

char is_system_setting_var(int no)
{
	char res = 0;
	
	if (no > 10 && no <= 90)
	{
		res = 1;
	}
	
	return res;
}

// global data interface

char get_cmd_state()
{
	return g_MotionCommand;
}

int  get_current_error()
{
	return g_ErrorCode;
}

void set_cmd_state(int state)
{
	if (g_MotionCommand != state)
	{
		g_MotionCommand = state;
	}
}

void set_error(int err)
{
	g_ErrorCode = err;
}

void clear_error()
{
	char i = 0;

	g_ErrorCode = 0;
	g_MotionCommand = 0;
	g_PointDataCommandState = CMD_READY;

	g_OverRun_Command = 0;
	g_OverRun_AxisNo = 0;
	g_OverRun_LimitSensor = 0;
	g_OverRun_LimitCount = 0;

	g_MoveStartErrorLine = 0;
	for (i = 0; i<MAX_AXIS; i++)
	{
		g_MoveStartErrorCode[i] = 0;
		if (DriverErrorCheck(i) != 0) 
		{
			DriverReset(i);
		}
	}
}

int get_org_offset(int axis)
{
	int offset = 0;
	
	if (axis >= 0 && axis < MAX_AXIS)
	{
		offset = g_OriginOffset[axis];
	}
	return offset;
}

int set_org_offset(int axis, int counter)
{
	int prevOffset = 0;
	if (axis >= 0 && axis < MAX_AXIS)
	{
		prevOffset = g_OriginOffset[axis];
		g_OriginOffset[axis] = counter;
	}
	
	return prevOffset;
}

int get_var(int no)
{
	if (no > 0 && no <= MAX_VARS)
	{
		return g_Vars[no-1];
	}
	
	return 0;
}

// bychul2. VAR 저장 함수를 수정
// 변경되는 VAR만 eeprom에 저장 
int set_var(int no, int val)
{
	int oldVal = 0;
	int var_size = sizeof(int);
	unsigned short pAddr = 0;
	unsigned char *p = 0; 
	int i;
	
	if (no > 0 && no <= MAX_VARS)
	{
		oldVal = g_Vars[no-1];
		if (oldVal != val)
		{
			pAddr = ADDR_EEP_VAR + ((no - 1) * var_size);
			g_Vars[no-1] = val;
			p = (unsigned char*)&(g_Vars[no-1]);
			if (1); //is_system_setting_var(no))
			{
				// save_var();
				for (i = 0; i<var_size; i++)
				{
					WriteI2C_1B(pAddr+i, *p);
					p++;
				}
			}
		}
	}
	
	return oldVal;
}

POINT_DATA get_point_data(int no)
{
	POINT_DATA pd = { 0, };

	if (no > 0 && no <= MAX_POINT_DATA)
	{
		pd = g_PointData[no-1];
	}
	
	return pd;
}

POINT_DATA set_point_data_from(int no, double x, double y, double z)
{
	POINT_DATA pd;
	pd.x = x;
	pd.y = y;
	pd.z = z;

	return set_point_data(no, pd);
}

POINT_DATA set_point_data(int no, POINT_DATA pd)
{
	POINT_DATA oldPd = { 0, };

	int i = 0;
	int pdDataLen = sizeof(POINT_DATA);
	unsigned short pdAddr = ADDR_EEP_POINT_DATA + (no-1) * pdDataLen;
	unsigned char* ppd = (unsigned char*)&pd;

	if (no > 0 && no <= MAX_POINT_DATA)
	{
		oldPd = g_PointData[no-1];
		g_PointData[no-1] = pd;
		
		for (i = 0; i < pdDataLen; ++i)
		{
			WriteI2C_1B(pdAddr + i, *(ppd+i));
		}
	}
	
	debugf("set_point_data(%d): %7.2f, %7.2f, %7.2f", no, pd.x, pd.y, pd.z);
	
	return oldPd;
}

MOTION_PARAM get_motion_param(int axis)
{
	MOTION_PARAM param = { 0, };

	if (axis >= 0 && axis < MAX_AXIS)
	{
		param = g_MotionParam[axis];
	}
	
	return param;
}

MOTION_PARAM set_motion_param(int axis, MOTION_PARAM param)
{
	MOTION_PARAM oldParam = { 0, };

	if (axis >= 0 && axis < MAX_AXIS)
	{
		oldParam = g_MotionParam[axis];
		g_MotionParam[axis] = param;
	}
	
	return oldParam;
}

// save/load eeprom
void init_eeprom()
{
	char str[32] = "";
	
	if (ReadI2C_1B(0x0000) == 0x55)
	{
		load_motion_param();
	}
	else
	{
		sprintf(str, "EEPROM Data Initialization ...");
		SerialWriteBytes(UART_PORT0, str, strlen(str));

		WriteI2C_1B(0x0000, 0x55);

		save_motion_param();

		sprintf(str, "OK\r\n");
		SerialWriteBytes(UART_PORT0, str, strlen(str));
	}
}

// bychul2. _MB() 함수 문제로 _1B() 함수 사용 
// 라이브러리 수정 후 원상복귀 후 테스트 => 저장 안되서 byte 함수로 복구 
// 라이브러리 두번째 수정 버전은 정상 동작 확인  
void save_motion_param()
{
	WriteI2C_MB(ADDR_EEP_MOTION_PARAM, (unsigned char *)&g_MotionParam, sizeof(g_MotionParam));
/*
	int i = 0;
	int len = sizeof(g_MotionParam);
	unsigned char* p = (unsigned char*)&g_MotionParam;

	for (i = 0; i < len; i++)
	{
		WriteI2C_1B(ADDR_EEP_MOTION_PARAM+i, *p);
		p++;
	}
	*/
}

void load_motion_param()
{
	ReadI2C_MB(ADDR_EEP_MOTION_PARAM, (unsigned char *)&g_MotionParam, sizeof(g_MotionParam));
}

void save_point_data()
{
	WriteI2C_MB(ADDR_EEP_POINT_DATA, (unsigned char *)&g_PointData, sizeof(g_PointData));
}

void load_point_data()
{
	ReadI2C_MB(ADDR_EEP_POINT_DATA, (unsigned char *)&g_PointData, sizeof(g_PointData));
}

void save_var()
{
//	WriteI2C_MB(ADDR_EEP_VAR, (unsigned char *)&g_Vars, sizeof(g_Vars));
	int len = sizeof(g_Vars);
	int i = 0;
	unsigned char *p = (unsigned char*)g_Vars;
	for (i = 0; i<len; i++)
	{
		WriteI2C_1B(ADDR_EEP_VAR+i, *p);
		p++;
	}
}

void load_var()
{
	ReadI2C_MB(ADDR_EEP_VAR, (unsigned char *)&g_Vars, sizeof(g_Vars));
}

void save_all()
{
	save_motion_param();
	save_point_data();
	save_var();
}

void load_all()
{
	load_motion_param();
	load_point_data();
	load_var();
}

void init_data()
{
	int temp = 0;

	// reset global variables	
	g_MotionCommand = 0;
	g_Running = 0;
	g_Alarm = 0;
	
	debugf("Sensor Type = %d", get_var(VAR_SENSOR_TYPE));
	debugf("Wafer Size = %d", get_var(VAR_WAFER_SIZE));
	debugf("Wafer Type = %d", get_var(VAR_WAFER_TYPE));
	debugf("Aligner Type = %d", get_var(VAR_ALIGNER_TYPE));
	

	// set sensor type
	temp = get_var(VAR_SENSOR_TYPE);
	
	if (temp < SST_CCD || temp > SST_LASER_KEYENCE)
	{
		temp = SST_CCD;
	}
	if (temp != SST_CCD)
	{
		temp = SST_LASER;
	}

}

void reset_motion_param()
{
	int axis = 0;
	int i = 0;

	for (axis = 0; axis <= MAX_AXIS; axis++)
	{
		g_MotionParam[axis].m_uOrgSLimit 	= 2147483647;	//00 4(4)���� ������ ����Ʈ ����Ʈ (���� : pulse, ���� : 0 ~ 2147483647)
		g_MotionParam[axis].m_ucOrgDir 		= 1;			//01 5(1)���� ������ ã�ư� ���� (0 : CW, 1 : CCW)
		g_MotionParam[axis].m_ucOrgNeed 	= 1;			//02 6(1)���� ���� �ʿ� ����
		g_MotionParam[axis].m_uOrgVmax 		= 5333;			//03 10(4)���� ������ �ִ� �ӵ� (���� : pps, ���� : 0 ~ 524287)
		g_MotionParam[axis].m_uOrgVmin 		= 533;			//04 14(4)���� ������ ���� �ӵ� (���� : pps, ���� : 0 ~ 32767)
		g_MotionParam[axis].m_uOrgAcel 		= 48;			//05 18(4)���� ������ ��/�� �ӵ� (���� : ?, ���� : 0 ~ 1023)
		g_MotionParam[axis].m_uNorVmax 		= 26666;		//06 22(4)Normal ���۽� �ִ� �ӵ� (���� : pps, ���� : 0 ~ 524287)
		g_MotionParam[axis].m_uNorVmin 		= 2666;			//07 26(4)Normal ���۽� ���� �ӵ� (���� : pps, ���� : 0 ~ 32767)
		g_MotionParam[axis].m_uNorAcel 		= 256;			//08 30(4)Normal ���۽� ��/�� �ӵ� (���� : ?, ���� : 0 ~ 1023)
		g_MotionParam[axis].m_uJogVmax 		= 2665;			//09 34(4)JOG ���۽� �ִ� �ӵ� (���� :pps : 0 ~ 524287)
		g_MotionParam[axis].m_uJogVmin 		= 533;			//10 38(4)JOG ���۽� ���� �ӵ� (���� :pps : 0 ~ 32767)
		g_MotionParam[axis].m_uJogAcel 		= 6555;			//11 42(4)JOG ���۽� ��/�� �ӵ� (���� :pps : 0 ~ 1023)
		g_MotionParam[axis].m_nHomeOff 		= 16000;		//12 46(4)�������� Home ��ġ������ Offset (���� : pulse)
		g_MotionParam[axis].m_ucMoveDir		= 0;			//13 47(1)���� �̵� ����
		g_MotionParam[axis].m_ucEncSign		= 1;			//14 48(1)���ڴ� ��ȣ
		g_MotionParam[axis].m_fLead    		= 1.0;			//15 56(8)Lead
		g_MotionParam[axis].m_ucEncPulse 	= 32000;		//16 60(4)���� 1ȸ�� �޽��� (�⺻ : 3200)
		g_MotionParam[axis].m_fScaleFactor 	= 1.0/32000.0;	//17 68(8)���� Scale Facotr ( = m_fLead/m_ucEncPulse )
		g_MotionParam[axis].m_ucOrgSensor 	= 0;			//18 69(1)Origin Sensor
		g_MotionParam[axis].m_ucNegLimit  	= 0;			//19 70(1)Negative Limit Sensor
		g_MotionParam[axis].m_ucPosLimit  	= 0;			//20 71(1)Positive Limit Sensor
		g_MotionParam[axis].m_ucHoldTorque 	= 5;			//21 72(1)Hold Torque 
		g_MotionParam[axis].m_ucMoveTorque 	= 10;			//22 73(1)Move Torque 
		for (i = 0; i<7; i++) 
		{
			g_MotionParam[axis].m_reversed[i] = 0;	
 		}
	}

	// HMV. 원점복귀 속도 
	g_MotionParam[X_AXIS].m_uOrgVmax = 433;
	g_MotionParam[Y_AXIS].m_uOrgVmax = 200;
	g_MotionParam[Z_AXIS].m_uOrgVmax = 5333;

	// HMB. 원점복귀 속도 
	g_MotionParam[X_AXIS].m_uOrgVmin = 433;
	g_MotionParam[Y_AXIS].m_uOrgVmin = 200;
	g_MotionParam[Z_AXIS].m_uOrgVmin = 5000;

	// VSO. PTP 속도 - High
	g_MotionParam[X_AXIS].m_uNorVmax = 4000; 
	g_MotionParam[Y_AXIS].m_uNorVmax = 1800; 
	g_MotionParam[Z_AXIS].m_uNorVmax = 30000; 

	// VSL. PTP 속도 - Low
	g_MotionParam[X_AXIS].m_uNorVmin = 560; 
	g_MotionParam[Y_AXIS].m_uNorVmin = 180; 
	g_MotionParam[Z_AXIS].m_uNorVmin = 3000; 

	// VSN 4000,1800,30000
	g_MotionParam[X_AXIS].m_uNorVmax = 4000;
	g_MotionParam[Y_AXIS].m_uNorVmax = 1800;
	g_MotionParam[Z_AXIS].m_uNorVmax = 30000;

	// VST. 가감속 
	g_MotionParam[X_AXIS].m_uNorAcel = 1600; 
	g_MotionParam[Y_AXIS].m_uNorAcel = 1600; 
	g_MotionParam[Z_AXIS].m_uNorAcel = 256; 

	// VJO 888,888,1332 
	g_MotionParam[X_AXIS].m_uJogVmax = 888;
	g_MotionParam[Y_AXIS].m_uJogVmax = 888;
	g_MotionParam[Z_AXIS].m_uJogVmax = 1332;

	// VJL 533,533,533 
	g_MotionParam[X_AXIS].m_uJogVmin = 533;
	g_MotionParam[Y_AXIS].m_uJogVmin = 533;
	g_MotionParam[Z_AXIS].m_uJogVmin = 533;

	// VJT 6555,6555, 6555 	
	g_MotionParam[X_AXIS].m_uJogAcel = 6555;
	g_MotionParam[Y_AXIS].m_uJogAcel = 6555;
	g_MotionParam[Z_AXIS].m_uJogAcel = 6555;
	
	// POD. Origin Dir 
	g_MotionParam[X_AXIS].m_ucOrgDir = 0; 
	g_MotionParam[Y_AXIS].m_ucOrgDir = 0; 
	g_MotionParam[Z_AXIS].m_ucOrgDir = 1; 

	// PPD. PPT Dir 
	g_MotionParam[X_AXIS].m_ucMoveDir = 1; 
	g_MotionParam[Y_AXIS].m_ucMoveDir = 1; 
	g_MotionParam[Z_AXIS].m_ucMoveDir = 0; 

	// ENS
	g_MotionParam[X_AXIS].m_ucEncSign = 0; 
	g_MotionParam[Y_AXIS].m_ucEncSign = 0; 
	g_MotionParam[Z_AXIS].m_ucEncSign = 0; 

    /*
	unsigned char	m_ucOrgSensor;	//18 69(1)Origin Sensor
	unsigned char	m_ucNegLimit;	//19 70(1)Negative Limit Sensor
	unsigned char	m_ucPosLimit;	//20 71(1)Positive Limit Sensor
	*/
	
	////////////////////////////////////////
	// GNE 에서 사용하는 모듈의 설정 
	//////////////////////////////////////// 
//	// AHM. 원점 센서 
//	g_MotionParam[X_AXIS].m_ucOrgSensor = 8;
//	g_MotionParam[Y_AXIS].m_ucOrgSensor = 3;
//	g_MotionParam[Z_AXIS].m_ucOrgSensor = 5;
//
//	// ANL. -Limit 센서 
//	g_MotionParam[X_AXIS].m_ucNegLimit = 8;
//	g_MotionParam[Y_AXIS].m_ucNegLimit = 3;
//	g_MotionParam[Z_AXIS].m_ucNegLimit = 5;
//
//	// APL. +Limit 센서 
//	g_MotionParam[X_AXIS].m_ucPosLimit = 9;
//	g_MotionParam[Y_AXIS].m_ucPosLimit = 2;
//	g_MotionParam[Z_AXIS].m_ucPosLimit = 4;

	////////////////////////////////////////
	// Lilly 에서 사용하는 모듈의 설정 
	////////////////////////////////////////
	// HMN 1,1,1 // 원점복귀 사용 

	// AHM. 원점 센서 
	g_MotionParam[X_AXIS].m_ucOrgSensor = 8;
	g_MotionParam[Y_AXIS].m_ucOrgSensor = 2;
	g_MotionParam[Z_AXIS].m_ucOrgSensor = 4;

	// ANL. -Limit 센서 
	g_MotionParam[X_AXIS].m_ucNegLimit = 8;
	g_MotionParam[Y_AXIS].m_ucNegLimit = 2;
	g_MotionParam[Z_AXIS].m_ucNegLimit = 4;

	// APL. +Limit 센서 
	g_MotionParam[X_AXIS].m_ucPosLimit = 7;
	g_MotionParam[Y_AXIS].m_ucPosLimit = 3;
	g_MotionParam[Z_AXIS].m_ucPosLimit = 0;	// 5; Z축 +Limit는 Grip 센서로 사용 

	// ALD. Lead 값 
	g_MotionParam[X_AXIS].m_fLead = 10000.0; 
	g_MotionParam[Y_AXIS].m_fLead = 10000.0; 
	g_MotionParam[Z_AXIS].m_fLead = 1.0; 

	// AEN. Enclode Pulse 
	g_MotionParam[X_AXIS].m_ucEncPulse = 853333; 
	g_MotionParam[Y_AXIS].m_ucEncPulse = 266666; 
	g_MotionParam[Z_AXIS].m_ucEncPulse = 4800; 

	g_MotionParam[X_AXIS].m_fScaleFactor = g_MotionParam[X_AXIS].m_fLead / g_MotionParam[X_AXIS].m_ucEncPulse;
	g_MotionParam[Y_AXIS].m_fScaleFactor = g_MotionParam[Y_AXIS].m_fLead / g_MotionParam[Y_AXIS].m_ucEncPulse;
	g_MotionParam[Z_AXIS].m_fScaleFactor = g_MotionParam[Z_AXIS].m_fLead / g_MotionParam[Z_AXIS].m_ucEncPulse;

	// HTQ. 정지 토크
	g_MotionParam[X_AXIS].m_ucHoldTorque = 25;
	g_MotionParam[Y_AXIS].m_ucHoldTorque = 25;
	g_MotionParam[Z_AXIS].m_ucHoldTorque = 10;

	// MTQ. 이동 토크 
	g_MotionParam[X_AXIS].m_ucMoveTorque = 50;
	g_MotionParam[Y_AXIS].m_ucMoveTorque = 50;
	g_MotionParam[Z_AXIS].m_ucMoveTorque = 30;

	save_motion_param();
}

void reset_point_data()
{
	// Grip 
	set_point_data_from(1, 0.0, 0.0, 10.0);
	// Ungrip 
	set_point_data_from(2, 0.0, 0.0, 1.0);
	// Load 
	set_point_data_from(3, 90.0, 1.0, 0.0);
	// Aspirate
	set_point_data_from(4, 10.0, 15.0, 0.0);
	// Disp
	set_point_data_from(5, 10.0, 1.0, 0.0);
	// Shake 
	set_point_data_from(6, 90.0, 1.0, 0.0);
	// None 
	set_point_data_from(7, 1.0, 1.0, 0.0);
	// Waste 
	set_point_data_from(8, 1.0, 170.0, 0.0);
	// Separate EQIL 
	set_point_data_from(9, 1.0, 1.0, 0.0);
	// Separate EQIS
	set_point_data_from(10, 1.0, 90.0, 0.0);
	// Async Waste Output pos MWRD 
	set_point_data_from(11, 1.0, 9.0, 0.0);
	// Origin Offset 
	set_point_data_from(12, 0.5, 0.2, 0.2);
}

void reset_system_var()
{
	// Var
	set_var(1, 10);
	set_var(2, 0);
	set_var(3, 0);
	set_var(4, 5);
	set_var(5, 6);
	set_var(6, 0);
	set_var(7, 200);
	set_var(8, 3000);
	set_var(9, 5000);
	set_var(10, 500);
	set_var(11, 0);
	set_var(12, 0);
	set_var(13, 15);
	set_var(14, 5000);
	set_var(15, 1);
	set_var(91, 0);

	set_var(16, 0);
	set_var(17, 0);
	set_var(18, 0);
	set_var(19, 0);
	set_var(20, 0);
}
