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

char g_PointDataCommandState = CMD_READY;

int  g_OriginNeed[MAX_AXIS] = { 0, 1, 1 };
int  g_OriginCompletedAxes[MAX_AXIS] = { 0, 0, 0 };

static int g_OriginOffset[MAX_AXIS] = { 0, 0, 0 };
static int g_Vars[MAX_VARS] = { 0, };
static POINT_DATA g_PointData[MAX_POINT_DATA] = { 0, };


MOTION_PARAM g_MotionParam[MAX_AXIS] = {
					// R_AXIS
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
						256,			// m_uJogAcel
						0,				// m_nHomeOff
						1,				// m_nMoveDir
						1,				// m_ucEncSign
						1.0,			// m_fLead
						3200,			// m_ucEncPulse
						1.0/3200,		// m_fScaleFactor
						10,				// m_ucOrgSensor
						11,				// m_ucNegLimit
						12,				// m_ucPosLimit
						},
					// X_AXIS
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
						48,				// m_uJogAcel
						-16000,			// m_nHomeOff	// 10mm
						0,				// m_nMoveDir
						0,				// m_ucEncSign
						1.0,			// m_fLead
						3200,			// m_ucEncPulse
						1.0/3200,		// m_fScaleFactor
						10,				// m_ucOrgSensor
						11,				// m_ucNegLimit
						12,				// m_ucPosLimit
						},
					// Y_AXIS
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
						48,				// m_uJogAcel
						16000,			// m_nHomeOff	// 10mm
						0,				// m_nMoveDir	// 1->0
						1,				// m_ucEncSign
						1.0,			// m_fLead
						3200,			// m_ucEncPulse
						1.0/3200,		// m_fScaleFactor
						14,				// m_ucOrgSensor
						11,				// m_ucNegLimit
						12,				// m_ucPosLimit
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
	g_ErrorCode = 0;
	g_MotionCommand = 0;
	g_PointDataCommandState = CMD_READY;
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
		snprintf(str, 63, "EEPROM Data Initialization ...");
		SerialWriteBytes(UART_PORT0, str, strlen(str));

		WriteI2C_1B(0x0000, 0x55);

		save_motion_param();

		snprintf(str, 63, "OK\r\n");
		SerialWriteBytes(UART_PORT0, str, strlen(str));
	}
}

// _MB() 함수 문제로, _1B() 함수를 사용하려고 만들었으나, 제대로 동작 안함 
void save_motion_param_type(int type)
{
	int offset_table[] = {
		0,	// MOTION_PARAM_ORG_SOFT_LIMIT
		0+4, 	// MOTION_PARAM_ORG_DIR
		0+4+1,	// MOTION_PARAM_ORG_NEED
		0+4+1+1,	// MOTION_PARAM_ORG_SPEED
		0+4+1+1+4,	// MOTION_PARAM_ORG_LOW_SPEED
		0+4+1+1+4+4,	// MOTION_PARAM_ACCEL
		0+4+1+1+4+4+4,		// MOTION_PARAM_PTP_SPEED
		0+4+1+1+4+4+4+4,	// MOTION_PARAM_PTP_LOW_SPEED
		0+4+1+1+4+4+4+4+4,
		0+4+1+1+4+4+4+4+4+4,	// MOTION_PARAM_JOG_SPEED
		0+4+1+1+4+4+4+4+4+4+4,	// MOTION_PARAM_JOG_LOW_SPEED
		0+4+1+1+4+4+4+4+4+4+4+4,
		0+4+1+1+4+4+4+4+4+4+4+4+4,
		0+4+1+1+4+4+4+4+4+4+4+4+4+4,	// MOTION_PARAM_PTP_DIR
		0+4+1+1+4+4+4+4+4+4+4+4+4+4+1,	// MOTION_PARAM_ENC_SIGN
		0+4+1+1+4+4+4+4+4+4+4+4+4+4+1+1,	// MOTION_PARAM_LEAD
		0+4+1+1+4+4+4+4+4+4+4+4+4+4+1+1+8,	// MOTION_PARAM_ENC_PULSE
		0+4+1+1+4+4+4+4+4+4+4+4+4+4+1+1+8+4,	// MOTION_PARAM_SCALE_FACTOR
		0+4+1+1+4+4+4+4+4+4+4+4+4+4+1+1+8+4+8,	// MOTION_PARAM_ORG_SENSOR
		0+4+1+1+4+4+4+4+4+4+4+4+4+4+1+1+8+4+8+1,	// MOTION_PARAM_NEG_LIMIT
		0+4+1+1+4+4+4+4+4+4+4+4+4+4+1+1+8+4+8+1+1,	// MOTION_PARAM_POS_LIMIT
		0+4+1+1+4+4+4+4+4+4+4+4+4+4+1+1+8+4+8+1+1+1
	};

	unsigned short pBase = ADDR_EEP_MOTION_PARAM;
	unsigned short pOffset = offset_table[type];
	unsigned short len = offset_table[type+1] - offset_table[type];
	unsigned char* pval = 0;
	int i = 0, j = 0;

	for (i = 0; i<3; i++)
	{
		pBase = ADDR_EEP_MOTION_PARAM + (sizeof(MOTION_PARAM) * i);
		pBase += pOffset;

		pval = get_motion_param_ptr(i, type);	// 저장할 파라메타의 주소 
		if (pval == 0) 
		{
			continue;
		}
		
		for (j = 0; j<len; j++)
		{
			WriteI2C_1B((unsigned short)(pBase+j), *(pval+j));
		}
	}
}

unsigned char* get_motion_param_ptr(int axis, int type)
{
	unsigned char* p = 0;
	int i = axis;

	switch (type)
	{
	case MOTION_PARAM_ORG_SOFT_LIMIT:	p = (unsigned char*)&(g_MotionParam[i].m_uOrgSLimit);	break;
	case MOTION_PARAM_ORG_SPEED:		p = (unsigned char*)&(g_MotionParam[i].m_uOrgVmax);		break;
	case MOTION_PARAM_ORG_LOW_SPEED:	p = (unsigned char*)&(g_MotionParam[i].m_uOrgVmin);		break;
	case MOTION_PARAM_JOG_SPEED:		p = (unsigned char*)&(g_MotionParam[i].m_uJogVmax);		break;
	case MOTION_PARAM_JOG_LOW_SPEED:	p = (unsigned char*)&(g_MotionParam[i].m_uJogVmin);		break;
	case MOTION_PARAM_PTP_SPEED:		p = (unsigned char*)&(g_MotionParam[i].m_uNorVmax);		break;
	case MOTION_PARAM_PTP_LOW_SPEED:	p = (unsigned char*)&(g_MotionParam[i].m_uNorVmin);		break;
	case MOTION_PARAM_ACCEL:			p = (unsigned char*)&(g_MotionParam[i].m_uNorAcel);		break;
	case MOTION_PARAM_ORG_DIR:			p = (unsigned char*)&(g_MotionParam[i].m_ucOrgDir);		break;
	case MOTION_PARAM_PTP_DIR:			p = (unsigned char*)&(g_MotionParam[i].m_ucMoveDir);	break;
	case MOTION_PARAM_ENC_SIGN:			p = (unsigned char*)&(g_MotionParam[i].m_ucEncSign);	break;
	case MOTION_PARAM_ORG_NEED:			p = (unsigned char*)&(g_MotionParam[i].m_ucOrgNeed);	break;
	case MOTION_PARAM_ORG_SENSOR:		p = (unsigned char*)&(g_MotionParam[i].m_ucOrgSensor);	break;
	case MOTION_PARAM_ENC_PULSE:		p = (unsigned char*)&(g_MotionParam[i].m_ucEncPulse);	break;
	case MOTION_PARAM_NEG_LIMIT:		p = (unsigned char*)&(g_MotionParam[i].m_ucNegLimit);	break;
	case MOTION_PARAM_POS_LIMIT:		p = (unsigned char*)&(g_MotionParam[i].m_ucPosLimit);	break;
	default: return 0;
	}

	return p;
}

// bychul2. _MB() 함수 문제로 _1B() 함수 사용 
void save_motion_param()
{
//	WriteI2C_MB(ADDR_EEP_MOTION_PARAM, (unsigned char *)&g_MotionParam, sizeof(g_MotionParam));
	int i = 0;
	int len = sizeof(g_MotionParam);
	unsigned char* p = (unsigned char*)&g_MotionParam;

	for (i = 0; i < len; i++)
	{
		WriteI2C_1B(ADDR_EEP_MOTION_PARAM+i, *p);
		p++;
	}
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
