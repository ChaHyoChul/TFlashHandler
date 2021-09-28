// 

#include "function.h"
#include "hardware.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdarg.h>
#include "data.h"
#include "defines.h"
#include "control.h"

extern MOTION_PARAM g_MotionParam[MAX_AXIS];

void debug(char* str)
{
	if (str == 0)
	{
		return;
	}
	
	if (get_var(VAR_DEBUG) == 1)
	{
		SerialWriteBytes(UART_PORT0, str, strlen(str));
		SerialWriteBytes(UART_PORT0, "\r\n", 2);
	}
}

void debugf(char* format, ...)
{
	static char buffer[128] = "";
	
	va_list args;
	va_start(args, format);
	vsnprintf(buffer, 128, format, args);
	va_end(args);
	
	debug(buffer);
}

char* strnosp(char* str)
{
	int len = 0;
	int i = 0;
	
	if (str == 0)
	{
		return 0;
	}
	
	//len = strlen(str);
	
	while (*(str) != '\0' || *(str) == 0)
	{
		if (isspace(*str))
		{
			++str;
		}
		else
		{
			break;
		}
	}
	
	return str;
}

char* int_to_binary(int i)
{
	static char s[32 + 1] = { '0', };
	int count = 32;

	do
	{
		s[--count] = '0' + (char) (i & 1);
		i = i >> 1;
	} while (count && i != 0);

	s[count] = 0;

	return s;
}

// specific function to array size 3
char* int3_to_str(int* values)
{
	static char s[128] = "";

	sprintf(s, "%d,%d,%d", values[0], values[1], values[2]);
	return s;
}

char* double3_to_str(double* values)
{
	static char s[128] = "";
	
	snprintf(s, 127, "%.2f,%.2f,%.2f", values[0], values[1], values[2]);
	return s;
}

INTS str_to_ints(char* str)
{
	int max_no = 3;
	int i = 0;
	char sp = ',';
	char* s2 = 0;
	INTS ints;
	
	ints.flag[0] = 0;
	ints.flag[1] = 0;
	ints.flag[2] = 0;
	
	for (; i < max_no; ++i)
	{
		str = strnosp(str);
		if (str == 0)
		{
			return ints;
		}
		
		if (strlen(str) == 0)
		{
			continue;
		}
		
		s2 = strchr(str, sp);
		if (str == s2)
		{
			ints.flag[i] = 0;
		}
		else
		{
			ints.flag[i] = 1;
			ints.val[i] = atoi(str);
		}
		
		if (s2 == 0)
		{
			break;
		}
		else
		{
			str = s2 + 1;
		}
	}
	
	return ints;
	
}

char* ints_to_str(INTS ints)
{
	static char buf[64] = "";
	int index = 0;

	if (ints.flag[0]) snprintf(buf, 63, "%d", ints.val[0]);
	index = strlen(buf);
	buf[index++] = ',';
	if (ints.flag[1]) snprintf((buf+index), 63-index, "%d", ints.val[1]);
	index = strlen(buf);
	buf[index++] = ',';
	if (ints.flag[2]) snprintf((buf+index), 63-index, "%d", ints.val[2]);

	return buf;
}

DOUBLES str_to_doubles(char* str)
{
	int max_no = 3;
	int i = 0;
	char sp = ',';
	char* s2 = 0;
	DOUBLES dbls;
	
	dbls.flag[0] = 0;
	dbls.flag[1] = 0;
	dbls.flag[2] = 0;
	
	for (; i < max_no; ++i)
	{
		str = strnosp(str);
		if (str == 0)
		{
			return dbls;
		}
		
		if (strlen(str) == 0)
		{
			continue;
		}

		s2 = strchr(str, sp);
		if (str == s2)
		{
			dbls.flag[i] = 0;
		}
		else
		{
			dbls.flag[i] = 1;
			dbls.val[i] = my_atof(str);
		}
		
		if (s2 == 0)
		{
			break;
		}
		else
		{
			str = s2 + 1;
		}
	}
	
	return dbls;
}

char* doubles_to_str(DOUBLES dbls)
{
	static char buf[64] = "";
	int index = 0;

	if (dbls.flag[0]) snprintf(buf, 63, "%.3f", dbls.val[0]);
	index = strlen(buf);
	buf[index++] = ',';
	if (dbls.flag[1]) snprintf((buf+index), 63-index, "%.3f", dbls.val[1]);
	index = strlen(buf);
	buf[index++] = ',';
	if (dbls.flag[2]) snprintf((buf+index), 63-index, "%.3f", dbls.val[2]);

	return buf;
}

int get_axis_sensor(int axis)
{
	int sensor = 0;

	if (POS_LIMIT(axis) == SENS_ON)
	{
		sensor += 1;
	}
	if (NEG_LIMIT(axis) == SENS_ON)
	{
		sensor += 2;
	}
	
	return sensor;
}

char* get_axes_sensor()
{
	static int val[MAX_AXIS] = { 0, 0, 0 };
	
	val[0] = get_axis_sensor(0);
	val[1] = get_axis_sensor(1);
	val[2] = get_axis_sensor(2);
	
	return int3_to_str(val);
}

char* get_motors_pos()
{
	static int val[MAX_AXIS] = { 0, 0, 0 };
	char i = 0;
	
	for (i = 0; i < MAX_AXIS; ++i)
	{
		val[i] = CounterRead(i);
		if (g_MotionParam[i].m_ucEncSign == 1)
		{
			val[i] = -val[i];
		}
	}
	
	return int3_to_str(val);
}

int get_motor_pulse(char axis)
{
	static int val = 0;
	
	if (axis < 0 || axis >= MAX_AXIS)
	{
		return 0;
	}

	val = CounterRead(axis);
	if (g_MotionParam[axis].m_ucEncSign == 1)
	{
		val = -val;
	}
	
	return val;
}

double get_motor_pos(char axis)
{
	static double val = 0.0;
	
	if (axis < 0 || axis >= MAX_AXIS)
	{
		return 0.0;
	}
	
	val = g_MotionParam[axis].m_fScaleFactor * get_motor_pulse(axis);

	return val;
}


void send(char* str)
{
	if (str == 0 || strlen(str) == 0)
	{
		return;
	}

	SerialWriteBytes(UART_PORT0, str, strlen(str));
}

void sendf(char* format, ...)
{
	static char buffer[128] = "";
	
	va_list args;
	va_start(args, format);
	vsnprintf(buffer, 128, format, args);
	va_end(args);
	
	send(buffer);
}

void send_dbg(char* format, ...)
{
	static char buffer[128] = "";
	
	va_list args;
	va_start(args, format);
	vsnprintf(buffer, 128, format, args);
	va_end(args);
	
	SerialWriteBytes(UART_PORT0, buffer, strlen(buffer));
}

INTS get_param(int type)
{
	static INTS ints = { 0, };
	int i = 0;
	
	for (i = 0; i < MAX_AXIS; ++i)
	{
		ints.flag[i] = 1;

		switch (type)
		{
			case MOTION_PARAM_ORG_SOFT_LIMIT:	ints.val[i] = g_MotionParam[i].m_uOrgSLimit;	break;
			case MOTION_PARAM_ORG_SPEED:	ints.val[i] = g_MotionParam[i].m_uOrgVmax;	break;
			case MOTION_PARAM_ORG_LOW_SPEED:ints.val[i] = g_MotionParam[i].m_uOrgVmin;	break;
			case MOTION_PARAM_JOG_SPEED:	ints.val[i] = g_MotionParam[i].m_uJogVmax;	break;
			case MOTION_PARAM_JOG_LOW_SPEED:ints.val[i] = g_MotionParam[i].m_uJogVmin;	break;
			case MOTION_PARAM_PTP_SPEED:	ints.val[i] = g_MotionParam[i].m_uNorVmax;	break;
			case MOTION_PARAM_PTP_LOW_SPEED:ints.val[i] = g_MotionParam[i].m_uNorVmin;	break;
			case MOTION_PARAM_ACCEL:		ints.val[i] = g_MotionParam[i].m_uNorAcel;	break;
			case MOTION_PARAM_ORG_DIR:		ints.val[i] = g_MotionParam[i].m_ucOrgDir;	break;
			case MOTION_PARAM_PTP_DIR:		ints.val[i] = g_MotionParam[i].m_ucMoveDir;	break;
			case MOTION_PARAM_ENC_SIGN:		ints.val[i] = g_MotionParam[i].m_ucEncSign;	break;
			case MOTION_PARAM_ORG_NEED:		ints.val[i] = g_MotionParam[i].m_ucOrgNeed;	break;
			case MOTION_PARAM_ORG_SENSOR:	ints.val[i] = g_MotionParam[i].m_ucOrgSensor;	break;
			case MOTION_PARAM_ENC_PULSE:	ints.val[i] = g_MotionParam[i].m_ucEncPulse;	break;
			case MOTION_PARAM_NEG_LIMIT:	ints.val[i] = g_MotionParam[i].m_ucNegLimit;	break;
			case MOTION_PARAM_POS_LIMIT:	ints.val[i] = g_MotionParam[i].m_ucPosLimit;	break;
			case MOTION_PARAM_HOLD_TROQUE:	ints.val[i] = g_MotionParam[i].m_ucHoldTorque; 	break;
			case MOTION_PARAM_MOVE_TROQUE:	ints.val[i] = g_MotionParam[i].m_ucMoveTorque; 	break;
			default:
				ints.flag[i] = 0;
		}
	}
	
	return ints;
}

void set_param(int type, INTS ints)
{
	int i = 0;
	
	for (i = 0; i < MAX_AXIS; ++i)
	{
		if (ints.flag[i] == 0)
		{
			continue;
		}

		switch (type)
		{
			case MOTION_PARAM_ORG_SOFT_LIMIT:	g_MotionParam[i].m_uOrgSLimit = ints.val[i];	break;
			case MOTION_PARAM_ORG_SPEED:	g_MotionParam[i].m_uOrgVmax = ints.val[i];	break;
			case MOTION_PARAM_ORG_LOW_SPEED:g_MotionParam[i].m_uOrgVmin = ints.val[i];	break;
			case MOTION_PARAM_JOG_SPEED:	g_MotionParam[i].m_uJogVmax = ints.val[i];	break;
			case MOTION_PARAM_JOG_LOW_SPEED:g_MotionParam[i].m_uJogVmin = ints.val[i];	break;
			case MOTION_PARAM_PTP_SPEED:	g_MotionParam[i].m_uNorVmax = ints.val[i];	break;
			case MOTION_PARAM_PTP_LOW_SPEED:g_MotionParam[i].m_uNorVmin = ints.val[i];	break;
			case MOTION_PARAM_ACCEL:		g_MotionParam[i].m_uNorAcel = ints.val[i];	break;
			case MOTION_PARAM_ORG_DIR:		g_MotionParam[i].m_ucOrgDir = ints.val[i];	break;
			case MOTION_PARAM_PTP_DIR:		g_MotionParam[i].m_ucMoveDir = ints.val[i];	break;
			case MOTION_PARAM_ENC_SIGN:		g_MotionParam[i].m_ucEncSign = ints.val[i];	break;
			case MOTION_PARAM_ORG_NEED:		g_MotionParam[i].m_ucOrgNeed = ints.val[i];	break;
			case MOTION_PARAM_ORG_SENSOR:	g_MotionParam[i].m_ucOrgSensor = ints.val[i];	break;
			case MOTION_PARAM_ENC_PULSE:	g_MotionParam[i].m_ucEncPulse = ints.val[i];	break;
			case MOTION_PARAM_NEG_LIMIT:	g_MotionParam[i].m_ucNegLimit = ints.val[i];	break;
			case MOTION_PARAM_POS_LIMIT:	g_MotionParam[i].m_ucPosLimit = ints.val[i];	break;
			case MOTION_PARAM_HOLD_TROQUE:	g_MotionParam[i].m_ucHoldTorque = ints.val[i]; 	break;
			case MOTION_PARAM_MOVE_TROQUE:	g_MotionParam[i].m_ucMoveTorque = ints.val[i]; 	break;
		}
	}
	
	save_motion_param();
	//save_motion_param_type(type);
}

void set_param_no_save(int type, INTS ints)
{
	int i = 0;
	
	for (i = 0; i < MAX_AXIS; ++i)
	{
		if (ints.flag[i] == 0)
		{
			continue;
		}

		switch (type)
		{
			case MOTION_PARAM_ORG_SOFT_LIMIT:	g_MotionParam[i].m_uOrgSLimit = ints.val[i];	break;
			case MOTION_PARAM_ORG_SPEED:	g_MotionParam[i].m_uOrgVmax = ints.val[i];	break;
			case MOTION_PARAM_ORG_LOW_SPEED:g_MotionParam[i].m_uOrgVmin = ints.val[i];	break;
			case MOTION_PARAM_JOG_SPEED:	g_MotionParam[i].m_uJogVmax = ints.val[i];	break;
			case MOTION_PARAM_JOG_LOW_SPEED:g_MotionParam[i].m_uJogVmin = ints.val[i];	break;
			case MOTION_PARAM_PTP_SPEED:	g_MotionParam[i].m_uNorVmax = ints.val[i];	break;
			case MOTION_PARAM_PTP_LOW_SPEED:g_MotionParam[i].m_uNorVmin = ints.val[i];	break;
			case MOTION_PARAM_ACCEL:		g_MotionParam[i].m_uNorAcel = ints.val[i];	break;
			case MOTION_PARAM_ORG_DIR:		g_MotionParam[i].m_ucOrgDir = ints.val[i];	break;
			case MOTION_PARAM_PTP_DIR:		g_MotionParam[i].m_ucMoveDir = ints.val[i];	break;
			case MOTION_PARAM_ENC_SIGN:		g_MotionParam[i].m_ucEncSign = ints.val[i];	break;
			case MOTION_PARAM_ORG_NEED:		g_MotionParam[i].m_ucOrgNeed = ints.val[i];	break;
			case MOTION_PARAM_ORG_SENSOR:	g_MotionParam[i].m_ucOrgSensor = ints.val[i];	break;
			case MOTION_PARAM_ENC_PULSE:	g_MotionParam[i].m_ucEncPulse = ints.val[i];	break;
			case MOTION_PARAM_NEG_LIMIT:	g_MotionParam[i].m_ucNegLimit = ints.val[i];	break;
			case MOTION_PARAM_POS_LIMIT:	g_MotionParam[i].m_ucPosLimit = ints.val[i];	break;
			case MOTION_PARAM_HOLD_TROQUE:	g_MotionParam[i].m_ucHoldTorque = ints.val[i]; 	break;
			case MOTION_PARAM_MOVE_TROQUE:	g_MotionParam[i].m_ucMoveTorque = ints.val[i];	break;
		}
	}
}

DOUBLES get_param_f(int type)
{
	static DOUBLES dbls = { 0.0, };
	int i = 0;
	
	for (i = 0; i < MAX_AXIS; ++i)
	{
		dbls.flag[i] = 1;

		switch (type)
		{
			case MOTION_PARAM_LEAD:			dbls.val[i] = g_MotionParam[i].m_fLead;	break;
			case MOTION_PARAM_SCALE_FACTOR:	dbls.val[i] = g_MotionParam[i].m_fScaleFactor;	break;
			default:
				dbls.flag[i] = 0;
		}
	}
	
	return dbls;
}

void set_param_f(int type, DOUBLES dbls)
{
	int i = 0;
	
	for (i = 0; i < MAX_AXIS; ++i)
	{
		if (dbls.flag[i] == 0)
		{
			continue;
		}

		switch (type)
		{
			case MOTION_PARAM_LEAD:
				g_MotionParam[i].m_fLead = dbls.val[i];
				g_MotionParam[i].m_fScaleFactor = g_MotionParam[i].m_fLead / g_MotionParam[i].m_ucEncPulse;
				break;
		}
	}
	
	save_motion_param();
}


//
//
//
double my_atof(const char* s)
{
	double dec_val = 0;
	double pot_val = 0;
	double sign = 1;
	double multiple = 1;
	
	while (*s && (*s == ' ' || *s == '\t')) s++;
	if (*s == '\0') return 0;
	
	switch (*s)
	{
	case '-':
		sign = -1;
		s++;
		break;
		
	case '+':
		sign = 1;
		s++;
		break;
	}
	
	while (*s && ((*s >= '0' && *s <= '9') || *s == '.'))
	{
		if (*s == '.')
		{
			s++;
			while (*s && (*s >= '0' && *s <= '9'))
			{
				pot_val = (pot_val * 10) + (*s - '0');
				multiple = 0.1 * multiple;
				s++;
			}
			
			pot_val = pot_val * multiple;
			break;
		}
		else
		{
			dec_val = (dec_val * 10) + (*s - '0');
			s++;
		}
	}
	
	return sign * (dec_val + pot_val);
}

char* cha_strchr(char* str, char c)
{
	char* ret = NULL;

	while (*str != NULL)
	{
		if (*str == c) {
			ret = str;
			break;
		}
		str++;
	}

	return ret;
}

char* cha_strstr(char* str, char* sub)
{
	int len_str = strlen(str);
	int len_sub = strlen(sub);
	int loop_count = len_str - len_sub + 1;
	int i = 0, j = 0, sub_index;
	char find = 0;
	char* ret = 0;

	if (loop_count <= 0) return 0;

	for (i = 0; i<loop_count; i++)
	{
		sub_index = 0;
		find = 1;
		for (j = i; j<i+len_sub; j++)
		{
			if (str[j] != sub[sub_index++])
			{
				find = 0;
				break;
			}
		}
		if (find == 1)
		{
			ret = str + i;
			break;
		}
	}

	return ret;
}

