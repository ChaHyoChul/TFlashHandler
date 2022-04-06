#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


#include "defines.h"
#include "data.h"
#include "hardware.h"
#include "control.h"
#include "function.h"
#include "macro.h"


char str[128];	// �ø��� �۽��� ���� ���ڿ�
char retStr[128];
char g_strtemp[128];

#define IS_COMMAND(a, b)	(strstr((a), (b)) != 0)
#define IS_COMMAND_N(a, b)	((strstr((a), (b)) == a) && (strlen((a)) == strlen((b))))

#define CHECK_COMMAND(A, B)								\
	else if (IS_COMMAND_N(cmd, A))						\
	{													\
														\
		if (! IsStopped())								\
		{												\
			SendResponseRaw(A, ERR_COMMAND_IN_RUNNING);	\
			return;										\
		}												\
														\
		if (IsError())									\
		{												\
			SendResponseRaw(A, ERR_COMMAND_IN_ERROR);	\
			return;										\
		}												\
														\
		if (! IsOriginCompleted())						\
		{												\
			SendResponseRaw(A, ERR_ORIGIN_ERROR);		\
			return;										\
		}												\
														\
		SetCommand(A);									\
		SetControlCommand(B);							\
	}


extern MOTION_PARAM g_MotionParam[MAX_AXIS];
extern MOVE_VARIABLE MovVar[MAX_AXIS];

extern int  g_MovePointDataNo;
extern int  g_MoveOffset[MAX_AXIS];
extern char g_MotionCommand;
extern int  g_ErrorCode;
extern int  g_OriginAxis;

extern POINT_DATA g_PointData[MAX_POINT_DATA];

extern char g_PointDataCommandState;

char g_cMotionComm[MAX_AXIS];


void TimerIsr_1ms()
{
	
}
void DoCmd(char *cmd);
static char szCmd[100];
static char nszCmdOfs = 0;

static char g_bResponse = 0;
static char g_ResponseBuffer[128] = "";

static unsigned long g_LoopTime = 0;
static unsigned long g_CommReceiveTime = 0;


int _tmain(void)
{
	int counter = 0;
	char ch;
	
	unsigned char out_data = 0;
	unsigned char in_data = 0;
	unsigned long timeStart = 0;
	unsigned long timeElapsed = 0;
	unsigned long timeCommStart = 0;
	unsigned long timeCommElapsed = 0;
	
	unsigned long mainControlInterval = 1000;	// ms
	unsigned long systemCheckInterval = 500;	// ms
	unsigned long mainControlIntervalTime = 1;
	unsigned long systemCheckIntervalTime = 1;


	InitHardware();
	UartInit(UART_PORT0, 9600);
	UartInit(UART_PORT1, 9600);
	
	load_all();
	init_data();

	InitAxis();
	
	// Release break
	ReleaseBreak();

//	sprintf(str, "Robots and Design Co., Ltd. - PreAlignerV400\r\n");
//	SerialWriteBytes(UART_PORT0, str, strlen(str));

	for(;;)
	{
		timeStart = GetTicCountUS();
		
		SendCommResponse();
		
		if (g_LoopTime == 0)
		{
			g_LoopTime = 1;
		}
		
		mainControlIntervalTime = mainControlInterval / g_LoopTime;
		systemCheckIntervalTime = systemCheckInterval / g_LoopTime;
		
		if (mainControlIntervalTime == 0)
		{
			mainControlIntervalTime = 200;
		}
		
		if (systemCheckIntervalTime == 0)
		{
			systemCheckIntervalTime = 100;
		}
		
		if ((counter % mainControlIntervalTime) == 0)		// about 1ms interval
		{
			MainControl();
		}
		
		if ((counter % systemCheckIntervalTime) == 0)		// about 0.5ms interval
		{
			SystemCheck();
		}
		
		if (counter++ > 200000)
		{
			counter = 0;
			StatusLedToggle();
		}

		timeCommStart = GetTicCountUS();

		while (PopRcvChar(UART_PORT0, &ch) != 0)
		{
			if (nszCmdOfs == 0)
			{
				if (ch == ' ' || ch == '\r' || ch == '\n')
				{
					continue;
				}
			}
			
			if (ch == '\r')
			{
				szCmd[nszCmdOfs] = '\0';
			}
			else if (ch == '\n' && szCmd[nszCmdOfs] == '\0')
			{
				DoCmd(szCmd);
				ch = 0;
				memset(szCmd, 0, sizeof(szCmd));
				nszCmdOfs = 0;

				timeCommElapsed = GetTicCountUS() - timeCommStart;
				g_CommReceiveTime = (timeCommElapsed + 9*g_CommReceiveTime) / 10;
			}
			else if (ch == '\b')
			{
				if (nszCmdOfs > 0)
				{
					--nszCmdOfs;
				}
			}
			else
			{
				szCmd[nszCmdOfs ++] = ch;
				if (nszCmdOfs >= 100)
				{
					nszCmdOfs = 0;
				}
			}
		}
		
		timeElapsed = GetTicCountUS() - timeStart;
		g_LoopTime = (timeElapsed + 9*g_LoopTime) / 10;
	}
	
	debugf("main terminated");
	
	return 0;
}


void DoCmd(char *cmd)
{
	char res = 0;
	char *ch;
	int i;
	char axis;
	int tmp;
	int var_no = 0;
	INTS ints;
	DOUBLES dbls = { 0, };
	char* comma_pos = 0;
	
	char is_por_comma = 0;
	
	// POR
	char* por_comma_pos = 0;
	int por_pd_no = 0;
	int por_no = 0;
	
	// POS
	char* pos_comma_pos = 0;

	// POF
	char* pof_comma_pos = 0;
	int pof_flag = 0;
	static int pof_pd_no = 0;
	
	static POINT_DATA pd = { 0,	};

	char ch_temp = 0;
	int int_temp = 0;	
	double dbl_temp = 0.0;
	
	unsigned char io_val = 0;
	
	char iot_bit = 0;
	char iot_flag = 0;
	
	char str_temp[32] = "";
	int idi_no = 0;
	
	int jog_flag = 0;
	
	
	int wafer_size = 0;
	int wafer_type = 0;
	
	int motor_pos_pulse[MAX_AXIS] = { 0, };
	double motor_pos[MAX_AXIS] = { 0.0, };
	char sz[32];
	
	char dbg[64];

	if (cmd == 0 || strlen(cmd) == 0)
	{
		return;
	}

	if (IS_COMMAND_N(cmd, "VER"))
	{
		sendf("VER %s, Firmware:%03X\r\n", SOFTWARE_VERSION, FIRMWARE_VER);
	}
	else if (IS_COMMAND_N(cmd, "ACD"))
	{
		sprintf(str, "ACD %s\r\n", get_axes_sensor());
		send(str);
	}
	else if (IS_COMMAND_N(cmd, "ASP"))
	{
		for (i = 0; i < MAX_AXIS; ++i)
		{
			motor_pos[i] = get_motor_pos(i);
		}
		snprintf(str, 127, "ASP %.2f,%.2f,%.2f\r\n", motor_pos[0], motor_pos[1], motor_pos[2]);
		send(str);
	}
	else if (IS_COMMAND_N(cmd, "MPS"))
	{
		for (i = 0; i < MAX_AXIS; ++i)
		{
			motor_pos[i] = get_motor_pos(i);
		}
		
		snprintf(str, 127, "MPS %.2f,%.2f,%.2f\r\n", motor_pos[0], motor_pos[1], motor_pos[2]);
		send(str);
	}
	else if (IS_COMMAND(cmd, "IDI"))
	{
		char temp[64];

		if (strlen(cmd) < 4)
		{
			SetIdiBuffer("IDI E06\r\n");	// no uesed
			return;
		}
		
		ch_temp = atoi(cmd+4);
		
		idi_no = 1;
		comma_pos = strchr(cmd+4, ',');

		if (comma_pos != 0)
		{
			idi_no = atoi(comma_pos + 1);
		}
		
		if (idi_no < 1)
		{
			SetIdiBuffer("IDI E06\r\n");	// 채널 개수 
			return;
		}
		
		io_val = GetInput2(ch_temp);
		sprintf (str, "IDI %02X", io_val);
		
		for (i = 1; i < idi_no; ++i)
		{
			io_val = GetInput2(ch_temp + i);
			sprintf(str_temp, ",%02X", io_val);
			strcat(str, str_temp);
		}
		
		strcat(str, "\r\n");
		
		SetIdiBuffer(str);
	}
	else if (IS_COMMAND(cmd, "IDO"))
	{
		if (strlen(cmd) < 4)
		{
			SetIdoBuffer("IDO E06\r\n");	// no used 
			return;
		}
		
		ch_temp = atoi(cmd+4);
		
		idi_no = 1;
		comma_pos = strchr(cmd+4, ',');
		if (comma_pos != 0)
		{
			idi_no = atoi(comma_pos + 1);
		}
		
		if (idi_no < 1)
		{
			SetIdoBuffer("IDO E06\r\n");	// 채널 개수 
			return;
		}
		
		io_val = GetOutput2(ch_temp);
		sprintf (str, "IDO %02X", io_val);
		
		for (i = 1; i < idi_no; ++i)
		{
			io_val = GetOutput2(ch_temp + i);
			sprintf(str_temp, ",%02X", io_val);
			strcat(str, str_temp);
		}
		
		strcat(str, "\r\n");
		
		SetIdoBuffer(str);
	}
	else if (IS_COMMAND(cmd, "IOT"))
	{
		if (strlen(cmd) < 8)
		{
			send("IOT E06\r\n");
			return;
		}
		
		comma_pos = strchr(cmd+3, '-');
		if (comma_pos == 0)
		{
			send("IOT E06\r\n");
			return;
		}
		
		ch_temp = atoi(cmd+4);
		iot_bit = atoi(comma_pos+1);
		
		comma_pos = strchr(comma_pos+1, ':');
		
		if (comma_pos == 0)
		{
			send("IOT E06\r\n");
			return;
		}
		
		iot_flag = *(comma_pos+1);
		
		switch (iot_flag)
		{
		case '+':
			SetDOBit(ch_temp, iot_bit, 1);
			break;

		case '-':
			SetDOBit(ch_temp, iot_bit, 0);
			break;
			
		default:
			send("IOT E06\r\n");
			return;
		}
		send("IOT\r\n");
	}
	else if (IS_COMMAND(cmd, "ERR"))
	{
		SendResponseRaw("ERR", g_ErrorCode);
	}
	else if (IS_COMMAND(cmd, "DRT"))
	{
		clear_error();
		AsyncWasteOff();
		send("DRT\r\n");
	}
	else if (IS_COMMAND(cmd, "VAR"))
	{
		ch = strstr(cmd, "VAR");
		ch = strchr(ch+3, 'V');  
	
		if (ch == 0)
		{
			send("VAR E06\r\n");
			return;
		}
		else
		{	
			var_no = atoi(ch+1);

			// check var no
			if (var_no < 1 || var_no > MAX_VARS)
			{
				send("VAR E06\r\n");
				return;
			}
			
			ch = strchr(ch+1, '=');
			
			if (ch == 0)
			{
				// get var
				tmp = get_var(var_no);
				sprintf(str, "VAR V%02d=%d\r\n", var_no, tmp);
				send(str);
			}
			else
			{
				// set var
				tmp = atoi(ch+1);
				set_var(var_no, tmp);
				send("VAR\r\n");
			}
		}		
	}
	else if (IS_COMMAND(cmd, "POR"))
	{
		if (! IsStopped())
		{
			SendResponseRaw("POR", ERR_COMMAND_IN_RUNNING);
			return;
		}

		if (IsError())
		{
			SendResponseRaw("POR", ERR_COMMAND_IN_ERROR);
			return;
		}

		// check cmd length
		if (strlen(cmd) < 4)
		{
			send("POR E04\r\n");	// no used 
			return;
		}
		
		por_pd_no = atoi(cmd+4);
		por_no = 1;
		por_comma_pos = strchr(cmd+3, ',');
		if (por_comma_pos != 0)
		{
			por_no = atoi(por_comma_pos+1);
		}
		
		for (i = 0; i < por_no; ++i)
		{
			pd = get_point_data(por_pd_no+i);
			snprintf(str, 127, "POS %.2f,%.2f,%.2f\r\n", pd.x, pd.y, pd.z);
			send(str);
		}
		
		send("POE\r\n");
	}
	else if (IS_COMMAND(cmd, "POF"))
	{
		if (! IsStopped())
		{
			SendResponseRaw("POF", ERR_COMMAND_IN_RUNNING);
			return;
		}

		if (IsError())
		{
			SendResponseRaw("POF", ERR_COMMAND_IN_ERROR);
			return;
		}
	
		// check length
		if (strlen(cmd) < 4)
		{
			send("POF E04\r\n");	// no used 
			g_PointDataCommandState = CMD_READY;
			return;
		}
		
		pof_flag = atoi(cmd+4);
		pof_pd_no = 1;
		pof_comma_pos = strchr(cmd+3, ',');
		
		switch (pof_flag)
		{
		case 1:
			if (g_PointDataCommandState != CMD_READY)
			{
				send("POF E06\r\n");
				g_PointDataCommandState = CMD_READY;
				return;
			}

			if (pof_comma_pos != 0)
			{
				pof_pd_no = atoi(pof_comma_pos+1);
			}
			
			g_PointDataCommandState = CMD_POF_RECV;
			debugf("Point Write Start (%d)", pof_pd_no);
			break;
		
		case 2:
			if (g_PointDataCommandState != CMD_POF_RECV)
			{
				send("POF E05\r\n");
				g_PointDataCommandState = CMD_READY;
				return;
			}
			
			g_PointDataCommandState = CMD_READY;
			if (pof_comma_pos != 0)
			{
				send("POF E04\r\n");
				return;
			}
			
			debug("Point Write End");
			break;
		
		default:
			send("POF E04\r\n");
			g_PointDataCommandState = CMD_READY;
			break;			
		}
		
		send("POF\r\n");
	}
	else if (IS_COMMAND(cmd, "POS"))
	{
		if (g_PointDataCommandState != CMD_POF_RECV)
		{
			g_PointDataCommandState = CMD_READY;
			send("POS E04\r\n");
			return;
		}
		
		strcpy(g_strtemp, cmd);
		debugf("POS0: %s", g_strtemp);
		
		pd = get_point_data(pof_pd_no);
		
		pos_comma_pos = strchr(g_strtemp+3, ',');
		if (pos_comma_pos == 0)
		{
			g_PointDataCommandState = CMD_READY;
			send("POS E04\r\n");
			return;
		}
		
		pd.x = my_atof((char*)(g_strtemp+3));
		pd.y = my_atof((char*)(pos_comma_pos+1));

		pos_comma_pos = strchr(pos_comma_pos+1, ',');
				
		if (pos_comma_pos == 0)
		{
			g_PointDataCommandState = CMD_READY;
			send("POS E04\r\n");
			return;
		}

		pos_comma_pos = pos_comma_pos + 1;
		pd.z = my_atof(pos_comma_pos);
		
		set_point_data(pof_pd_no, pd);
		
		++pof_pd_no;

		send("POS\r\n");
		return;
	}
	else if (IS_COMMAND(cmd, "POM"))
	{
		if (! IsOriginCompleted())
		{
			SendResponseRaw("POM", ERR_ORIGIN_ERROR);
			return;
		}
		
		if (! IsStopped())
		{
			SendResponseRaw("POM", ERR_COMMAND_IN_RUNNING);
			return;
		}

		if (IsError())
		{
			SendResponseRaw("POM", ERR_COMMAND_IN_ERROR);
			return;
		}

		if (strlen(cmd) < 4)
		{
			send("POM E06\r\n");
			return;
		}
		
		int_temp = atoi(cmd+4);
		if (int_temp < 1 || int_temp > 999)
		{
			send("POM E06\r\n");
			return;
		}
		
		pd = get_point_data(int_temp);
		
		// check pd
		// 2021.09.28 얼라이너 전용 같아서 주석 처리 
		/*
		if (fabs(pd.x) > 30.0 || fabs(pd.y) > 30.0)
		{
			send("POM E08\r\n");
			return;
		}*/
		
		g_MoveOffset[0] = (pd.x - get_motor_pos(0)) / g_MotionParam[0].m_fScaleFactor;
		g_MoveOffset[1] = (pd.x - get_motor_pos(1)) / g_MotionParam[1].m_fScaleFactor;
		g_MoveOffset[2] = (pd.x - get_motor_pos(2)) / g_MotionParam[2].m_fScaleFactor;

		SetCommand("POM");
		SetControlCommand(COMM_PTP);
		SetCommand("POM");
	}

/*	else if (IS_COMMAND(cmd, "ORG"))
	{
		if (! IsStopped())
		{
			SendResponseRaw("ORG", ERR_COMMAND_IN_RUNNING);
			return;
		}
		if (IsError())
		{
			SendResponseRaw("ORG", ERR_COMMAND_IN_ERROR);
			return;
		}
		if (!IsReleaseBreak()) 
		{
			send("ORG E11\r\n");
			return ;
		}

		SetCommand("ORG");
		SetControlCommand(COMM_ORIGIN);
		g_ResponseSend = 1;
		send("ORG\r\n");
	}
*/	else if (IS_COMMAND(cmd, "ORA"))
	{
		if (! IsStopped())
		{
			SendResponseRaw("ORA", ERR_COMMAND_IN_RUNNING);
			return;
		}

		if (IsError())
		{
			SendResponseRaw("ORA", ERR_COMMAND_IN_ERROR);
			return;
		}
		if (!IsReleaseBreak()) 
		{
			send("ORA E11\r\n");
			return ;
		}

		if (strlen(cmd) < 4)
		{
			send("ORA E06\r\n");
			return;
		}
		
		g_OriginAxis = (atoi(cmd+3) - 1);
		
		if (g_OriginAxis < 0 || g_OriginAxis >= MAX_AXIS)
		{
			g_OriginAxis = 0;
			send("ORA E06\r\n");
			return;
		}

		SetCommand("ORA");
		SetControlCommand(COMM_ORIGIN_A);
		g_ResponseSend = 1;
		send("ORA\r\n");
	}
	else if (IS_COMMAND(cmd, "JOG"))
	{
		if (!IsReleaseBreak()) 
		{
			send("JOG E11\r\n");
			return ;
		}

		if (strlen(cmd) > 4)
		{
			ch = (strchr(cmd, 'G')) + 2;
			
			switch (*ch)
			{
				case 'X': axis = 0; break;
				case 'Y': axis = 1; break;
				case 'Z': axis = 2; break;
				default:
					sprintf(str, "JOG E06\r\n");
					SerialWriteBytes(UART_PORT0, str, strlen(str));
					return;
			}
			if (strlen(cmd) > 5)
			{
				switch (*(ch+1))
				{
					case '+': tmp = 1; break;
					case '-': tmp = 0; break;
					default:
						sprintf(str, "JOG E06\r\n");
						SerialWriteBytes(UART_PORT0, str, strlen(str));
						return;
				}
				jog_flag = 1;
			}
			else
			{
				jog_flag = 0;
			}
		}
		else
		{
			sprintf(str, "JOG E06\r\n");
			SerialWriteBytes(UART_PORT0, str, strlen(str));
			return;
		}
		
		if (jog_flag == 0)
		{
			JogStop(axis);
		}
		else
		{
			if (! IsStopped())
			{
				SendResponseRaw("JOG", ERR_COMMAND_IN_RUNNING);
				return;
			}
			
			JogStart(axis, (char)tmp);
		}

		sprintf(str, "JOG\r\n");
		SerialWriteBytes(UART_PORT0, str, strlen(str));
	}
	else if (IS_COMMAND(cmd, "LMI"))
	{
		if (! IsStopped())
		{
			SendResponseRaw("LMI", ERR_COMMAND_IN_RUNNING);
			return;
		}
		if (IsError())
		{
			SendResponseRaw("LMI", ERR_COMMAND_IN_ERROR);
			return;
		}
		if (!IsReleaseBreak()) 
		{
			send("LMI E11\r\n");
			return ;
		}

		SetCommand("LMI");
		
		ch = strstr(cmd, "LMI");
		ch = ch + 3;
		ints = str_to_ints(ch);
		dbls = str_to_doubles(ch);
		
		for (i = 0; i < MAX_AXIS; ++i)
		{
			g_MoveOffset[i] = ints.flag[i] ? (int)(dbls.val[i]/g_MotionParam[i].m_fScaleFactor) : 0;	
		}
		
		SetControlCommand(COMM_PTP);
	}
	else if (IS_COMMAND(cmd, "LMA"))
	{		
		if (!IsOriginCompleted())
		{
			SendResponseRaw("LMA", ERR_ORIGIN_ERROR);
			return;
		}
		if (!IsStopped())
		{
			SendResponseRaw("LMA", ERR_COMMAND_IN_RUNNING);
			return;
		}
		if (!IsReleaseBreak()) 
		{
			send("LMA E11\r\n");
			return ;
		}
	
		SetCommand("LMA");
			
		ch = strstr(cmd, "LMA");
		ch = ch + 3;
		ints = str_to_ints(ch);
		dbls = str_to_doubles(ch);
		
		for (i = 0; i < MAX_AXIS; ++i)
		{
			g_MoveOffset[i] = ints.flag[i] ? (int)(dbls.val[i]/g_MotionParam[i].m_fScaleFactor - get_motor_pulse(i)) : 0;
		}
		
		SetControlCommand(COMM_PTP);
		SetCommand("LMA");
	}
	else if (IS_COMMAND(cmd, "MMI"))
	{
		if (! IsStopped())
		{
			SendResponseRaw("MMI", ERR_COMMAND_IN_RUNNING);
			return;
		}
		if (IsError())
		{
			SendResponseRaw("MMI", ERR_COMMAND_IN_ERROR);
			return;
		}
		if (!IsReleaseBreak()) 
		{
			send("MMI E11\r\n");
			return ;
		}

		SetCommand("MMI");
		
		ch = strstr(cmd, "MMI");
		ch = ch + 3;
		ints = str_to_ints(ch);
		dbls = str_to_doubles(ch);
		
		for (i = 0; i < MAX_AXIS; ++i)
		{
			g_MoveOffset[i] = ints.flag[i] ? (int)(dbls.val[i]/g_MotionParam[i].m_fScaleFactor) : 0;	
		}
		
		SetControlCommand(COMM_PTP);
		g_ResponseSend = 1;
		send("MMI\r\n");
	}
	else if (IS_COMMAND(cmd, "MMA"))
	{		
		if (! IsOriginCompleted())
		{
			SendResponseRaw("MMA", ERR_ORIGIN_ERROR);
			return;
		}
		if (! IsStopped())
		{
			SendResponseRaw("MMA", ERR_COMMAND_IN_RUNNING);
			return;
		}
		if (!IsReleaseBreak()) 
		{
			send("MMA E11\r\n");
			return ;
		}

		SetCommand("MMA");
			
		ch = strstr(cmd, "MMA");
		ch = ch + 3;
		ints = str_to_ints(ch);
		dbls = str_to_doubles(ch);
		
		for (i = 0; i < MAX_AXIS; ++i)
		{
			g_MoveOffset[i] = ints.flag[i] ? (int)(dbls.val[i]/g_MotionParam[i].m_fScaleFactor - get_motor_pulse(i)) : 0;
		}
		
		SetControlCommand(COMM_PTP);
		SetCommand("LMA");
		g_ResponseSend = 1;
		send("MMA\r\n");
	}
	else if (IS_COMMAND(cmd, "ASS"))
	{
		MoveStop(0);
		MoveStop(1);
		MoveStop(2);
		
		g_UserStop = 10;

		sprintf(str, "ASS\r\n");
		SerialWriteBytes(UART_PORT0, str, strlen(str));
	}
	else if (IS_COMMAND(cmd, "OSL"))
	{
		ch = strstr(cmd, "OSL");
		ch = strnosp(ch + 3);
		if (strlen(ch) == 0)
		{
			ints = get_param(MOTION_PARAM_ORG_SOFT_LIMIT);
			sprintf(str, "OSL %s\r\n", ints_to_str(ints));	
		}
		else
		{
			ints = str_to_ints(ch);
			set_param(MOTION_PARAM_ORG_SOFT_LIMIT, ints);
			sprintf(str, "OSL\r\n");
		}
		send(str);
	}
	else if (IS_COMMAND(cmd, "HMV"))
	{
		ch = strstr(cmd, "HMV");
		ch = strnosp(ch + 3);
		if (strlen(ch) == 0)
		{
			ints = get_param(MOTION_PARAM_ORG_SPEED);
			sprintf(str, "HMV %s\r\n", ints_to_str(ints));
		}
		else
		{
			ints = str_to_ints(ch);
			set_param(MOTION_PARAM_ORG_SPEED, ints);
			sprintf(str, "HMV\r\n");
		}
		send(str);
	}
	else if (IS_COMMAND(cmd, "HMB"))
	{
		ch = strstr(cmd, "HMB");
		ch = strnosp(ch + 3);
		if (strlen(ch) == 0)
		{
			ints = get_param(MOTION_PARAM_ORG_LOW_SPEED);
			sprintf(str, "HMB %s\r\n", ints_to_str(ints));
		}
		else
		{
			ints = str_to_ints(ch);
			set_param(MOTION_PARAM_ORG_LOW_SPEED, ints);
			sprintf(str, "HMB\r\n");
		}
		send(str);
	}
	else if (IS_COMMAND(cmd, "VSO"))
	{
		ch = strstr(cmd, "VSO");
		ch = strnosp(ch + 3);
		if (strlen(ch) == 0)
		{
			ints = get_param(MOTION_PARAM_PTP_SPEED);
			sprintf(str, "VSO %s\r\n", ints_to_str(ints));
		}
		else
		{
			ints = str_to_ints(ch);
			set_param(MOTION_PARAM_PTP_SPEED, ints);
			sprintf(str, "VSO\r\n");
		}
		send(str);
	}
	else if (IS_COMMAND(cmd, "VSN"))
	{
		ch = strstr(cmd, "VSN");
		ch = strnosp(ch + 3);
		if (strlen(ch) == 0)
		{
			ints = get_param(MOTION_PARAM_PTP_SPEED);
			sprintf(str, "VSN %s\r\n", ints_to_str(ints));
		}
		else
		{
			ints = str_to_ints(ch);
			set_param_no_save(MOTION_PARAM_PTP_SPEED, ints);
			sprintf(str, "VSN\r\n");
		}
		send(str);
	}
	else if (IS_COMMAND(cmd, "VSL"))
	{
		ch = strstr(cmd, "VSL");
		ch = strnosp(ch + 3);
		if (strlen(ch) == 0)
		{
			ints = get_param(MOTION_PARAM_PTP_LOW_SPEED);
			sprintf(str, "VSL %s\r\n", ints_to_str(ints));
		}
		else
		{
			ints = str_to_ints(ch);
			set_param(MOTION_PARAM_PTP_LOW_SPEED, ints);
			sprintf(str, "VSL\r\n");
		}
		send(str);
	}
	else if (IS_COMMAND(cmd, "VJO"))
	{
		ch = strstr(cmd, "VJO");
		ch = strnosp(ch + 3);
		if (strlen(ch) == 0)
		{
			ints = get_param(MOTION_PARAM_JOG_SPEED);
			sprintf(str, "VJO %s\r\n", ints_to_str(ints));
		}
		else
		{
			ints = str_to_ints(ch);
			set_param(MOTION_PARAM_JOG_SPEED, ints);
			sprintf(str, "VJO\r\n");
		}
		send(str);
	}
	else if (IS_COMMAND(cmd, "VJL"))
	{
		ch = strstr(cmd, "VJL");
		ch = strnosp(ch + 3);
		if (strlen(ch) == 0)
		{
			ints = get_param(MOTION_PARAM_JOG_LOW_SPEED);
			sprintf(str, "VJL %s\r\n", ints_to_str(ints));
		}
		else
		{
			ints = str_to_ints(ch);
			set_param(MOTION_PARAM_JOG_LOW_SPEED, ints);
			sprintf(str, "VJL\r\n");
		}
		send(str);
	}
	else if (IS_COMMAND(cmd, "VST"))
	{
		ch = strstr(cmd, "VST");
		ch = strnosp(ch + 3);
		if (strlen(ch) == 0)
		{
			ints = get_param(MOTION_PARAM_ACCEL);
			sprintf(str, "VST %s\r\n", ints_to_str(ints));
		}
		else
		{
			ints = str_to_ints(ch);
			set_param(MOTION_PARAM_ACCEL, ints);
			sprintf(str, "VST\r\n");
		}
		send(str);
	}
	else if (IS_COMMAND(cmd, "VJT")) // bychul2 추가 
	{
		ch = strstr(cmd, "VJT");
		ch = strnosp(ch + 3);
		if (strlen(ch) == 0)
		{
			ints = get_param(MOTION_PARAM_JOG_ACCEL);
			sprintf(str, "VJT %s\r\n", ints_to_str(ints));
		}
		else
		{
			ints = str_to_ints(ch);
			set_param(MOTION_PARAM_JOG_ACCEL, ints);
			sprintf(str, "VJT\r\n");
		}
		send(str);
	}
	else if (IS_COMMAND(cmd, "POD"))
	{
		ch = strstr(cmd, "POD");
		ch = strnosp(ch + 3);
		if (strlen(ch) == 0)
		{
			ints = get_param(MOTION_PARAM_ORG_DIR);
			sprintf(str, "POD %s\r\n", ints_to_str(ints));
		}
		else
		{
			ints = str_to_ints(ch);
			set_param(MOTION_PARAM_ORG_DIR, ints);
			sprintf(str, "POD\r\n");
		}
		send(str);
	}
	else if (IS_COMMAND(cmd, "PPD"))
	{
		ch = strstr(cmd, "PPD");
		ch = strnosp(ch + 3);
		if (strlen(ch) == 0)
		{
			ints = get_param(MOTION_PARAM_PTP_DIR);
			sprintf(str, "PPD %s\r\n", ints_to_str(ints));
		}
		else
		{
			ints = str_to_ints(ch);
			set_param(MOTION_PARAM_PTP_DIR, ints);
			sprintf(str, "PPD\r\n");
		}
		send(str);
	}
	else if (IS_COMMAND(cmd, "ENS"))
	{
		ch = strstr(cmd, "ENS");
		ch = strnosp(ch + 3);
		if (strlen(ch) == 0)
		{
			ints = get_param(MOTION_PARAM_ENC_SIGN);
			sprintf(str, "ENS %s\r\n", ints_to_str(ints));
		}
		else
		{
			ints = str_to_ints(ch);
			set_param(MOTION_PARAM_ENC_SIGN, ints);
			sprintf(str, "ENS\r\n");
		}
		send(str);
	}
	else if (IS_COMMAND(cmd, "HMN"))
	{
		ch = strstr(cmd, "HMN");
		ch = strnosp(ch + 3);
		if (strlen(ch) == 0)
		{
			ints = get_param(MOTION_PARAM_ORG_NEED);
			sprintf(str, "HMN %s\r\n", ints_to_str(ints));
		}
		else
		{
			ints = str_to_ints(ch);
			set_param(MOTION_PARAM_ORG_NEED, ints);
			sprintf(str, "HMN\r\n");
		}
		send(str);
	}
	else if (IS_COMMAND(cmd, "AHM"))
	{
		ch = strstr(cmd, "AHM");
		ch = strnosp(ch + 3);
		if (strlen(ch) == 0)
		{
			ints = get_param(MOTION_PARAM_ORG_SENSOR);
			sprintf(str, "AHM %s\r\n", ints_to_str(ints));
		}
		else
		{
			ints = str_to_ints(ch);
			set_param(MOTION_PARAM_ORG_SENSOR, ints);
			sprintf(str, "AHM\r\n");
		}
		send(str);
	}
	else if (IS_COMMAND(cmd, "ANL"))
	{
		ch = strstr(cmd, "ANL");
		ch = strnosp(ch + 3);
		if (strlen(ch) == 0)
		{
			ints = get_param(MOTION_PARAM_NEG_LIMIT);
			sprintf(str, "ANL %s\r\n", ints_to_str(ints));
		}
		else
		{
			ints = str_to_ints(ch);
			set_param(MOTION_PARAM_NEG_LIMIT, ints);
			sprintf(str, "ANL\r\n");
		}
		send(str);
	}
	else if (IS_COMMAND(cmd, "APL"))
	{
		ch = strstr(cmd, "APL");
		ch = strnosp(ch + 3);
		if (strlen(ch) == 0)
		{
			ints = get_param(MOTION_PARAM_POS_LIMIT);
			sprintf(str, "APL %s\r\n", ints_to_str(ints));
		}
		else
		{
			ints = str_to_ints(ch);
			set_param(MOTION_PARAM_POS_LIMIT, ints);
			sprintf(str, "APL\r\n");
		}
		send(str);
	}
	else if (IS_COMMAND(cmd, "ALD"))
	{
		ch = strstr(cmd, "ALD");
		ch = strnosp(ch + 3);
		if (strlen(ch) == 0)
		{
			dbls = get_param_f(MOTION_PARAM_LEAD);
			sprintf(str, "ALD %s\r\n", doubles_to_str(dbls));
		}
		else
		{
			dbls = str_to_doubles(ch);
			set_param_f(MOTION_PARAM_LEAD, dbls);
			sprintf(str, "ALD\r\n");
		}
		send(str);
	}
	else if (IS_COMMAND(cmd, "AEN"))
	{
		ch = strstr(cmd, "AEN");
		ch = strnosp(ch + 3);
		if (strlen(ch) == 0)
		{
			ints = get_param(MOTION_PARAM_ENC_PULSE);
			sprintf(str, "AEN %s\r\n", ints_to_str(ints));
		}
		else
		{
			ints = str_to_ints(ch);
			set_param(MOTION_PARAM_ENC_PULSE, ints);
			sprintf(str, "AEN\r\n");
		}
		send(str);
	}
	else if (IS_COMMAND(cmd, "ASF"))
	{
		ch = strstr(cmd, "ASF");
		ch = strnosp(ch + 3);
		if (strlen(ch) == 0)
		{
			dbls = get_param_f(MOTION_PARAM_SCALE_FACTOR);
			sprintf(str, "ASF %s\r\n", doubles_to_str(dbls));
		}
		else
		{
			ints = str_to_ints(ch);
			set_param_f(MOTION_PARAM_SCALE_FACTOR, dbls);
			sprintf(str, "ASF\r\n");
		}
		send(str);
	}
	else if (IS_COMMAND(cmd, "HTQ"))
	{
		ch = strstr(cmd, "HTQ");
		ch = strnosp(ch + 3);
		if (strlen(ch) == 0)
		{
			ints = get_param(MOTION_PARAM_HOLD_TROQUE);
			sprintf(str, "HTQ %s\r\n", ints_to_str(ints));
		}
		else 
		{
			ints = str_to_ints(ch);
			set_param(MOTION_PARAM_HOLD_TROQUE, ints);
			sprintf(str, "HTQ\r\n");
		}
		send(str);
	} 
	else if (IS_COMMAND(cmd, "MTQ"))
	{
		ch = strstr(cmd, "MTQ");
		ch = strnosp(ch + 3);
		if (strlen(ch) == 0)
		{
			ints = get_param(MOTION_PARAM_MOVE_TROQUE);
			sprintf(str, "MTQ %s\r\n", ints_to_str(ints));
		}
		else 
		{
			ints = str_to_ints(ch);
			set_param(MOTION_PARAM_MOVE_TROQUE, ints);
			sprintf(str, "MTQ\r\n");
		}
		send(str);
	} 
	else if (IS_COMMAND(cmd, "RBK"))
	{
		ReleaseBreak();
		send("RBK\r\n");
	}
	else if (IS_COMMAND(cmd, "HBK"))
	{
		HoldBreak();
		send("HBK\r\n");
	}
	/////////////////////////////////////////////////////////
	// T-Flask Command 
	else if (IS_COMMAND_N(cmd, "MRST"))	// Motion Param Reset 
	{
		if (! IsStopped())
		{
			SendResponseRaw("MRST", ERR_COMMAND_IN_RUNNING);
			return;
		}

		if (IsError())
		{
			SendResponseRaw("MRST", ERR_COMMAND_IN_ERROR);
			return;
		}

		reset_motion_param();

		send("MRST\r\n");
	}
	else if (IS_COMMAND_N(cmd, "HOME"))	// 원점복귀 
	{
		if (! IsStopped())
		{
			SendResponseRaw("HOME", ERR_COMMAND_IN_RUNNING);
			return;
		}
		if (IsError())
		{
			SendResponseRaw("HOME", ERR_COMMAND_IN_ERROR);
			return;
		}
		if (!IsReleaseBreak()) 
		{
			send("HOME E11\r\n");
			return ;
		}

		SetCommand("HOME");
		SetControlCommand(COMM_HOME);
		g_ResponseSend = 1;
		send("HOME\r\n");
	}
	else if (IS_COMMAND_N(cmd, "HOMF"))	// 원점복귀 other version 
	{
		if (! IsStopped())
		{
			SendResponseRaw("HOMF", ERR_COMMAND_IN_RUNNING);
			return;
		}
		if (IsError())
		{
			SendResponseRaw("HOMF", ERR_COMMAND_IN_ERROR);
			return;
		}
		if (!IsReleaseBreak()) 
		{
			send("HOMF E11\r\n");
			return ;
		}

		SetCommand("HOMF");
		SetControlCommand(COMM_HOMF);
		g_ResponseSend = 1;
		send("HOMF\r\n");
	}
	else if (IS_COMMAND_N(cmd, "MGRI"))	// Grip
	{
		if (!IsOriginCompleted())
		{
			SendResponseRaw("MGRI", ERR_ORIGIN_ERROR);
			return ;
		}
		if (!IsStopped()) {
			SendResponseRaw("MGRI", ERR_COMMAND_IN_RUNNING);
			return ;
		}
		if (IsError()) {
			SendResponseRaw("MGRI", ERR_COMMAND_IN_ERROR);
			return ;
		}

		g_MovePointDataNo = 1;	// 사용할 POINT_NO 저장 
		SetCommand("MGRI");
		SetControlCommand(COMM_MGRI);
		g_ResponseSend = 1;
		send("MGRI\r\n");
	}
	else if (IS_COMMAND_N(cmd, "MUNG"))	// UnGrip
	{
		if (!IsOriginCompleted())
		{
			SendResponseRaw("MUNG", ERR_ORIGIN_ERROR);
			return ;
		}
		if (!IsStopped()) {
			SendResponseRaw("MUNG", ERR_COMMAND_IN_RUNNING);
			return ;
		}
		if (IsError()) {
			SendResponseRaw("MUNG", ERR_COMMAND_IN_ERROR);
			return ;
		}

		g_MovePointDataNo = 2;	// 사용할 POINT_NO 저장 
		SetCommand("MUNG");
		SetControlCommand(COMM_MUNG);
		g_ResponseSend = 1;
		send("MUNG\r\n");
	}
	else if (IS_COMMAND_N(cmd, "MLOA"))	// Move Load Position
	{
		if (!IsOriginCompleted())
		{
			SendResponseRaw("MLOA", ERR_ORIGIN_ERROR);
			return ;
		}
		if (!IsStopped()) {
			SendResponseRaw("MLOA", ERR_COMMAND_IN_RUNNING);
			return ;
		}
		if (IsError()) {
			SendResponseRaw("MLOA", ERR_COMMAND_IN_ERROR);
			return ;
		}
		if (!IsReleaseBreak()) 
		{
			send("MLOA E11\r\n");
			return ;
		}

		g_MovePointDataNo = 3;	// 사용할 POINT_NO 저장 
		SetCommand("MLOA");
		SetControlCommand(COMM_MLOA);
		g_ResponseSend = 1;
		send("MLOA\r\n");
	}
	else if (IS_COMMAND_N(cmd, "MASP"))	// Move Aspirate Position 
	{
		if (!IsOriginCompleted())
		{
			SendResponseRaw("MASP", ERR_ORIGIN_ERROR);
			return ;
		}
		if (!IsStopped()) {
			SendResponseRaw("MASP", ERR_COMMAND_IN_RUNNING);
			return ;
		}
		if (IsError()) {
			SendResponseRaw("MASP", ERR_COMMAND_IN_ERROR);
			return ;
		}
		if (!IsReleaseBreak()) 
		{
			send("MASP E11\r\n");
			return ;
		}
		if ((get_var(91) == 0) && !IsExistFlask()) {
			send("MASP E10\r\n");
			return ;
		}

		g_MovePointDataNo = 4;	// 사용할 POINT_NO 저장 
		SetCommand("MASP");
		SetControlCommand(COMM_MASP);
		g_ResponseSend = 1;
		send("MASP\r\n");
	}
	else if (IS_COMMAND_N(cmd, "MDIS"))	// Move Dispense Position 
	{
		if (!IsOriginCompleted())
		{
			SendResponseRaw("MDIS", ERR_ORIGIN_ERROR);
			return ;
		}
		if (!IsStopped()) {
			SendResponseRaw("MDIS", ERR_COMMAND_IN_RUNNING);
			return ;
		}
		if (IsError()) {
			SendResponseRaw("MDIS", ERR_COMMAND_IN_ERROR);
			return ;
		}
		if (!IsReleaseBreak()) 
		{
			send("MDIS E11\r\n");
			return ;
		}
		if ((get_var(91) == 0) && !IsExistFlask()) {
			send("MDIS E10\r\n");
			return ;
		}

		g_MovePointDataNo = 5;	// 사용할 POINT_NO 저장 
		SetCommand("MDIS");
		SetControlCommand(COMM_MDIS);
		g_ResponseSend = 1;
		send("MDIS\r\n");
	}
	else if (IS_COMMAND_N(cmd, "MSHA"))	// Shake 
	{
		if (!IsOriginCompleted())
		{
			SendResponseRaw("MSHA", ERR_ORIGIN_ERROR);
			return ;
		}
		if (!IsStopped()) {
			SendResponseRaw("MSHA", ERR_COMMAND_IN_RUNNING);
			return ;
		}
		if (IsError()) {
			SendResponseRaw("MSHA", ERR_COMMAND_IN_ERROR);
			return ;
		}
		if (!IsReleaseBreak()) 
		{
			send("MSHA E11\r\n");
			return ;
		}
		if ((get_var(91) == 0) && !IsExistFlask()) {
			send("MSHA E10\r\n");
			return ;
		}

		SetCommand("MSHA");
		SetControlCommand(COMM_MSHA);
		g_ResponseSend = 1;
		send("MSHA\r\n");
	}
	else if (IS_COMMAND_N(cmd, "MWAS"))	
	{
		if (!IsOriginCompleted())
		{
			SendResponseRaw("MWAS", ERR_ORIGIN_ERROR);
			return ;
		}
		if (!IsStopped()) {
			SendResponseRaw("MWAS", ERR_COMMAND_IN_RUNNING);
			return ;
		}
		if (IsError()) {
			SendResponseRaw("MWAS", ERR_COMMAND_IN_ERROR);
			return ;
		}
		if (!IsReleaseBreak()) 
		{
			send("MWAS E11\r\n");
			return ;
		}
		if ((get_var(91) == 0) && !IsExistFlask()) {
			send("MWAS E10\r\n");
			return ;
		}

		SetCommand("MWAS");
		SetControlCommand(COMM_MWAS);
		g_ResponseSend = 1;
		send("MWAS\r\n");
	}
	else if (IS_COMMAND_N(cmd, "MWRD"))	
	{
		if (!IsOriginCompleted())
		{
			SendResponseRaw("MWRD", ERR_ORIGIN_ERROR);
			return ;
		}
		if (!IsStopped()) {
			SendResponseRaw("MWRD", ERR_COMMAND_IN_RUNNING);
			return ;
		}
		if (IsError()) {
			SendResponseRaw("MWRD", ERR_COMMAND_IN_ERROR);
			return ;
		}
		if (!IsReleaseBreak()) 
		{
			send("MWRD E11\r\n");
			return ;
		}
		if ((get_var(91) == 0) && !IsExistFlask()) {
			send("MWRD E10\r\n");
			return ;
		}

		SetCommand("MWRD");
		SetControlCommand(COMM_MWRD);
		g_ResponseSend = 1;
		send("MWRD\r\n");
	}
	else if (IS_COMMAND_N(cmd, "MWPR"))	
	{
		if (!IsOriginCompleted())
		{
			SendResponseRaw("MWPR", ERR_ORIGIN_ERROR);
			return ;
		}
		if (!IsStopped()) {
			SendResponseRaw("MWPR", ERR_COMMAND_IN_RUNNING);
			return ;
		}
		if (IsError()) {
			SendResponseRaw("MWPR", ERR_COMMAND_IN_ERROR);
			return ;
		}
		if (!IsReleaseBreak()) 
		{
			send("MWPR E11\r\n");
			return ;
		}
		if ((get_var(91) == 0) && !IsExistFlask()) {
			send("MWPR E10\r\n");
			return ;
		}
		if (IsWasteReadyPos() == 0)
		{
			// 현재 위치가 Ready 위치가 아님 
			send("MWPR E14\r\n");
			return ;
		}

		SetCommand("MWPR");
		SetControlCommand(COMM_MWPR);
		g_ResponseSend = 1;
		send("MWPR\r\n");
	}
	else if (IS_COMMAND_N(cmd, "MSEP"))
	{
		if (!IsOriginCompleted())
		{
			SendResponseRaw("MSEP", ERR_ORIGIN_ERROR);
			return ;
		}
		if (!IsStopped()) {
			SendResponseRaw("MSEP", ERR_COMMAND_IN_RUNNING);
			return ;
		}
		if (IsError()) {
			SendResponseRaw("MSEP", ERR_COMMAND_IN_ERROR);
			return ;
		}
		if (!IsReleaseBreak()) 
		{
			send("MSEP E11\r\n");
			return ;
		}
		if ((get_var(91) == 0) && !IsExistFlask()) {
			send("MSEP E10\r\n");
			return ;
		}

		SetCommand("MSEP");
		SetControlCommand(COMM_MSEP);
		g_ResponseSend = 1;
		send("MSEP\r\n");
	}
	else if (IS_COMMAND_N(cmd, "AWAS"))	// Async Waste 
	{
		if (!IsOriginCompleted())
		{
			SendResponseRaw("AWAS", ERR_ORIGIN_ERROR);
			return ;
		}
		if (!IsStopped()) {
			SendResponseRaw("AWAS", ERR_COMMAND_IN_RUNNING);
			return ;
		}
		if (IsError()) {
			SendResponseRaw("AWAS", ERR_COMMAND_IN_ERROR);
			return ;
		}
		if (!IsReleaseBreak()) 
		{
			send("AWAS E11\r\n");
			return ;
		}

		SetCommand("AWAS");
		SetControlCommand(COMM_AWAS);
		g_ResponseSend = 1;
		send("AWAS\r\n");
	}
	else if (IS_COMMAND_N(cmd, "GMEC"))	// g_MotorErrorCode를 리턴한다 
	{
		sprintf(str, "GMEC %d,%d,%d,%d\r\n", 
			g_MoveStartErrorCode[X_AXIS], 
			g_MoveStartErrorCode[Y_AXIS], 
			g_MoveStartErrorCode[Z_AXIS], 
			g_MoveStartErrorLine);
		send(str);
	}
	else if (IS_COMMAND_N(cmd, "GDEC"))
	{
		sprintf(str, "GDEC %d,%d,%d\r\n", DriverErrorCheck(0), DriverErrorCheck(1), DriverErrorCheck(2));
		send(str);
	}
	// Separate Flask의 용액을 나누는 동작 
	// TODO: Rot 90도(수평) -> Tilt -> Rot 0도  
	///////
	// Debugging command 
	// OCP 는 원점복귀를 안하고, 한 것 처럼 설정을 바꾸고, 현재 위치를 0으로 설정한다 
	else if (IS_COMMAND(cmd, "OCP"))
	{
		SetOriginCompletedFlag(0, TRUE);
		CounterReset(0);
		SetOriginCompletedFlag(1, TRUE);
		CounterReset(1);
		SetOriginCompletedFlag(2, TRUE);
		CounterReset(2);
		send("OCP\r\n");
	}
	else if (IS_COMMAND_N(cmd, "TEST"))
	{
		sprintf(str, "TEST %d\r\n", GetWasteAsyncInput());
		send(str);
	}
	else
	{
		if (strlen(cmd) > 0)
		{
			//sprintf(str, "NOK\r\n");
			sprintf(str, "%s E12\r\n", cmd);
			SerialWriteBytes(UART_PORT0, str, strlen(str));
			debug(cmd);
		}
	}
}
