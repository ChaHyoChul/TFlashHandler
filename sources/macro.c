//

#include <math.h>
#include "hardware.h"
#include "macro.h"
#include "control.h"
#include "data.h"
#include "function.h"
#include "defines.h"


#define MACRO_MAX_STEP_NO		2	//(100)
#define MACRO_STEP_MAX_ARG_NO	(5)

int g_MacroNo = 0;
int g_MacroRun = 0;
int g_MacroStep = 0;

int g_TimerCount = 0;

extern int  g_ErrorCode;
extern char g_MotionCommand;
extern char g_MotionCommandBackup;
extern int g_MoveOffset[MAX_AXIS];
extern MOTION_PARAM g_MotionParam[MAX_AXIS];

extern double motor_x_pitch;
extern double motor_y_pitch;
extern int als_n;

typedef struct _MacroState
{
	char state;
	int  no;
	int  step_no;
	void* macro;
} MacroState;

typedef struct _MacroStep
{
	char type;
	//int  arg[MACRO_STEP_MAX_ARG_NO];
	int  arg1, arg2, arg3, arg4, arg5;
} MacroStep;

typedef struct _Macro
{
	MacroStep	steps[MACRO_MAX_STEP_NO];
} Macro;

// define macros
Macro mac_rst = 
{
	{
		{ MT_PTP_ABS, 4, 0, 0, 0, 0 },
		{ MT_END, 0, 0, 0, 0, 0 }
	}
};

// globa data
MacroState g_macro_state = { 0, 0, 0 };


//// not implemented
int start_macro(int no);
int run_macro();
int run_step(MacroStep step);


// funtions
int start_macro(int no)
{
	if (g_macro_state.state == MS_ERROR)
	{
		return 1002;
	}
	
	if (g_macro_state.state != MS_IDLE)
	{
		return 1001;
	}

	switch (no)
	{
	case 701:	// RST
		g_macro_state.step_no = 1;
		g_macro_state.macro = &mac_rst;
		g_macro_state.state = MS_START;
		break;
	}
	
	return 0;
}

int run_macro()
{
	switch (g_macro_state.state)
	{
	case MS_ERROR:	return 1002;
	case MS_IDLE:	return 0;
	}
	
	return 0;
}

int run_step(MacroStep step)
{
	step;
	return 0;
}

//
//
//
char MacroControl()
{
	char res = 0;
	
	if (g_MacroRun == 0)
	{
		return 0;
	}
	
	if (g_ErrorCode != 0)
	{
		g_MacroRun = 0;
		return 0;
	}
	
	switch (g_MacroNo)
	{
	case 1:		// TST
		res = MacroTest();
		break;

	default:
		return 12;
	}
	
	return 0;
}

char RunMacro(int no)
{
	if (g_MacroRun != 0)
	{
		return 1;
	}
	
	g_MacroNo = no;
	g_MacroRun = 1;
	g_MacroStep = 0;
	
	ResetTimer();
	
	return 0;
}

char StopMacro()
{
	if (g_MacroRun == 1)
	{
		g_MacroRun = 0;
		SendResponse();
	}
	
	return 0;
}

char WaitToIdle()
{
	if (g_MotionCommand == COMM_IDLE)
	{
		++g_MacroStep;
	}
	
	return 0;
}

char ResetTimer()
{
	g_TimerCount = 0;
}

char SetTimer(int ms)
{
	++g_TimerCount;

	if (g_TimerCount > ms)
	{
		g_TimerCount = 0;
		return 1;
	}
	
	return 0;
}

// Macros
char MacroTest()
{
	static POINT_DATA pd = { 0,	};
	int int_temp = 0;
	static int r_inc = 3456;
	static int timerCount = 0;

	switch (g_MacroStep)
	{
	case 0:
		SetControlCommand(COMM_ALIGN);

		++g_MacroStep;
		break;

	case 1:
		WaitToIdle();
		timerCount = 0;
		break;
	
	case 2:
		if (timerCount > 3000)
		{
			++g_MacroStep;
			break;
		}
		
		++timerCount;
		Delay1ms();
		break;
		
	case 3:
		pd = get_point_data(1);

		g_MoveOffset[0] = (pd.x - get_motor_pos(0)) / g_MotionParam[0].m_fScaleFactor;
		g_MoveOffset[1] = (pd.x - get_motor_pos(1)) / g_MotionParam[1].m_fScaleFactor;
		g_MoveOffset[2] = (pd.x - get_motor_pos(2)) / g_MotionParam[2].m_fScaleFactor;
		
		SetControlCommand(COMM_PTP);
		
		r_inc += 6543;
		if (r_inc > ONE_CYCLE_PULSE)
		{
			r_inc -= ONE_CYCLE_PULSE;
		}

		++g_MacroStep;
		break;
	
	case 4:
		WaitToIdle();
		break;

	case 5:
		if (SetTimer(200) > 0)
		{
			++g_MacroStep;
		}
		break;

	case 6:
		g_MacroStep = 0;
		break;
	}
	
	return 0;
}