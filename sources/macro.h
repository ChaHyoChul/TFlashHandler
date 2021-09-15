#ifndef _MACRO_H_
#define _MACRO_H_

// MACRO STATE
#define MS_IDLE					(0)
#define MS_START				(1)
#define MS_STEP_RUN				(2)
#define MS_STEP_RUNNING			(3)
#define MS_STEP_DONE			(4)
#define MS_END					(5)
#define MS_ERROR				(6)

// MACRO STEP COMMAND TYPE
#define MT_GOTO					(1)
#define MT_CALL					(2)
#define MT_PTP_ABS				(3)
#define MT_PTP_ABS_VAR			(4)
#define MT_PTP_INC				(5)
#define MT_IF_DI_THEN			(6)
#define MT_IF_VAR_THEN			(7)
#define MT_OUT					(8)
#define MT_VAR_ADD_CON			(9)
#define MT_VAR_SUB_CON			(10)
#define MT_VAR_MUL_CON			(11)
#define MT_VAR_DIV_CON			(12)
#define MT_VAR_ADD_VAR			(13)
#define MT_VAR_SUB_VAR			(14)
#define MT_VAR_MUL_VAR			(15)
#define MT_VAR_DIV_VAR			(16)
#define MT_ERROR				(17)
#define MT_END					(18)


char MacroControl();
char RunMacro(int no);
char StopMacro();

char WaitToIdle();
char ResetTimer();
char SetTimer(int ms);

char MacroTest();

#endif //#ifndef _MACRO_H_