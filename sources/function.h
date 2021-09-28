#ifndef _FUNCTION_H_
#define _FUNCTION_H_


typedef struct _INTS
{
	int val[3];
	char flag[3];
} INTS;

typedef struct _DOUBLES
{
	double	val[3];
	char flag[3];
} DOUBLES;


void debug(char* str);
void debugf(char* format, ...);

// string help functions
char* strnosp(char* str);

char* int_to_binary(int i);
char* int3_to_str(int* values);
char* double3_to_str(double* values);

INTS str_to_ints(char* str);
char* ints_to_str(INTS ints);

DOUBLES str_to_doubles(char* str);
char* doubles_to_str(DOUBLES dbls);

int get_axis_sensor(int axis);
char* get_axes_sensor();
char* get_motors_pos();
int get_motor_pulse(char axis);
double get_motor_pos(char axis);


void send(char* str);
void sendf(char* format, ...);
void send_dbg(char* format, ...);

INTS get_param(int type);
void set_param(int type, INTS ints);
void set_param_no_save(int type, INTS ints);
DOUBLES get_param_f(int type);
void set_param_f(int type, DOUBLES dbls);


double my_atof(const char* s);

// 아래 함수 필요 없음 
char* cha_strchr(char* str, char c);
char* cha_strstr(char* str, char* sub);

#endif //#ifndef _FUNCTION_H_