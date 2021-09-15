#ifndef _SYSTEM_H
#define _SYSTEM_H

// AXIS

enum COORDINATE 
{
	COORD_ANGLE,
	COORD_TILT,
	COORD_MM
};

enum ORG_METHOD
{
	OM_CURRENT_POS,
	OM_ORIGIN,
	OM_LIMIT
};


typedef struct _SpeedInfo
{
	unsigned int	max;
	unsigned int	min;
	unsigned int	accel;
} SpeedInfo;

enum SPEED_TYPE
{
	ST_ORG,
	ST_PTP,
	ST_JOG,
	ST_MAX
};

typedef struct _AxisInfo
{
	unsigned int	encoder;
	double			gear;
	char			coord;
	
	char			enc_dir;
	char			motor_dir;
	char			org_dir;

	char			org_method;
	
	char			neg_limit_sensor;
	char			pos_limit_sensor;
	char			org_sensor;
	
	SpeedInfo		speed_info[ST_MAX];
	
	int				accel_time;		// ms
	int				decel_timie;	// ms
	
	unsigned int	pos_soft_limit;
	unsigned int	neg_soft_limit;
} AxisInfo;

typedef struct _AxisStatus
{
	char			org;
	char			neg_limit;
	char			pos_limit;

	char			is_homed;
	char			is_running;
	char			is_error;
} AxisStatus;

typedef struct _Axis
{
	AxisInfo	info;
	AxisStatus	status;
} Axis;


#define	MAX_AXES	(3)

typedef struct _System
{
	char			axes;
	Axis			axis[MAX_AXES];

	unsigned int	disable_mask;
} System;

extern System g_System;


//
// functions
//

char set_system_axes(char val);
char set_system_axis_disable(char axis, char flag);	// flag = 1 -> disable

// set axis
char set_axis_enc(char axis, unsigned int enc);
char set_axis_gear(char axis, double gear);
char set_axis_coord(char axis, char coord);
char set_axis_enc_dir(char axis, char dir);
char set_axis_motor_dir(char axis, char dir);
char set_axis_org_dir(char axis, char dir);
char set_axis_org_method(char axis, char val);



#endif // _SYSTEM_H