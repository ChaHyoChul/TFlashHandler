#ifndef _HARDWARE_H
#define _HARDWARE_H

#include "common.h"

//***********************************************
// System Define
//***********************************************
#define	FIRMWARE_VER	0x10

// V5.5  : 2018.08.16, motion status filtering
//                     FPGA Read/Write ������ ��å
#define		FIRM_VER	0x55
#define		FPGA_VER	FpgaVersion()

//***********************************************
// System Function
//***********************************************

// �ϵ���� �ʱ�ȭ �Լ�
// CPU, FPGA �������͸� �ʱ�ȭ��
void InitHardware();

// �ø��� ��Ʈ 2�� ����
#define UART_PORT0       0
#define UART_PORT1       1

// UART �ʱ�ȭ �Լ�
// - port	  : uart port number
// - baudrate : ������ baudrate �� (9600 ~ 115200)
void UartInit(int port, unsigned long baudrate);

// UART ���Ź��ۿ��� 1 byte�� �������� ���� �Լ�
// - port	: uart port number
// - *ptr : char pointer
// - return : 0(���ŵ����;���), 1(���ŵ���������)
int PopRcvChar(int port, char *ptr);

// UART �۽� �Լ�
// - port	: uart port number
// - *bytes : �۽� ���ڿ��� pointer
// - length : �۽� ���ڿ��� ����
void SerialWriteBytes(int port, const void *bytes, unsigned long length);
void SerialWriteString(int port, const void *bytes);

// us ������ ������ �Լ�
void DelayUS(unsigned long us);

// ms ������ ������ �Լ�
void DelayMS(unsigned long ms);

// 2013.3.20
// us ������ tic counter ���� �о�´�.
unsigned long GetTicCountUS();

// ����� �ִ� State LED�� ON ��Ų��.
void StatusLedOn();

// ����� �ִ� State LED�� OFF ��Ų��.
void StatusLedOff();

// ����� �ִ� State LED�� TOGGLE ��Ų��.
void StatusLedToggle();

// �ݹ� �Լ� : 1ms ���ͷ�Ʈ ���� ��ƾ
// ���̺귯�� ���� ISR ���� 25us �ҿ��
void TimerIsr_1ms();

// FPGA ���� ������ �о�´�.
unsigned char FpgaVersion();

//***********************************************
// Motion Define & Structure
//***********************************************
//#define	MAX_AXIS		3		// �ִ� ���

#define	CCW				1		// ������ �ݽð���� ����
#define	CW				0		// ������ �ð���� ����

#define	MOVE_TORQUE		60		// ���۽� ���� ���� ���� (���� : 0.03A, ���� : 0 ~ 100) ex) 40 => 1.2A
#define	HOLD_TORQUE		20		// ������ ���� ���� ���� (���� : 0.03A, ���� : 0 ~ 100) ex) 10 => 0.3A

#define	MOVE_TORQUE_3A	80		// ���۽� ���� ���� ���� (���� : 0.03A, ���� : 0 ~ 100) ex) 80 => 2.4A
#define	HOLD_TORQUE_3A	20		// ������ ���� ���� ���� (���� : 0.03A, ���� : 0 ~ 100) ex) 20 => 0.6A


// ����� ���� ������ Ȯ�� �� �� �ִ� ���µ�
// GetMoveStatus() �Լ��� �̿��� ���¸� ���ü� �ִ�
#define	MOVE_STS_STOP			0x00	// ���� ����
#define	MOVE_STS_VMIN			0x01	// ������ ����
#define	MOVE_STS_ACEL			0x02	// ���� ����
#define	MOVE_STS_VMAX			0x03	// ��� ���� (�ִ��)
#define	MOVE_STS_DCEL			0x04	// ���� ����
#define	MOVE_STS_INPOS			0x05	// INPOSITION ����

// ��� ������ �ʿ��� �������� ��Ƴ��� ����ü
// App ���� ������ �ϰ� Lib ���ο��� ����Ѵ�.
// MoveStart() �Լ� ������ �� �������� �����Ͽ� ��Ǳ����� �����ϰ� �ȴ�.
typedef struct _MOVE_VARIABLE
{
	unsigned int  m_uS;			// �̵� �Ÿ� (���� : pulse, ���� : 0 ~ 2147483647)
	unsigned char m_ucDir;		// �̵� ���� (0 : CW, 1 : CCW)
	unsigned int  m_uVmax;		// �ִ� �ӵ� (���� : pps, ���� : 0 ~ 524287)
	unsigned int  m_uVmin;		// ���� �ӵ� (���� : pps, ���� : 0 ~ 32767)
	unsigned int  m_uAcel;		// ��/�� �ӵ� (���� : pps2, ���� : 0 ~ 1023)
	unsigned char m_ucMoveTorq;	// ���� ���� (���� : 0.03A, ���� : 0 ~ 100)
	unsigned char m_ucHoldTorq;	// ���� ���� (���� : 0.03A, ���� : 0 ~ 100)
} MOVE_VARIABLE;

// ��Ʈ��ũ �Ķ���� ����ü
// EEPROM ���� ���� �ּ� : 0x01C0
typedef unsigned char 	BYTE;
typedef struct
{
	BYTE bCheck1;		// 0x0000	// �׽� 0xa5 �̾�� �ǹ̰� �ִ�.
	BYTE bCheck2;		// 0x0001	// �׽� 0x5a �̾�� �ǹ̰� �ִ�.
    BYTE IPAddr[4];		// 0x0002
    BYTE MACAddr[6];	// 0x0006
    BYTE Mask[4];		// 0x000C
    BYTE GateWay[4];	// 0x0010
	int	 nTcpSvrPort;	// 0x0014

    BYTE bIsDHCPEnabled;// 0x0018

	BYTE NetBIOSName[6];// 0x0019
	BYTE szNull;		// 0x001F
} APPL_CONFIG;

// �ý��� �ڵ�, ��� ����ü
// EEPROM ���� ���� �ּ� : 0x01E0
typedef struct
{
	BYTE szCode[16];// 0
	BYTE nAxis;		// 16
	BYTE nInput;	// 17
	BYTE nOutput;	// 18
	BYTE szNull;	// 19
	int  nTimeout;	// 20	
} SYS_INFO;

extern APPL_CONFIG ApplConfig;
extern SYS_INFO SysInfo;

//***********************************************
// Motion Function
//***********************************************

// ���ܸ��� ����̹� Enable �Լ�
// - axis : 0(Z) : �������ȣ
void DriverEnable(char axis);

// ���ܸ��� ����̹� Disable �Լ�
// - axis : 0(Z) : �������ȣ
void DriverDisable(char axis);

// ���ܸ��� ����̹��� �̻���¸� Ȯ���ϱ� ���� �Լ�
// ���ܸ��� ����̹��� ������ �̻����� ����Ǹ� ����̹��� Shutdown �� �� �ִ�.
// ������ �Ǵ� ���ڰ� ��Ʈ� ���ؼ� �߻��� �� �ִ�.
// ���� ���� �� DriverReset() �Լ��� ȣ���ϰų� ���带 ���½��Ѿ� �Ѵ�.
// - axis : 0(Z) : �������ȣ
// - return : 0(����), 1(����)
char DriverErrorCheck(char axis);

// ���ܸ��� ����̹� IC�� ���½�Ű�� ���� �Լ�
// MoveStart ���� �ÿ� ���� '1' �� ���ϵ� ��� �Ǵ�,
// DriverErrorCheck ���� ���� '1' �� ���ϵ� ��� �� �Լ��� ȣ���Ͽ� ������ �� �ִ�.
// - axis : 0(Z) : �������ȣ
void DriverReset(char axis);

// ��� ���� ����
// _MOVE_VARIABLE ����ü�� ����������� �����Ͽ� ������ �����Ѵ�.
// - axis : 0(Z) : �������ȣ
// - return :	0(���� �⵿)
//				1(���� : ���ܸ��� ����̹� ����)
//				2(���� : ���� ��� ���°� MOVE_STS_STOP ���°� �ƴ� ���)
//				3(���� : �������� �̻��� �־ ���� �⵿�� ���� ���� ���)
// - ������������ :
//				�����ӵ��� 0 �̰� ���ӵ��� 0 �� ���
//				�ִ�ӵ��� �����ӵ����� ���� ���
//				�ִ�ӵ��� 0 �� ���
char MoveStart(char axis);

// ��� ���� ����
// _MOVE_VARIABLE ����ü�� m_uAcel ������ ���� �Ͽ� ���� �� ���� �Ѵ�.
// - axis : 0(Z) : �������ȣ
void MoveStop(char axis);

// ����� ���� ���� ���¸� �����´�.
// - axis : 0(Z) : �������ȣ
// - return : 
//   0x00	: (MOVE_STS_STOP)	���� ����
//   0x01	: (MOVE_STS_VMIN)	������ ����
//   0x02	: (MOVE_STS_ACEL)	���� ����
//   0x03	: (MOVE_STS_VMAX)	��� ���� (�ִ��)
//   0x04	: (MOVE_STS_DCEL)	���� ����
//   0x05	: (MOVE_STS_INPOS)	INPOSITION ����
char GetMoveStatus(char axis);

// ������ Ȧ�� ���� ������ ����
// ������ ���Ϳ� �帣�� ������ �ٿ��� ���¼Ҹ� �� ������ ���� �Ѵ�.
// _MOVE_VARIABLE ����ü�� m_ucHoldTorq ������ ���� �Ͽ� �����Ѵ�.
// - axis : 0(Z) : �������ȣ
void SetHoldTorque(char axis);

// �޽� ī������ ī���� ������ �����Ѵ�.
// - axis : 0(R), 1(X), 2(Y) : �������ȣ
// - dir : 0(CW �������� ����), 1(CCW �������� ����)
void SetCounterDirection(char axis, char dir);

// �޽� ī���� ���� 0 ���� ���� ��Ų��.
// - axis : 0(Z) : �������ȣ
void CounterReset(char axis);

// ���� ��ġ(pulse position)�� �о� �´�.
int CounterRead(char axis);


//***********************************************
// Encoder Counter Function
//***********************************************

// ���ڴ� ī���͸� ���½�Ű�� ���� �Լ�
void EncoderReset(char axis);

// ���� �����࿡ ����� ���ڴ� ī���Ϳ� Ư�� ���� ����.
unsigned int EncoderWrite(char axis, unsigned int cnt);

// ���� �����࿡ ����� ���ڴ� ī������ ���� ���� �д´�.
unsigned int EncoderRead(char axis);

// ���ڴ� ī������ ī���� ������ �����Ѵ�.
void SetEncoderDir(int axis, int dir);

// ���ڴ� ī������ ī���� ���� ���� ���¸� �����´�.
char GetEncoderDir(int axis);

//***********************************************
// I/O Function & Define
//***********************************************

// I/O Input 16���� ���� �Է°��� �����´�.
unsigned short GetInput16();

// I/O Output 8���� ���� ��µǰ� �ִ� ���� �����´�
unsigned char GetOutput8();

// I/O Output 8���� ����Ѵ�.
void SetOutput8(unsigned char data);


#define SENS_ON			1		// Active High
#define SENS_OFF		0		// Active High

// 1 ���� HOME Sensor �Է°��� �����´�. (INPUT1-0, 1-1�� �ӽ��Ҵ�)
// #define HOME_SENSOR	(~GetInput16() & 0x01)


//***********************************************
// EEPROM Access �Լ�
// EEPROM SIZE : 8KByte
// Page Size : 32Byte (�⺻ Write ����)
// �ý��� ���� ���� : 0x0000 ~ 0x01FF
// ��� ���� ����   : 0x0200 ~ 0x1FFF 
//***********************************************

// EEPROM 1Byte Read
// - usAddr : Address (���� : 0x0000 ~ 0x1FFF)
// - return : Read Data
unsigned char ReadI2C_1B(unsigned short usAddr);

// EEPROM Multi Byte Read
// - usAddr : Read ������ ���� Address (���� : 0x0000 ~ 0x1FFF)
//            Page Size(32Byte) �������� Address ������ �ؾ���
// - *pRtn  : Read Data pointer
// - Len    : Read Data Size
void ReadI2C_MB(unsigned short usAddr, unsigned char *pRtn, int Len);

// EEPROM 1Byte Write
// - usAddr : Address (���� : 0x0000 ~ 0x1FFF)
// - data   : Write Data
void WriteI2C_1B(unsigned short usAddr, unsigned char data);

// EEPROM Multi Byte Write
// - usAddr : Write ������ ���� Address (���� : 0x0000 ~ 0x1FFF)
//            Page Size(32Byte) �������� Address ������ �ؾ���
// - *pSrc  : Write Data pointer
// - Len    : Write Data Size
void WriteI2C_MB(unsigned short usAddr, unsigned char *pSrc, int Len);


#endif /* _HARDWARE_H */
