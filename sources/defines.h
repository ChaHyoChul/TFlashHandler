#ifndef _DEFINES_H_
#define _DEFINES_H_

// 2021.11.22
// #define SOFTWARE_VERSION	"TFLASK-H 1.00.001"
// #define SOFTWARE_VERSION	"TFLASK-H 1.00.002"
//  - AWAS (Async Washing) 기능 추가
//  - g_MoveStartErrorCode 추가 (GMEC)
// #define SOFTWARE_VERSION	"TFLASK-H 1.00.003"
//  - g_MoveStartErrorCode[3]으로 각 축의 정보 저장하는 방식으로 수정
//  - DRT에서 DriverReset() 함수 호출
//  - GDEC 명령 추가. DriverErrorCheck() 함수 결과 리턴
// #define SOFTWARE_VERSION	"TFLASK-H 1.00.004"
//  - MSEP 동작에세 X축 올라오는 속도를 VAR에 등록 하도록 수정 (VAR11)
//  - MSEP 동작의 동작 옵션 (VAR12)
//      - VAR12 == 0 : Default 동작
//      - VAR12 == 1 : Y축 이동 없이, 바로  X축 이동
// #define SOFTWARE_VERSION	"TFLASK-H 1.00.005"
//  - Waste 기능 추가
//      - MWRD. 버리는 동작의 대기 위치로 이동
//      - MWPR. 대기 위치에서 붓는 동작 후 대기 위치로 이동
// #define SOFTWARE_VERSION	"TFLASK-H 1.00.006"
//  - Waste 기능 버그 수정
//      - MWRD. 명령에서 Point Data 지정 오류 수정
//      - AWAS. 명령 디버깅 완료
// #define SOFTWARE_VERSION	"TFLASK-H 1.00.007"
//  - ORA 명령에도 origin offset 적용
//  - motor error 발생하는 라인 번호 저장. GMEX 명령으로 확인
//  - home 명령 중 모터 이동사이 200ms delay
// #define SOFTWARE_VERSION	"TFLASK-H 1.00.008"
//  - Origin Offset 위치로 이동하기 전에 SetSpeed() 함수 실행
//    (간헐적으로 Home 실행 중 Y축 Origin Offset 위치로 이동 명령에서 012 에러 대응)
// #define SOFTWARE_VERSION	"TFLASK-H 1.00.009"
//  - 008 수정에도 동일한 문제가 발생해서
//    SetSpeed() 함수에서 ORG Speed에 max, min 값 모두 사용하도록 수정

// #define SOFTWARE_VERSION	"TFLASK-H 1.00.010"
//  - HOMF 명령 추가
//  - HOME 동작 시퀀스 수정. (+방향이동->센서ON->-방향이동->센서OFF->Offset이동)

// #define SOFTWARE_VERSION	"TFLASK-H 1.01.000" // 이 버전 명은 사용하지 않음
//   - MLOA, MSAP, MDIS 에 속도(Rate) 파라메타 추가

// #define SOFTWARE_VERSION	"TFLASK-H 1.00.011" // 이 버전 명은 사용하지 않음
//   -  MLOA, MSAP, MDIS 에 속도(Rate) 파라메타 추가

// #define SOFTWARE_VERSION	"TFLASK-H 1.00.012" // 이 버전 명은 사용하지 않음
//  -  MLOA, MSAP, MDIS 에 속도(Rate) 파라메타 추가

// #define SOFTWARE_VERSION	"TFLASK-H 1.00.013" // 이 버전 명은 사용하지 않음
//   - ORA 명령 무의미한 수정

// ##define SOFTWARE_VERSION	"TFLASK-H 1.00.014" // GLEI 명령 추가. OVERRUN 에러시 에러 정보 리턴, 응답: Command-No, Axis-No(1/2/3), Sensor-No(1:+/2:-)

// #define SOFTWARE_VERSION	"TFLASK-H 1.00.015" // Limit Counter로 OVERRUN 감지. (V1=100처럼 Count 값을 입력한다)
//  MRST 명령에 파라메타와 PointData, Var 초기화

// #define SOFTWARE_VERSION	"TFLASK-H 1.00.016" // clear_error() 함수에서 "DriverReset()"호출전 에러 체크
//  SWIRL, MMLD 명령 추가

// #define SOFTWARE_VERSION	"TFLASK-H 1.00.017" // SWIRL 명령의 파라메타 확장
//  특정 매크로 실행중 ASS 명령으로 멈추지 않는 버그 수정

// #define SOFTWARE_VERSION	"TFLASK-H 1.00.018" // SWIRL 명령의 파라메타 확장. tilt 후 delay 파라메타 추가

// #define SOFTWARE_VERSION	"TFLASK-H 1.00.019" // Break timeout 기능 추가. VAR90 번에 sec 단위 시간 설정. 시간동안 통신하지 않으면 Break Hold
//  HBK, RBK 명령 제거

// #define SOFTWARE_VERSION	"TFLASK-H 01.00.20"     // MAMV 기능 추가. x, y 위치로 이동
//                                                     // MRGI 명령 추가

// #define SOFTWARE_VERSION	"TFLASK-H 01.00.21"     // RASP, RAMV 명령 추가

// #define SOFTWARE_VERSION	"TFLASK-H 01.00.22"     // Origin 버그 수정
// #define SOFTWARE_VERSION	"TFLASK-H 01.01.01"     // MAMV: move ratio 에러일 때 MRGI 리턴
// MSHK: title delay 에러일 때 MSHA 리턴

// #define SOFTWARE_VERSION	"TFLASK-H 01.02.00"     // MASP 명령에 offset 파라메타 추가 MASP 100[,0.1,0.2]
// #define SOFTWARE_VERSION	"TFLASK-H 01.02.01"     // RASP 명령에 offset 파라메타 추가 RASP 100[,0.1,0.2]
// #define SOFTWARE_VERSION	"TFLASK-H 01.02.02"     // 새로운 H/W 적용. EQIS, EQIL 변경
// #define SOFTWARE_VERSION	"TFLASK-H 01.02.03"     // 새로운 H/W 적용. Home 전에 X축 이동 (VAR12)
// #define SOFTWARE_VERSION	"TFLASK-H 01.02.04"     // STOP 명령 추가
// #define SOFTWARE_VERSION	"TFLASK-H 01.02.05"     // HOME 시퀀스 수정 (Y->X->Load 위치->Z)
// #define SOFTWARE_VERSION	"TFLASK-H 01.02.06"     // HOME 시퀀스 수정. 동작 전에 Flask 감지 되면, Grip 센서 감지될때 까지 Grip 한다 => Celltrio 확인 후 적용 예정
// PORO 명령 추가 (포인트 데이터를 하나만 읽는 명령)
// #define SOFTWARE_VERSION	"TFLASK-H 01.02.07"     // HOME 시퀀스 수정 (Flask가 감지될 때, Flask grip)

// #define SOFTWARE_VERSION	"TFLASK-H 01.02.08"     // HOME 동작 중 X축 Load 위치로 이동 속도를 SPEED_NORMAL의 80%로 변경
// #define SOFTWARE_VERSION "TFLASK-H 01.02.09" // HOME 동작 중 X축 Load 위치로 이동 속도를 SPEED_NORMAL의 80%로 변경
// #define SOFTWARE_VERSION "TFLASK-H 01.02.10" // X축 Move Torque를 65로 변경 (기존 50)
// #define SOFTWARE_VERSION "TFLASK-H 01.03.00" // X축 브레이크 동작 수정
// #define SOFTWARE_VERSION "TFLASK-H 01.03.01" // X축 브레이크 동작 수정
// #define SOFTWARE_VERSION "TFLASK-H 01.04.01" // Encoder
// #define SOFTWARE_VERSION "TFLASK-H 01.04.02" // Encoder. Home 동작 중 엔코더 상시 체크. 매크로 실행 중 Teahcing 위치 이동 후 위치 체크 후 다음 위치로 이동
// #define SOFTWARE_VERSION "TFLASK-H 01.04.03" // Encoder. 브레이크 잡을 때 delay 적용
// #define SOFTWARE_VERSION "TFLASK-H 1.5.0" // 엔코더 상시 체크 (var 21, 22 추가)
// 매크로 완료 후 브레이크 잡기전 delay 추가
// #define SOFTWARE_VERSION "TFLASK-H 1.5.0-Test" // 엔코더 상시 체크 (var 21, 22 추가)
// #define SOFTWARE_VERSION "TFLASK-H 1.5.1" // 엔코더 상시 체크. 에러 리셋 기능 수정 (Encoder  에러가 있을 경우, X,Y 모두 클리어)
// #define SOFTWARE_VERSION "TFLASK-H 1.5.2" // 엔코더 체크할 때, output 6번 bit 토글
#define SOFTWARE_VERSION "TFLASK-H 1.5.3" // TimerIsr 1ms 테스트 (6bit on -> off)

// Settings
#define COMPORT_BAUDRATE (57600)

#define NOTCH_SAMPLE (30)
#define CENTERING_SAMPLE (20)

#define M_PI (3.1415926535)
#define ONE_CYCLE_PULSE (64000)
// #define ONE_CYCLE_PULSE			(12800)
// #define ONE_CYCLE_PULSE			(25600)
#define XY_LEAD (2.0)

#define MAX_MATRIX_SIZE (7)

#define MAX_AXIS (3)
#define X_AXIS (0)
#define Y_AXIS (1)
#define Z_AXIS (2)

// Error List

#define COMMUNICATION_ERROR_BASE (1000)
#define COMMUNICATION_ERROR(A) (COMMUNICATION_ERROR_BASE + A)

#define ERR_NO (0)
#define ERR_ORIGIN_ERROR (2)
#define ERR_TIME_OVER (3)
#define ERR_EMERGENCY (4)
#define ERR_OVER_RUN (8)
#define ERR_ASYNC_WASTE_OUTPUT_ON (9)
#define ERR_ASYNC_WASTE_TIMEOUT (10)
#define ERR_MOTOR_ERROR (12)
#define ERR_GRIP_ERROR (13)
#define ERR_ENCODER_ERROR_X (14)
#define ERR_ENCODER_ERROR_Y (15)

#define ERR_RANGE_OVER (215)
#define ERR_SAMPLING_ERROR (216)
#define ERR_UNKNOWN (217)
#define ERR_BAD_DATA (218)
#define ERR_NOTCH_DATA_ERROR (221)
#define ERR_CCD_AREA (250)

#define ERR_POS_ERROR (251)

#define ERR_COMMAND_IN_RUNNING COMMUNICATION_ERROR(1)
#define ERR_COMMAND_IN_ERROR COMMUNICATION_ERROR(2)
#define ERR_WRONG_DATA_RANGE COMMUNICATION_ERROR(5)
#define ERR_WRONG_FORMAT COMMUNICATION_ERROR(6)
#define ERR_POS_NO_ERROR COMMUNICATION_ERROR(8)
#define ERR_WRONG_COMMAND COMMUNICATION_ERROR(12)

// VAR
#define VAR_TARGET_POSITION (11)
#define VAR_JUDGE_LEVEL (12)
#define VAR_NOTCH_WIDTH (13)
#define VAR_MOTOR_UNIT_CONV (14)
#define VAR_CCD_UNIT_CONV (15)
#define VAR_FILTER_SIZE (16)
#define VAR_FILTER_EXE_NO (17)
#define VAR_MAX_CCD_INTV (18)
#define VAR_SCAN_SPEED (19)
#define VAR_INOUT_DIST (20)

#define VAR_MOVE_X_DIST (21)
#define VAR_MOVE_Y_DIST (22)
#define VAR_NOTCH_POSITION (23)
#define VAR_COUNT (24)
#define VAR_ELAPSED_TIME (25)
#define VAR_MAX_SATURATION (26)
#define VAR_MIN_SATURATION (27)
#define VAR_MIN_MOTOR_POS (28)
#define VAR_STATUS (29)
#define VAR_ERROR_NO (30)

#define VAR_TEST_CCD_VALUE (31)
#define VAR_SEND_IMAGE_FLAG (32)
#define VAR_SEND_RESULT_FLAG (33)
#define VAR_SEND_STED_FLAG (34)
#define VAR_DEBUG (35)
#define VAR_SEND_OFFSET_FLAG (36)
#define VAR_CHECK_CENTER (37)
#define VAR_CCD_R0 (38)
#define VAR_NO_SENSOR_FLAG (39)
#define VAR_TEST_STEP (40)

#define VAR_RESULT_T1 (71)
#define VAR_RESULT_T2 (72)
#define VAR_RESULT_L (73)
#define VAR_ALIGNER_TYPE (74)
#define VAR_WAFER_SIZE (75)
#define VAR_WAFER_TYPE (76)
#define VAR_RANGEOUT_COUNT (77)
#define VAR_SENSOR_TYPE (78)
#define VAR_INOUT_MODE (79)
#define VAR_EXE_MODE (80)

#define VAR_CENTER_ONLY (81)
#define VAR_RETRY_NO (82)
#define VAR_VACUUM_DELAY (83)
#define VAR_DYNAX_COMPATIBLE (84)

// Execution Mode
#define EXE_MODE_OCH (2)
#define EXE_MODE_WCHC (3)
#define EXE_MODE_CCD (91)
#define EXE_MODE_DEBUG (92)
#define EXE_MODE_CHECK_DATA (90)

#define WFT_NOTCH (1)
#define WFT_FLAT (2)

#define ALT_RXY (1)
#define ALT_RXZ (2)

#define SST_CCD (0)
#define SST_LASER (1)
#define SST_LASER_KEYENCE (2)

// I/O
#define DI_VACUUM (7)
#define DI_X_POS_LIMIT (9)
#define DI_X_NEG_LIMIT (10)
#define DI_Y_POS_LIMIT (13)
#define DI_Y_NEG_LIMIT (14)

#define DO_VACUUM (6)
#define DO_LIGHT (7)

#define MAX_DI (16)
#define MAX_DO (8) //(12)

// Etc
#define EMERGENCY_STOP_DECEL (768)

#endif // #ifndef _DEFINES_H_