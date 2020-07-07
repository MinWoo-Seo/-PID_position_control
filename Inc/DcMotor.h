#include "main.h"

#define DIR_CW        GPIO_PIN_RESET
#define DIR_CCW       GPIO_PIN_SET

#define MOTOR_ENCODER_PPS    512
#define MOTOR_MULTIPLE_RATIO          2
#define MOTOR_1REVOLUTION_PULSE  (MOTOR_ENCODER_PPS * MOTOR_MULTIPLE_RATIO)
#define MOTOR_GEAR_RATIO      4
#define WHEEL_RESOLUTION         (MOTOR_1REVOLUTION_PULSE * MOTOR_GEAR_RATIO)      

//DC Motor Enable/Disable
#define DcMotor1Enable()        HAL_GPIO_WritePin(DCMOT1_EN_GPIO_Port, DCMOT1_EN_Pin,  GPIO_PIN_RESET)
#define DcMotor1Disable()       HAL_GPIO_WritePin(DCMOT1_EN_GPIO_Port, DCMOT1_EN_Pin,  GPIO_PIN_SET)
//DC Motor Diection
#define DcMotor1DirCw()         HAL_GPIO_WritePin(DCMOT1_DIR_GPIO_Port, DCMOT1_DIR_Pin, DIR_CW)
#define DcMotor1DirCcw()        HAL_GPIO_WritePin(DCMOT1_DIR_GPIO_Port, DCMOT1_DIR_Pin, DIR_CCW)
//DC Motor Read Encoder 16bit 부호
#define DcMotor1ReadEncoder()   ( (int16_t) __HAL_TIM_GET_COUNTER(&htim4))
//DC Motor Duty Pulse
void DcMotor1SetDutyPulse(TIM_HandleTypeDef *htim, int16_t pulse);

#define DcMotor2Enable()        HAL_GPIO_WritePin(DCMOT2_EN_GPIO_Port, DCMOT2_EN_Pin,  GPIO_PIN_RESET)
#define DcMotor2Disable()       HAL_GPIO_WritePin(DCMOT2_EN_GPIO_Port, DCMOT2_EN_Pin,  GPIO_PIN_SET)
#define DcMotor2DirCw()         HAL_GPIO_WritePin(DCMOT2_DIR_GPIO_Port, DCMOT2_DIR_Pin, DIR_CW)
#define DcMotor2DirCcw()        HAL_GPIO_WritePin(DCMOT2_DIR_GPIO_Port, DCMOT2_DIR_Pin, DIR_CCW)
void DcMotor2SetDutyPulse(TIM_HandleTypeDef *htim, int16_t pulse);
//DC Motor Read Encoder 16bit 부호
#define DcMotor2ReadEncoder()   ( (int16_t) __HAL_TIM_GET_COUNTER(&htim6))


#define ENCODER_EXTI_DISABLE()   HAL_NVIC_DisableIRQ(EXTI9_5_IRQn)
#define ENCODER_EXTI_ENABLE()    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn)
#define ENCODER_EXTI_FLAG_CLEAR()       __HAL_GPIO_EXTI_CLEAR_FLAG(EXTI9_5_IRQn)
#define ENCODER_EXTI_FLAG_SET()         __HAL_GPIO_EXTI_SET_FLAG(EXTI9_5_IRQn)

#define READ_TIMER_CNT()   ( (uint32_t) __HAL_TIM_GET_COUNTER(&htim2))

#define Fsample  1000.0
#define Tsample  (1./Fsample)
#define RECORD_MAX  300
#define U_MAX  100    

typedef struct {  
  int16_t Encoder16;
  int16_t Encoder16Old;
  int16_t Encoder16Diff;
  int32_t Encoder32;
  int32_t Encoder32Old;
  int32_t Encoder32Diff;
  float   AxisPosition;
  float   AxisTurn;

  float    MotorRps;
  float    MotorRpm;
  float    WheelRps;
  float    WheelRpm;
} typeDcMotor;

typedef struct{
   int loop;

  int32_t Output32;   //32bit형  모터현재출력(제어기입력)
  int32_t Target32;    //32bit형  모터목표출력(제어기입력)
  float    OutputFloat;  //float  모터현재출력(제어기입력)
  float    TargetFloat;  //float  모터목표출력(제어기입력)

  int32_t  Error32;
   int32_t  ErrorSum32;
   int32_t  ErrorOld32;
   int32_t  ErrorDiff32;
   int32_t  ErrorDiffOld32;

   float  ErrorFloat;
   float  ErrorSumFloat;
   float  Errorfloat;  
   float  ErrorOldFloat;
   float  ErrorDiffFloat;
   float  ErrorDiffOldFloat;

   int16_t  U;
   int16_t  u;
   float    f;
}typeController;

typedef struct {
  int32_t RecordOutput[RECORD_MAX];
  int32_t RecordU[RECORD_MAX];
//  int32_t Recordu[RECORD_MAX];
//  int32_t RecordError[RECORD_MAX];
//  int32_t RecordErrorSum[RECORD_MAX];
//  int32_t RecordErrorDiff[RECORD_MAX];
}typeBackup;

//실수f를 int16으로 변환하는데 limit이내로 제한
int16_t FloatToInt16Limit(float f, int limit) ;

//init16형 변수를 limt이내로 제한
int16_t LimitInt16(int16_t input, int16_t  limit) ;

// int32형 input변수값을 limit내로 제한
int32_t LimitInt32(int32_t input, int32_t  limit) ;

//실수 input값을 limit로 제한
float LimitFloat(float input, float limit) ;

//Motor Deadzone 피하기
int16_t AvoidDeadzone(int16_t u, int16_t requirement_plus, int16_t requirement_minus);

#define ARRAY_SIZE(x) ( sizeof((x))/sizeof((x)[0]) ) // 배열의 길이
#define ABS(x)        ( ((x)<0)?-(x):(x) ) // 절대값
#define SQUARE(x)     ( (x)*(x) ) // 제곱근
//#define UPCASE(c)     ( ((c)>='a'&&(c)<='z')?(c)-('a'-'A'):(c) ) // 소문자 -> 대문자
//#define LOWCASE(c)    ( ((c)>='A'&&(c)<='z')?(c)+('a'-'A'):(c) ) // 대문자 -> 소문자

//2개중 최대값 선택
#define MAX(x,y)      ( (x)>(y)?(x):(y) ) // 최대 값
//2개중 최소값 선택
#define MIN(x,y)      ( (x)<(y)?(x):(y) ) // 최소 값


