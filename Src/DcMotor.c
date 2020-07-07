#include "main.h"

#include "DcMotor.h"


//Controller 제어기 변수
typeDcMotor  DcMotor1;
typeController Controller1;
typeBackup   Backup1;

void DcMotor1SetDutyPulse(TIM_HandleTypeDef *htim, int16_t pulse)
{
  //PWM duty를 100(ARR값)으로 제한
  if(pulse>=100) pulse=100;
  else if(pulse<-100) pulse=-100;
  //Motor Driver Enable <= 0
  DcMotor1Enable();

#if 0
  //  DC Motor 한축당 PWMx2개를 사용하는 경우
  if( pulse>=0){
 //   TIM9->CCR1=pulse; TIM9->CCR2=0;
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, pulse);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 0);
  } else{
 //   TIM9->CCR1=0  ; TIM9->CCR2=-1*pulse;
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, -1*pulse);    
  }  
#else
  //  DC Motor  Direction +PWMx1개를 사용하는 경우
  if( pulse>=0){ 
    DcMotor1DirCw() ;
    //TIMx->CCR1=pulse; 
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, pulse);
  } else{
    DcMotor1DirCcw() ;
    //TIMx->CCR1= -1*pulse; 
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1,  -1*pulse);
  }  
   HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1); 
#endif
}


void DcMotor2SetDutyPulse(TIM_HandleTypeDef *htim, int16_t pulse)
{
  //PWM duty를 100(ARR값)으로 제한
  if(pulse>=100) pulse=100;
  else if(pulse<-100) pulse=-100;
  //Motor Driver Enable <= 0
  DcMotor2Enable();

#if 0
  //  DC Motor 한축당 PWMx2개를 사용하는 경우
  if( pulse>=0){
 //   TIM9->CCR1=pulse; TIM9->CCR2=0;
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, pulse);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 0);
  } else{
 //   TIM9->CCR1=0  ; TIM9->CCR2=-1*pulse;
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, -1*pulse);    
  }  
#else
  //  DC Motor  Direction +PWMx1개를 사용하는 경우
  if( pulse>=0){ 
    DcMotor2DirCw() ;
    //TIMx->CCR1=pulse; 
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, pulse);
  } else{
    DcMotor2DirCcw() ;
    //TIMx->CCR1= -1*pulse; 
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2,  -1*pulse);
  }  
#endif
}

void CalculateAxisTurn(typeDcMotor *control)
{
   control->AxisPosition =  (float)control->Encoder32 /WHEEL_RESOLUTION;
}

void CalculateMotorRPS(typeDcMotor *control)
{
   control->MotorRps = (float)control->Encoder16Diff*Fsample/WHEEL_RESOLUTION;
}

void CalculateMotorRPM(typeDcMotor *control)
{
   control->MotorRpm = (float)control->Encoder16Diff*Fsample/WHEEL_RESOLUTION*60;
}

void CalculateWheelRPS(typeDcMotor *control)
{
   control->WheelRps = (float)control->Encoder16Diff*Fsample/4./WHEEL_RESOLUTION;
}
void CalculateWheelRPM(typeDcMotor *control)
{
   control->WheelRpm = (float)control->Encoder16Diff*Fsample/4./WHEEL_RESOLUTION*60;
}


int16_t FloatToInt16Limit(float f, int limit) 
{
   int16_t u;
   if( f >=0){
      u=Controller1.f >=U_MAX ?  U_MAX : (int16_t)(Controller1.f  + 0.5 ) ; 
   }else{
      u=Controller1.f <=-U_MAX ?  -U_MAX :(int16_t)(Controller1.f  - 0.5 ) ; 
   }
   return u;
}

int16_t LimitInt16(int16_t input, int16_t  limit) 
{
   int16_t  result;
   if(input >=0){
      result = input <= limit?   input:  limit ; 
   }else{
     result = input  >= -1*limit ? input:  -1*limit; 
   }
   return result;
}

int32_t LimitInt32(int32_t input, int32_t  limit) 
{
   int32_t  result;
   if(input >=0){
      result = input <= limit?   input:  limit ; 
   }else{
     result = input  >=-1*limit ? input:  -1*limit; 
   }
   return result;
}

float LimitFloat(float input, float limit) 
{
   float result;
   if(input >=0){
      result = input <= limit?   input:  limit ; 
   }else{
     result = input  >= -1*limit ? input:  -1*limit; 
   }
   return result;  
}

int16_t AvoidDeadzone(int16_t u, int16_t requirement_plus, int16_t requirement_minus)
{
   int16_t result;
   if(u>0)  result = MAX(u, requirement_plus); //
   else if(u<0) result = MIN(u, requirement_minus);
   else result = 0;
   return result;
}