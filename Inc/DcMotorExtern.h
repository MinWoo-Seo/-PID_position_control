
extern typeDcMotor DcMotor1;
extern typeController Controller1;
extern typeBackup   Backup1;

extern void DcMotor1SetDutyPulse(TIM_HandleTypeDef *htim, int16_t pulse);
extern void DcMotor2SetDutyPulse(TIM_HandleTypeDef *htim, int16_t pulse);

extern void CalculateAxisTurn(typeDcMotor *control);
extern void CalculateMotorRPS(typeDcMotor *control);
extern void CalculateMotorRPM(typeDcMotor *control);
extern void CalculateWheelRPS(typeDcMotor *control);
extern void CalculateWheelRPM(typeDcMotor *control);

extern int16_t FloatToInt16Limit(float f, int limit) ;
extern int16_t LimitInt16(int16_t input, int16_t  limit) ;
extern int32_t LimitInt32(int32_t input, int32_t  limit) ;
extern float LimitFloat(float input, float limit) ;
extern int16_t AvoidDeadzone(int16_t u, int16_t requirement_plus, int16_t requirement_minus);


