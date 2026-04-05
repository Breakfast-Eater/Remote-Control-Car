#include "PID.h"

float PID_Kp = 0, PID_Ki = 0, PID_Kd = 0;
float error_1 = 0;//last error
float error_2 = 0;//last error before last one

//incremental PID control
float PID_GetIncrement(float error)
{
	float increment = 0;
	
	increment = PID_Kp * (error - error_1)
				+ PID_Ki * error
				+ PID_Kd * (error - error_1 * 2 + error_2);
	error_2 = error_1;
	error_1 = error;
	if(increment < -15)
		return -15;
	if(increment > 15)
		return 15;
	return increment;
}
