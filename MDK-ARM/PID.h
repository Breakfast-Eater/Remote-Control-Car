//incremental PID control

extern float PID_Kp, PID_Ki, PID_Kd;

float PID_GetIncrement(float error);
