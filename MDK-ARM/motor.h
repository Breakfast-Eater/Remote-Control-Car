#include "main.h"

extern const uint32_t ROTATED;
extern uint8_t EXTI_counts[2];
extern uint32_t EXTI_ticks[2];
extern uint16_t RPM[2];

void MotorL_Go(void);
void MotorL_Back(void);
void MotorR_Go(void);
void MotorR_Back(void);

void SetDutyCycles(int16_t ldc, int16_t rdc);
void ESP8266_SendRPM(UART_HandleTypeDef* phuart);
