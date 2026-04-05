#include "motor.h"
#include "PID.h"
#include "tim.h"
#include <stdio.h>
#include <string.h>

const uint32_t ROTATED = 60000;
uint8_t EXTI_counts[2] = {0};
uint32_t EXTI_ticks[2] = {0};
uint16_t RPM[2] = {0};


void MotorL_Go()
{
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
}

void MotorL_Back()
{
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
}

void MotorR_Go()
{
	HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
}

void MotorR_Back()
{
	HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
}



//set left and right duty cycles
void SetDutyCycles(int16_t ldc, int16_t rdc)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ldc);//set left PWM CCR
	rdc += PID_GetIncrement(RPM[0] - RPM[1]);//get duty cycle according to speeds difference
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, rdc);
}

//send to phone via ESP8266
void ESP8266_SendRPM(UART_HandleTypeDef* phuart)
{
	char info[32] = {0}, temp[32] = {0};
	sprintf(info, "L RPM: %d\r\n", RPM[0]);
	sprintf(temp, "AT+CIPSEND=0,%d\r\n", strlen(info));
	HAL_UART_Transmit(phuart, (uint8_t*)temp, strlen(temp), 1000);
	HAL_Delay(100);
	HAL_UART_Transmit(phuart, (uint8_t*)info, strlen(info), 1000);
	
	info[0] = '\0';
	temp[0] = '\0';
	sprintf(info, "R RPM: %d\r\n", RPM[1]);
	sprintf(temp, "AT+CIPSEND=0,%d\r\n", strlen(info));
	HAL_UART_Transmit(phuart, (uint8_t*)temp, strlen(temp), 1000);
	HAL_Delay(100);
	HAL_UART_Transmit(phuart, (uint8_t*)info, strlen(info), 1000);
}
