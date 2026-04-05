/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#include "../../MDK-ARM/motor.h"
#include "../../MDK-ARM/PID.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
	STATE_STOP = 0x41,//'A'
	STATE_MOVE,
	STATE_BACK,
	STATE_TURN_LEFT,
	STATE_TURN_RIGHT,
	STATE_ROTATE_LEFT,
	STATE_ROTATE_RIGHT,//'G'
	STATE_SET_DUTY = 'J'
} CarState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LENGTH_UDP 64

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

const char* ESP8266_AT[] = 
{
	"AT+CWMODE=2\r\n",                    			// 重启模块
    "AT+RST\r\n",              			// AP模式
    "AT+CWSAP=\"Car_AP2\",\"12345678\",1,3\r\n",	// 设置热点
    "AT+CIPMUX=0\r\n",              			// 单连接模式
    "AT+CIPSTART=\"UDP\",\"192.168.4.1\",8080,8080,0\r\n",
    NULL
};
uint8_t buffer_RX[LENGTH_UDP];
int8_t duty_cycle = 0;
//uint8_t CCR_ratio[2] = {100, 100};
CarState car_state = STATE_STOP;
uint8_t completed = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void ESP8266_Init(void);
void ExecuteCommand(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

	ESP8266_Init();
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, buffer_RX, LENGTH_UDP);
	
	//start PWM
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if(car_state == STATE_MOVE)
		{
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_cycle + (int)PID_GetIncrement(RPM[0] - RPM[1]));
		}
		
		if(completed)//no new command
			continue;
		
		switch(car_state)
		{
			case STATE_STOP:
				//TB6612FNG stand by
				HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
				
				printf("STATE_STOP\n");
				PID_Kd =1;
				completed = 1;
				
				break;
			case STATE_MOVE://测试马达方向
				//both move forward
				//stand by
				HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
				//reset duty cycle
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_cycle + (int)PID_GetIncrement(RPM[0] - RPM[1]));
				//set motors direction
				MotorL_Go();
				MotorR_Go();
				//TB6612FNG work
				HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
				
				//send L&R RPM to phone
				//ESP8266_SendRPM(&huart2);
				//printf("PWM duty cycle: %d\r\n", duty_cycle);
				printf("Kp: %f, Ki: %f, Kd: %f\r\n", PID_Kp, PID_Ki, PID_Kd);
				printf("L.RPM: %d, R.RPM: %d\r\n", RPM[0], RPM[1]);
				
				printf("L - R: %d\r\n", RPM[0] - RPM[1]);
				
				completed = 1;
				break;
			case STATE_BACK://测试马达方向
				//both move backward
				//stand by
				HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
				//reset duty cycle
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_cycle);
				//set motors direction
				MotorL_Back();
				MotorR_Back();
				//motors work
				HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
				
				printf("STATE_BACK\n");
				
//				printf("Kp: %f, Ki: %f, Kd: %f\r\n", PID_Kp, PID_Ki, PID_Kd);
//				printf("duty L: %d, R: %d\r\n", __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1), __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1));
//				printf("L.RPM: %d, R.RPM: %d\r\n", RPM[0], RPM[1]);
//				printf("L - R: %d\r\n", RPM[0] - RPM[1]);
				completed = 1;
				break;
			case STATE_TURN_LEFT:
				//halve left PWM CCR
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle / 2);
				
				printf("STATE_TURN_LEFT\n");
				
				completed = 1;
				break;
			case STATE_TURN_RIGHT:
				//halve right PWM CCR
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_cycle / 2);
				
				printf("STATE_TURN_RIGHT\n");
				
				completed = 1;
				break;
			case STATE_ROTATE_LEFT://确定方向
				//reverse left wheels
				//stand by
				HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
				//reset duty cycle
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_cycle);
				//set motors direction
				MotorL_Back();
				MotorR_Go();
				//motors work
				HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
				
				printf("STATE_ROTATE_LEFT\n");
				
				completed = 1;
				break;
			case STATE_ROTATE_RIGHT://确定方向
				//reverse right wheels
				//stand by
				HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
				//reset duty cycle
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_cycle);
				//set motors direction
				MotorL_Go();
				MotorR_Back();
				//motors work
				HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
				
				printf("STATE_ROTATE_RIGHT\n");
				
				completed = 1;
				break;
			
			default:
				completed = 1;
				break;
		}
		
		
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int fputc(int ch, FILE* f)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

void ESP8266_Init()
{
	//reset ESP8266
	//PA4 low for 100ms
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//	HAL_Delay(100);
//	//PA4 high
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
//	HAL_Delay(1000);
	
	
	//send AT orders to ESP8266
	for(uint8_t i = 0; ESP8266_AT[i] != NULL; i++)
	{
		HAL_UART_Transmit(&huart2, (uint8_t*)ESP8266_AT[i], strlen(ESP8266_AT[i]), 1000);
		HAL_Delay(500);
	}
	
}

void ExecuteCommand()//change car_state according to command
{
	//check if starts with +IPD, 2 spaces in front!
	if(!memcmp("+IPD,", buffer_RX + 2, 5))
	{
		if(buffer_RX[7 + 2] < 0x41 || buffer_RX[7 + 2] > 0x5A)
			return;//not uppercase letter
		
		completed = 0;
		
		//handle according to byte
		switch(buffer_RX[7 + 2])
		{
			case 'H'://gear up
				printf("gear up\n");
				
				duty_cycle = (duty_cycle + 20) % 100;
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_cycle);
				completed = 1;
				
				printf("duty_cycle:%i\n", duty_cycle);
				
				break;
			case 'I'://gear up
				printf("gear down\n");
				
				duty_cycle = (duty_cycle - 20) % 100;
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_cycle);
				completed = 1;
				
				printf("duty_cycle:%i\n", duty_cycle);
				
				break;
				
			case STATE_SET_DUTY://set duty cycle
				//printf("buffer_RXS %s\n", buffer_RX);
				duty_cycle = (buffer_RX[10] - '0') * 10 + (buffer_RX[11] - '0');
				
				completed = 1;
				break;
				
			case 'K'://set PID_Kp
				PID_Kp = ((buffer_RX[10] - '0') * 10 + buffer_RX[11] - '0') / 100.;
				completed = 1;
				break;
				
			case 'L'://set PID_Ki
				PID_Ki = ((buffer_RX[10] - '0') * 10 + buffer_RX[11] - '0') / 100.;
				completed = 1;
				break;
				
			case 'M'://set PID_Kd
				PID_Kd = ((buffer_RX[10] - '0') * 10 + buffer_RX[11] - '0') / 100.;
				completed = 1;
				break;
				
			default:
			
				//printf("A-G\n");
			
				car_state = buffer_RX[7 + 2];//change car state
				break;
		}
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size_t)
{
	if(huart->Instance == USART2)
	{
		//debug
		//printf("\n\nreceived:");
		HAL_UART_Transmit(&huart1, buffer_RX, size_t, 100);
		//printf("\nsize_t == %i\n", size_t);
		
		//execute command from remote control terminal
		ExecuteCommand();
		
		//receive again
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, buffer_RX, LENGTH_UDP);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)//external interrupts
{

	//printf("EXTI: %d\r\n", pin);

	if(pin == GPIO_EXTI0_Pin)//PB0 EXTI0
	{//left trigger
		if(HAL_GetTick() - EXTI_ticks[0] < 7)
			return;//debounce
		
		EXTI_counts[0]++;
		if(EXTI_counts[0] < 20)
			return;
		EXTI_counts[0] = 0;
		RPM[0] = ROTATED / (HAL_GetTick() - EXTI_ticks[0]);
		EXTI_ticks[0] = HAL_GetTick();
		return;
		//printf("L trigger\r\n");
	}
	if(pin == GPIO_EXTI1_Pin)//PB1 EXTI1
	{//right trigger
		if(HAL_GetTick() - EXTI_ticks[1] < 7)
			return;//debounce
		
		EXTI_counts[1]++;
		if(EXTI_counts[1] < 20)
			return;
		EXTI_counts[1] = 0;
		RPM[1] = ROTATED / (HAL_GetTick() - EXTI_ticks[1]);
		EXTI_ticks[1] = HAL_GetTick();
		return;
		//printf("R trigger\r\n");
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
