/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include<stdio.h>
#include<math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// GY-521/MPU6050 Address
#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

// Tones in Hz
#define C1	33
#define C2	65
#define C3	131
#define C4	262
#define C5	523
#define C6	1047
#define C7	2093
#define C8	4186
#define D1	37
#define D2	73
#define D3	147
#define D4	294
#define D5	587
#define D6	1175
#define D7	2349
#define D8	4699

#define SYS_CLOCK_FREQUENCY 72000000 // 72MHz
#define PSC	6000	// Prescalar of 6000 for TIM2 & TIM3

// Battery voltage constants
#define ADC_BUF_LEN	100
#define RES_RATIO 4.30
#define SAFE_VOLTAGE 10.5

#define XBEE_BUF_LEN 22

// NEMA17 Specifications
#define STEPS_PER_REVOLUTION 200
#define STEP_ANGLE 1.8
#define MICROSTEPPING 8		// Using microstepping of 1/8

// Robot Specifications
#define WHEEL_DIAMETER 0.094		// 9.4 cm
#define WHEEL_CIRCUMFERENCE M_PI * WHEEL_DIAMETER


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

osThreadId MPU6050Handle;
osThreadId xBEERemoteHandle;
osThreadId RobotHandle;
osThreadId BatteryVoltageHandle;
osThreadId HCSR04Handle;
/* USER CODE BEGIN PV */

// MPU6050 configuration
int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax, Ay, Az, Gx, Gy, Gz;
float Accel_X_Angle, Accel_Y_Angle, Gyro_X_Angle, Gyro_Y_Angle;
float roll, pitch;

// Buzzer configuration
uint16_t buzzer_frequency = 0;
uint16_t buzzer_duty_cycle = 0;

// HC-SR04 configuration
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t is_first_captured = 0;	// is the first value captured ?
uint8_t distance[3];

// Battery voltage configuration
uint32_t adc_buf[ADC_BUF_LEN];
uint32_t adc_avg_val=0;
float battery_voltage;


// xBee configuration
uint8_t xBee_buf[XBEE_BUF_LEN];
uint8_t xBee_val[10];

// joystick configuration
uint8_t left_toggle_switch;
uint8_t right_toggle_switch;
uint8_t left_joystick_button;
uint8_t right_joystick_button;
uint16_t joystick_1_x;
uint16_t joystick_1_y;
uint16_t joystick_2_x;
uint16_t joystick_2_y;

// Stepper motor configuration
uint16_t freq_hz;

// Servo motor configuration
float servo_pos = 1500;

// Robot configuration
float Sref = 0;
float s = 0;

char uart_buf[100];
int uart_buf_len;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
void readMPU6050(void const * argument);
void moveRemote(void const * argument);
void balanceBot(void const * argument);
void checkBatteryVoltage(void const * argument);
void avoidCollision(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void MPU6050_Init(void)
{
	uint8_t check, Data;
	// check device ID WHO_AM_I
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

	if (check == 104) // if the device is present
	{
		// power management register 0x6B we should write all 0's to wake the sensor
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0, YA_ST=0, ZA_ST=0, FS_SEL=0 -> +-2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0, YG_ST=0, ZG_ST=0, FS_SEL=0 -> +- 250 degree/s
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}
}

void MPU6050_Read_Accel(void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	Ax = Accel_X_RAW/16384.0;  // get the float g
	Ay = Accel_Y_RAW/16384.0;
	Az = Accel_Z_RAW/16384.0;
}

void MPU6050_Read_Gyro(void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	/*** convert the RAW values into dps (°/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	Gx = Gyro_X_RAW/131.0;
	Gy = Gyro_Y_RAW/131.0;
	Gz = Gyro_Z_RAW/131.0;
}


void buzzer_tone(uint16_t tone)
{
	// Required frequency(tone) = SYSC_CLOCK_FREQUENCY / ((PSC + 1) * (ARR + 1))
	buzzer_frequency = SYS_CLOCK_FREQUENCY/(PSC * tone);
	buzzer_duty_cycle = buzzer_frequency/2;		// 50% duty cycle
}

void delay_us(uint16_t time)
/* Provides delay in microseconds */
{
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	while(__HAL_TIM_GET_COUNTER(&htim4) < time);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim)
{
	if(htim->Instance == TIM3 && buzzer_frequency > 0)
	{
		TIM3->ARR = buzzer_frequency - 1;
		TIM3->CCR3 = buzzer_duty_cycle;
	}

	else if(htim->Instance == TIM2)
		{
			uint16_t arr = (SYS_CLOCK_FREQUENCY/(PSC * freq_hz)) - 1;
			uint16_t step = 0;
			TIM2->ARR = arr;
			if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
				TIM2->CCR1 = arr/2;		//Duty cycle of 50%
			else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
				TIM2->CCR3 = arr/2;
			uart_buf_len = sprintf(uart_buf, "PWM STARTED FOR WHEEL\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 1000);
			HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1); 	// Stop PWM
			HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_3);
			uart_buf_len = sprintf(uart_buf, "PWM STOPPED FOR WHEEL\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 1000);
		}
	else if(htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
	{
		TIM4->ARR = 20000 - 1;	// To get frequency of 50Hz for servo motor
		TIM4->CCR4 = servo_pos;
		HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_4);
	}
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
			// if the interrupt source is channel1
		{
			if(is_first_captured==0) // if the first value is not captured
			{
				IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
				// read the first value
				is_first_captured = 1;  // set the first captured as true
				// Now change the polarity to falling edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
			}

			else if(is_first_captured==1)   // if the first is already captured
			{
				IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
				// read second value
				__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

				if (IC_Val2 > IC_Val1)
				{
					Difference = IC_Val2-IC_Val1;
				}

				else if (IC_Val1 > IC_Val2)
				{
					Difference = (0xffff - IC_Val1) + IC_Val2;
				}

				distance[0] = Difference * 0.034/2;
				is_first_captured = 0; // set it back to false

				// set polarity to rising edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
				__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC1);
			}
	}
	else if(htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
				// if the interrupt source is channel2
			{
				if(is_first_captured==0) // if the first value is not captured
				{
					IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
					// read the first value
					is_first_captured = 1;  // set the first captured as true
					// Now change the polarity to falling edge
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
				}

				else if(is_first_captured==1)   // if the first is already captured
				{
					IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
					// read second value
					__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

					if (IC_Val2 > IC_Val1)
					{
						Difference = IC_Val2-IC_Val1;
					}

					else if (IC_Val1 > IC_Val2)
					{
						Difference = (0xffff - IC_Val1) + IC_Val2;
					}

					distance[1] = Difference * 0.034/2;
					is_first_captured = 0; // set it back to false

					// set polarity to rising edge
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
					__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC2);
				}
		}
	else if(htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
					// if the interrupt source is channel4
				{
					if(is_first_captured==0) // if the first value is not captured
					{
						IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
						// read the first value
						is_first_captured = 1;  // set the first captured as true
						// Now change the polarity to falling edge
						__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
					}

					else if(is_first_captured==1)   // if the first is already captured
					{
						IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
						// read second value
						__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

						if (IC_Val2 > IC_Val1)
						{
							Difference = IC_Val2-IC_Val1;
						}

						else if (IC_Val1 > IC_Val2)
						{
							Difference = (0xffff - IC_Val1) + IC_Val2;
						}

						distance[2] = Difference * 0.034/2;
						is_first_captured = 0; // set it back to false

						// set polarity to rising edge
						__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
						__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC3);
					}
			}
}

void HCSR04_Read(void)
{
	HAL_GPIO_WritePin(TRIG_1_GPIO_Port, TRIG_1_Pin, GPIO_PIN_SET); // pull the TRIG pin HIGH
	delay_us(10); // wait for 10 us
	HAL_GPIO_WritePin(TRIG_1_GPIO_Port, TRIG_1_Pin, GPIO_PIN_RESET); // pull the TRIG pin LOW
	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
	uart_buf_len = sprintf(uart_buf, "Distance(1): %d\r\n", distance[0]);
	HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 1000);

	HAL_GPIO_WritePin(TRIG_2_GPIO_Port, TRIG_2_Pin, GPIO_PIN_SET); // pull the TRIG pin HIGH
	delay_us(10);
	HAL_GPIO_WritePin(TRIG_2_GPIO_Port, TRIG_2_Pin, GPIO_PIN_RESET); // pull the TRIG pin LOW
	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC2);
	uart_buf_len = sprintf(uart_buf, "Distance(2): %d\r\n", distance[1]);
	HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 1000);

	HAL_GPIO_WritePin(TRIG_3_GPIO_Port, TRIG_3_Pin, GPIO_PIN_SET); // pull the TRIG pin HIGH
	delay_us(10);
	HAL_GPIO_WritePin(TRIG_3_GPIO_Port, TRIG_3_Pin, GPIO_PIN_RESET); // pull the TRIG pin LOW
	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC3);
	uart_buf_len = sprintf(uart_buf, "Distance(3): %d\r\n", distance[2]);
	HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 1000);
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  for(int i=0; i < ADC_BUF_LEN; i++)
	  adc_avg_val += adc_buf[i];
  adc_avg_val = adc_avg_val/ADC_BUF_LEN;
  battery_voltage = adc_avg_val * (3.33 / 4096.00) * RES_RATIO;	// 12-bit ADC so 4096.00
  uart_buf_len = sprintf(uart_buf, "Voltage: %f\r\n", battery_voltage);
  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 1000);
}


void stepper_motor(int rpm, int direction)		// stepper motor function. Two parameters: Pulse and direction
{
	freq_hz = rpm / (((STEP_ANGLE/MICROSTEPPING) / 360) * 60);
	if(direction == 0)
	{
		HAL_GPIO_WritePin(GPIOA, DIR_1_Pin|DIR_2_Pin, GPIO_PIN_RESET); // if the direction information is zero, the PINA1 & PINA3 is set to logic 0
		HAL_GPIO_WritePin(LED_tail_1_GPIO_Port, LED_tail_1_Pin, GPIO_PIN_SET);		// Turn ON rear LEDs
		HAL_GPIO_WritePin(LED_tail_2_GPIO_Port, LED_tail_2_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, DIR_1_Pin|DIR_2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_front_3_GPIO_Port, LED_front_3_Pin, GPIO_PIN_SET);	// Turn the BLUE LEDs ON
		HAL_GPIO_WritePin(LED_front_4_GPIO_Port, LED_front_4_Pin, GPIO_PIN_SET);

	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);		// PWM Started
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3);

	HAL_GPIO_WritePin(LED_tail_1_GPIO_Port, LED_tail_1_Pin, GPIO_PIN_RESET);		// Turn OFF rear LEDs
	HAL_GPIO_WritePin(LED_tail_2_GPIO_Port, LED_tail_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_front_3_GPIO_Port, LED_front_3_Pin, GPIO_PIN_RESET);	// Turn the BLUE LEDs OFF
	HAL_GPIO_WritePin(LED_front_4_GPIO_Port, LED_front_4_Pin, GPIO_PIN_RESET);
	}
}


void left_turn(int rpm)
{
	freq_hz = rpm / (((STEP_ANGLE/MICROSTEPPING) / 360) * 60);
	HAL_GPIO_WritePin(LED_front_6_GPIO_Port, LED_front_6_Pin, GPIO_PIN_SET);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(LED_front_6_GPIO_Port, LED_front_6_Pin, GPIO_PIN_RESET);
}

void right_turn(int rpm)
{
	freq_hz = rpm / (((STEP_ANGLE/MICROSTEPPING) / 360) * 60);
	HAL_GPIO_WritePin(LED_front_1_GPIO_Port, LED_front_1_Pin, GPIO_PIN_SET);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(LED_front_1_GPIO_Port, LED_front_1_Pin, GPIO_PIN_RESET);
}

void balance_robot_func(float input_acceleration)
{
	float distance = (STEP_ANGLE/MICROSTEPPING)*(M_PI/180) * (WHEEL_DIAMETER/2); // Distance travelled in one step
	freq_hz = sqrt(input_acceleration/distance);
	uart_buf_len = sprintf(uart_buf, "ROTATION FREQUENCY: %d\r\n", freq_hz);
	HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 1000);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3);
}

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
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  uart_buf_len = sprintf(uart_buf, "HELLO WORLD\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 1000);

  MPU6050_Init();

  uart_buf_len = sprintf(uart_buf, "MPU6050 INIT DONE\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 1000);

  // turn on the PWM for buzzer
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_3);

  uart_buf_len = sprintf(uart_buf, "PWM FOR BUZZER INIT\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 1000);

  // Start receiving echo on HC-SR04 module
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);

  uart_buf_len = sprintf(uart_buf, "HCSR04 INIT\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 1000);

  uart_buf_len = sprintf(uart_buf, "ALL INIT DONE, STARTING TASKS\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 1000);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of MPU6050 */
//  osThreadDef(MPU6050, readMPU6050, osPriorityHigh, 0, 256);
//  MPU6050Handle = osThreadCreate(osThread(MPU6050), NULL);

  /* definition and creation of xBEERemote */
  osThreadDef(xBEERemote, moveRemote, osPriorityBelowNormal, 0, 320);
  xBEERemoteHandle = osThreadCreate(osThread(xBEERemote), NULL);

//  /* definition and creation of Robot */
//  osThreadDef(Robot, balanceBot, osPriorityAboveNormal, 0, 256);
//  RobotHandle = osThreadCreate(osThread(Robot), NULL);
//
//  /* definition and creation of BatteryVoltage */
//  osThreadDef(BatteryVoltage, checkBatteryVoltage, osPriorityLow, 0, 256);
//  BatteryVoltageHandle = osThreadCreate(osThread(BatteryVoltage), NULL);
//
//  /* definition and creation of HCSR04 */
//  osThreadDef(HCSR04, avoidCollision, osPriorityNormal, 0, 320);
//  HCSR04Handle = osThreadCreate(osThread(HCSR04), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 6000 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 6000 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR_1_Pin|DIR_2_Pin|TRIG_1_Pin|TRIG_2_Pin
                          |TRIG_3_Pin|LED_front_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_tail_1_Pin|LED_tail_2_Pin|LED_front_5_Pin|LED_front_6_Pin
                          |LED_front_1_Pin|LED_front_2_Pin|LED_front_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIR_1_Pin DIR_2_Pin TRIG_1_Pin TRIG_2_Pin
                           TRIG_3_Pin LED_front_4_Pin */
  GPIO_InitStruct.Pin = DIR_1_Pin|DIR_2_Pin|TRIG_1_Pin|TRIG_2_Pin
                          |TRIG_3_Pin|LED_front_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_tail_1_Pin LED_tail_2_Pin LED_front_5_Pin LED_front_6_Pin
                           LED_front_1_Pin LED_front_2_Pin LED_front_3_Pin */
  GPIO_InitStruct.Pin = LED_tail_1_Pin|LED_tail_2_Pin|LED_front_5_Pin|LED_front_6_Pin
                          |LED_front_1_Pin|LED_front_2_Pin|LED_front_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_readMPU6050 */
/**
  * @brief  Function implementing the readMPUhandle thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_readMPU6050 */
void readMPU6050(void const * argument)
{
  /* USER CODE BEGIN 5 */
//	+0.06 for some compenstation to make bot on top. Pitch is what changes from -0.69 to 0.41. -0.69 in front direction.
  uint32_t timer;
  uint32_t dt;
  /* Infinite loop */
  for(;;)
  {
	  // Calculate Roll and pitch from the accelerometer data
	  MPU6050_Read_Accel();
	  float accel_sqrt = sqrt(Ax*Ax + Az*Az);
	  if(accel_sqrt != 0.0)
	  {
		  Accel_X_Angle = atan(Ay / accel_sqrt) * 180 / M_PI;
		  Accel_Y_Angle = atan(-1 * Ax / accel_sqrt) * 180 / M_PI;
	  }

	  // Read gyroscope data
	  timer = osKernelSysTick();
	  dt = (uint32_t) (osKernelSysTick() - timer) / 1000;	// in ms
	  MPU6050_Read_Gyro();
	  Gyro_X_Angle += Gx * dt;
	  Gyro_Y_Angle += Gy * dt;

	  // Complementary filter - combine accelerometer and gyro angle values
	  roll = 0.96 * Gyro_X_Angle + 0.04 * Accel_X_Angle;
	  pitch = 0.96 * Gyro_Y_Angle + 0.04 * Accel_Y_Angle;
	  uart_buf_len = sprintf(uart_buf, "Roll: %f   Pitch: %f\r\n", roll, pitch);
	  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 1000);

	  osDelay(5);
   }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_moveRemote */
/**
* @brief Function implementing the xBEERemote thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_moveRemote */
void moveRemote(void const * argument)
{
  /* USER CODE BEGIN moveRemote */
  left_toggle_switch = 0;
  left_joystick_button = 0;
  right_joystick_button = 1;
  right_toggle_switch = 1;
  joystick_1_x = 550;
  joystick_1_y = 550;
  joystick_2_x = 550;
  joystick_2_y = 550;
  /* Infinite loop */
  for(;;)
  {
	  HAL_UART_Receive(&huart1, (uint8_t *)xBee_buf, XBEE_BUF_LEN, 50);

	  if(sizeof(xBee_buf) >= 22){
		  if(xBee_buf[1] == 0x7E){
			  for(int i=2; i<12; i++){
				  uint8_t discard = xBee_buf[i];
			  }
		  // Digital Values
		  left_toggle_switch = xBee_buf[12];
		  right_toggle_switch = (xBee_buf[13] >> 7) & 0x01;	// Reading 7th bit
		  left_joystick_button = (xBee_buf[13] >> 4) & 0x01;	// Reading 4th bit
		  right_joystick_button = (xBee_buf[13] >> 6) & 0x01;	// Reading 5th bit

		  // Analog Values
		  joystick_2_x = (xBee_buf[14] << 8) | xBee_buf[15]; // xBee_buf[14] is MSB and xBee_buf[15] is LSB
		  joystick_2_y = (xBee_buf[16] << 8) | xBee_buf[17];
		  joystick_1_x = (xBee_buf[18] << 8) | xBee_buf[19];
		  joystick_1_y = (xBee_buf[20] << 8) | xBee_buf[21];

		  uart_buf_len = sprintf(uart_buf, "L_T: %d  R_T: %d  L_J: %d  R_J: %d  J1_X: %d  J1_Y: %d  J2_X: %d  J2_Y: %d\r\n",
							     left_toggle_switch, right_toggle_switch, left_joystick_button, right_joystick_button,
							     joystick_1_x, joystick_1_y, joystick_2_x, joystick_2_y);
		  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 1000);
		  }
	  }
	  uart_buf_len = sprintf(uart_buf, "\r\nSERVO POS: %f\r\n", servo_pos);
	  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 1000);
	  if(servo_pos > 2500)
		  servo_pos = 2490;
	  else if(servo_pos < 600)
		  servo_pos = 630;

	  if(joystick_2_y > 600)
	  {
		  HAL_GPIO_WritePin(LED_tail_1_GPIO_Port, LED_tail_1_Pin, SET);
		  servo_pos += 30;
		  HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4);
		  HAL_GPIO_WritePin(LED_tail_1_GPIO_Port, LED_tail_1_Pin, SET);
	  }

	  else if(joystick_2_y < 400)
	  {
		  HAL_GPIO_WritePin(LED_tail_2_GPIO_Port, LED_tail_2_Pin, SET);
		  servo_pos -= 30;
		  HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4);
		  HAL_GPIO_WritePin(LED_tail_2_GPIO_Port, LED_tail_2_Pin, SET);
	  }

	  if(joystick_1_y > 1000)
		  stepper_motor(200, 1);
	  else if(joystick_1_y < 10)
		  stepper_motor(200, 0);
	  else if(joystick_1_x > 1000)
		  right_turn(200);
	  else if(joystick_1_x < 10)
		  left_turn(200);
	  else if(joystick_1_x < 600 && joystick_1_x > 400 && joystick_1_y < 600 && joystick_1_y > 400)
		  stepper_motor(0, 0);

	  if(right_toggle_switch == 1)
	  {
		  HAL_GPIO_WritePin(LED_front_1_GPIO_Port, LED_front_1_Pin, SET);
		  HAL_GPIO_WritePin(LED_front_4_GPIO_Port, LED_front_4_Pin, SET);
		  delay_us(1000);
		  HAL_GPIO_WritePin(LED_front_2_GPIO_Port, LED_front_2_Pin, SET);
		  HAL_GPIO_WritePin(LED_front_5_GPIO_Port, LED_front_5_Pin, SET);
		  delay_us(1000);
		  HAL_GPIO_WritePin(LED_front_3_GPIO_Port, LED_front_3_Pin, SET);
		  HAL_GPIO_WritePin(LED_front_6_GPIO_Port, LED_front_6_Pin, SET);
		  delay_us(1000);
		  HAL_GPIO_WritePin(LED_tail_1_GPIO_Port, LED_tail_1_Pin, SET);
		  HAL_GPIO_WritePin(LED_tail_2_GPIO_Port, LED_tail_2_Pin, SET);
		  delay_us(1000);
		  HAL_GPIO_WritePin(LED_front_1_GPIO_Port, LED_front_1_Pin, RESET);
		  HAL_GPIO_WritePin(LED_front_4_GPIO_Port, LED_front_4_Pin, RESET);
		  delay_us(1000);
		  HAL_GPIO_WritePin(LED_front_2_GPIO_Port, LED_front_2_Pin, RESET);
		  HAL_GPIO_WritePin(LED_front_5_GPIO_Port, LED_front_5_Pin, RESET);
		  delay_us(1000);
		  HAL_GPIO_WritePin(LED_front_3_GPIO_Port, LED_front_3_Pin, RESET);
		  HAL_GPIO_WritePin(LED_front_6_GPIO_Port, LED_front_6_Pin, RESET);
		  delay_us(1000);
		  HAL_GPIO_WritePin(LED_tail_1_GPIO_Port, LED_tail_1_Pin, RESET);
		  HAL_GPIO_WritePin(LED_tail_2_GPIO_Port, LED_tail_2_Pin, RESET);
		  delay_us(1000);
	  }

	  if(left_toggle_switch == 1)
	  {
		  HAL_GPIO_WritePin(LED_front_1_GPIO_Port, LED_front_1_Pin, SET);
		  HAL_GPIO_WritePin(LED_front_6_GPIO_Port, LED_front_6_Pin, SET);
		  buzzer_tone(C5);
		  uint8_t step = 0;
		  while(step < 255){
			  stepper_motor(200, 0);
			  step++;
		  }
	  }
	  if(left_joystick_button == 0 && right_joystick_button == 0)
	  {
		  stepper_motor(0, 0);
		  buzzer_tone(D8);
	  }
	  osDelay(50);
  }
  /* USER CODE END moveRemote */
}

/* USER CODE BEGIN Header_balanceBot */
/**
* @brief Function implementing the balRobotHandle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_balanceBot */
void balanceBot(void const * argument)
{
  /* USER CODE BEGIN balanceBot */
	uint32_t timer;
	uint32_t dt;
  /* Infinite loop */
  for(;;)
  {
	  uart_buf_len = sprintf(uart_buf, "\r\nBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB\r\n");
	  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 1000);
	  HAL_GPIO_WritePin(LED_front_2_GPIO_Port, LED_front_2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LED_front_4_GPIO_Port, LED_front_4_Pin, GPIO_PIN_SET);

	  timer = osKernelSysTick();

	  float velocity = WHEEL_CIRCUMFERENCE * freq_hz;
	  if(HAL_GPIO_ReadPin(GPIOA, DIR_1_Pin|DIR_2_Pin) == 0)
		  velocity = -velocity;
	  float displacement = velocity * dt;
	  float angle = pitch;
	  float angular_velocity = angle/dt;

	  /*** Balancing Robot using LQR Techniques
	   * u = k1x1 + k2x2 + k3x3 + k4x4 + k5*Summation(Sref - x3))
	   * where
	   * u is input for robot to balance (linear acceleration)
	   * x1 is angle
	   * x2 is angular_velocity
	   * x3 is displacement
	   * x4 is velocity
	   * Sref is reference distance
	   * K is find out using LQR Techniques in MATLAB.
	   */

	  float K[5] = {-17.7330,   -9.4892,   -3.9723,   -8.2863,    0.9029};
	  //-11.6627   -1.8369  -33.2238  -18.4712   29.8794

	  s += (Sref - displacement);
	  float u = K[0]*angle + K[1]*angular_velocity + K[2]*displacement + K[3]*velocity + K[4]*s;
	  balance_robot_func(u);	//DELETE THIS
	  osDelay(2);
	  while(u < -0.1 && u > 0.1)
	  {
		  if(angle<0.0)
			  balance_robot_func(u);
		  else if(angle>0.0)
			  balance_robot_func(-u);
	  }

	  if(u<0.1 && u>-0.1)
	  {
		  angle = 0;
		  displacement = 0;
		  angular_velocity = 0;
		  velocity = 0;
		  s = 0;
	  }

	  HAL_GPIO_WritePin(LED_front_2_GPIO_Port, LED_front_2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LED_front_4_GPIO_Port, LED_front_4_Pin, GPIO_PIN_RESET);
	  dt = (uint32_t) (osKernelSysTick() - timer) / 1000;	// in ms
	  osDelay(5);
  }
  /* USER CODE END balanceBot */
}

/* USER CODE BEGIN Header_checkBatteryVoltage */
/**
* @brief Function implementing the BatteryVoltage thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_checkBatteryVoltage */
void checkBatteryVoltage(void const * argument)
{
  /* USER CODE BEGIN checkBatteryVoltage */
  /* Infinite loop */
  for(;;)
  {
  uart_buf_len = sprintf(uart_buf, "\r\nVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 1000);
  // Start DMA for finding Battery voltage
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_BUF_LEN);	// start the ADC in DMA mode
  if(battery_voltage < SAFE_VOLTAGE)
  {
	  buzzer_tone(D8);
	  int step = 0;
	  while(step < 400)
	  {
		  stepper_motor(200, 0);
		  step++;
	  }
	  if(left_joystick_button == 0 && right_joystick_button == 0)
		  step++;
	  }
  osDelay(60000);
  }
  /* USER CODE END checkBatteryVoltage */
}

/* USER CODE BEGIN Header_avoidCollision */
/**
* @brief Function implementing the HCSR04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_avoidCollision */
void avoidCollision(void const * argument)
{
  /* USER CODE BEGIN avoidCollision */
  /* Infinite loop */
  for(;;)
  {
	  uart_buf_len = sprintf(uart_buf, "\r\nCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC\r\n");
	  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 1000);
	  HCSR04_Read();
	  if(distance[0] < 5)
	  {
		  if(distance[1] <= 5 && distance[2] <= 5)
			  stepper_motor(200, 0);
		  else if(distance[1] <= 5 && distance [2] > 5)
			  right_turn(200);
		  else
			  left_turn(200);
	  }
	  osDelay(200);
  }
  /* USER CODE END avoidCollision */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
