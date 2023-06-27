/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"
#include "math.h"
#include "move.h"
#include "GY-25.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
#define RED_ON 	HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, 1);
#define RED_OFF HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, 0);
#define GREEN_ON 	HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, 1);
#define GREEN_OFF HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, 0);
#define BLUE_ON 	HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, 1);
#define BLUE_OFF HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, 0);
#define BUZZER_ON 	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
#define BUZZER_OFF HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t Pitch=0, Roll=0, Heading=0, rotation_fix = 0;
#define RX2_Size 8
uint8_t Rx2_Buff[RX2_Size];
int pixy_updated = 0, GY_updated = 0, adc_updated = 0, pixy_update_cnt = 0;
uint8_t i2c_tx_data[1] = {0};
uint8_t i2c_rx_data[64];
int x_robot = 136 , y_robot = 112;
int v;
uint16_t checksum, signature, x_ball, y_ball, w_goal, h_goal, x_goal, y_goal;
int ball_angle , ball_dist, is_ball = 0;
int goal_angle , goal_dist, is_goal = 0;
int kaf[16];
int kaf_set[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t battery_blink = 1, battery_blink_cnt = 0;
int already_shooted = 0, shoot_cnt = 0, mf_cnt = 0, stop_before_shoot_cnt = 0;
int arrived_to_goal = 0, ball_in_kicker = 0, out_state = 0;
char look_dir = 'F';

void print_num(char* label, int num, int x, int y){
	SSD1306_GotoXY(x, y);
	SSD1306_Puts(label, &Font_7x10, SSD1306_COLOR_WHITE);
	if(num >= 0){
		SSD1306_Putc('+', &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_Putc((num/1000)%10+'0', &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_Putc((num/100)%10+'0', &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_Putc((num/10)%10+'0', &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_Putc((num/1)%10+'0', &Font_7x10, SSD1306_COLOR_WHITE);
	}else{
		SSD1306_Putc('-', &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_Putc((-num/1000)%10+'0', &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_Putc((-num/100)%10+'0', &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_Putc((-num/10)%10+'0', &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_Putc((-num/1)%10+'0', &Font_7x10, SSD1306_COLOR_WHITE);
	}
}
uint32_t read_adc(uint32_t channel){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	sConfig.Channel = channel;
	sConfig.Rank = 1;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint32_t val = HAL_ADC_GetValue(&hadc1);
	return val;
}
void read_pixy(){
	if (HAL_I2C_Master_Transmit(&hi2c1, 0x54 << 1, i2c_tx_data, sizeof(i2c_tx_data), 100) != HAL_OK)
		Error_Handler();
	if (HAL_I2C_Master_Receive(&hi2c1, 0x54 << 1,i2c_rx_data, sizeof(i2c_rx_data), 100) == HAL_OK){
		is_ball = 0;
		is_goal = 0;
		for (int i = sizeof(i2c_rx_data)-10; i >=0 ; i--) {
			if(i2c_rx_data[i] == 0x55 && i2c_rx_data[i+1] == 0xaa && i2c_rx_data[i+2] != 0xaa && i2c_rx_data[i+2] != 0x55 ){
				checksum  = i2c_rx_data[i+2]  | (i2c_rx_data[i+3] << 8);
				if(checksum == (i2c_rx_data[i+4]  | (i2c_rx_data[i+5] << 8)) + (i2c_rx_data[i+6]  | (i2c_rx_data[i+7] << 8)) + (i2c_rx_data[i+8]  | (i2c_rx_data[i+9] << 8)) + (i2c_rx_data[i+10] | (i2c_rx_data[i+11]<< 8)) + (i2c_rx_data[i+12] | (i2c_rx_data[i+13]<< 8))){
					signature = i2c_rx_data[i+4]  | (i2c_rx_data[i+5] << 8);
					if(signature == 1){
						x_ball    = i2c_rx_data[i+6]  | (i2c_rx_data[i+7] << 8);
						y_ball 	  = i2c_rx_data[i+8]  | (i2c_rx_data[i+9] << 8);
						ball_angle     = atan2(y_ball - y_robot, x_robot - x_ball) * 180 / M_PI;
						if(ball_angle < 0)   ball_angle = ball_angle + 360;
						ball_dist = sqrt(pow(x_robot - x_ball , 2) + pow(y_robot - y_ball , 2));
						is_ball = 1;
					}
					else if(signature == 2){
						x_goal    = i2c_rx_data[i+6]  | (i2c_rx_data[i+7] << 8);
						y_goal 	  = i2c_rx_data[i+8]  | (i2c_rx_data[i+9] << 8);
						w_goal 	  = i2c_rx_data[i+10] | (i2c_rx_data[i+11]<< 8);
						h_goal 	  = i2c_rx_data[i+12] | (i2c_rx_data[i+13]<< 8);
						goal_angle     = atan2(y_goal - y_robot, x_robot - x_goal) * 180 / M_PI;
						if(goal_angle < 0)   goal_angle = goal_angle + 360;
						goal_dist = sqrt(pow(x_robot - x_goal , 2) + pow(y_robot - y_goal , 2));
						is_goal = 1;
					}
				}
			}
		}
	}
	HAL_Delay(1);
}
void read_sensors(){
	if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin)){
		RED_ON;
		BUZZER_ON;
		for(int i=0; i<16; i++){
			HAL_GPIO_WritePin(ADD0_GPIO_Port, ADD0_Pin, (i/1)%2);
			HAL_GPIO_WritePin(ADD1_GPIO_Port, ADD1_Pin, (i/2)%2);
			HAL_GPIO_WritePin(ADD2_GPIO_Port, ADD2_Pin, (i/4)%2);
			HAL_GPIO_WritePin(ADD3_GPIO_Port, ADD3_Pin, (i/8)%2);
			kaf_set[i] = read_adc(ADC_CHANNEL_14);
		}
		while(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin));
		RED_OFF;
		BUZZER_OFF;
	}
	if(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin)){
		RED_ON;
		shoot();
		while(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin));
		RED_OFF;
	}
	if(GY_updated == 0){
		HAL_UART_Transmit(&huart2, &GY_Request_Command[0], 2, PHY_FULLDUPLEX_10M);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, &Rx2_Buff[0], RX2_Size);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
		GY_updated = 1;
	}
	if(pixy_updated == 0) {
		HAL_I2C_Master_Transmit(&hi2c1, 0x54 << 1, &i2c_tx_data[0], sizeof(i2c_tx_data), 100);
		HAL_I2C_Master_Receive_IT(&hi2c1, 0x54 << 1, &i2c_rx_data[0], sizeof(i2c_rx_data));
		pixy_updated = 1;
	}
	for(int i=0; i<16; i++){
		HAL_GPIO_WritePin(ADD0_GPIO_Port, ADD0_Pin, (i/1)%2);
		HAL_GPIO_WritePin(ADD1_GPIO_Port, ADD1_Pin, (i/2)%2);
		HAL_GPIO_WritePin(ADD2_GPIO_Port, ADD2_Pin, (i/4)%2);
		HAL_GPIO_WritePin(ADD3_GPIO_Port, ADD3_Pin, (i/8)%2);
		if(i == 5)  kaf[0]  = kaf_set[i] - read_adc(ADC_CHANNEL_14);
		if(i == 9)  kaf[1]  = kaf_set[i] - read_adc(ADC_CHANNEL_14);
		if(i == 11) kaf[2]  = kaf_set[i] - read_adc(ADC_CHANNEL_14);
		if(i == 4)  kaf[3]  = kaf_set[i] - read_adc(ADC_CHANNEL_14);
		if(i == 6)  kaf[4]  = kaf_set[i] - read_adc(ADC_CHANNEL_14);
		if(i == 8)  kaf[5]  = kaf_set[i] - read_adc(ADC_CHANNEL_14);
		if(i == 12) kaf[6]  = kaf_set[i] - read_adc(ADC_CHANNEL_14);
		if(i == 7)  kaf[7]  = kaf_set[i] - read_adc(ADC_CHANNEL_14);
		if(i == 3)  kaf[8]  = kaf_set[i] - read_adc(ADC_CHANNEL_14);
		if(i == 1)  kaf[9]  = kaf_set[i] - read_adc(ADC_CHANNEL_14);
		if(i == 14) kaf[10] = kaf_set[i] - read_adc(ADC_CHANNEL_14);
		if(i == 10) kaf[11] = kaf_set[i] - read_adc(ADC_CHANNEL_14);
		if(i == 2)  kaf[12] = kaf_set[i] - read_adc(ADC_CHANNEL_14);
		if(i == 15) kaf[13] = kaf_set[i] - read_adc(ADC_CHANNEL_14);
		if(i == 13) kaf[14] = kaf_set[i] - read_adc(ADC_CHANNEL_14);
		if(i == 0)  kaf[15] = kaf_set[i] - read_adc(ADC_CHANNEL_14);
	}
	int shoot_sens   = read_adc(ADC_CHANNEL_10);
	if(shoot_sens < 1000) ball_in_kicker = 1;
	else 				  ball_in_kicker = 0;
	int battery_volt = read_adc(ADC_CHANNEL_7) * 0.044;

	print_num("GY:", Heading, 0, 0);
	print_num("XB:", x_ball, 0, 10);
	print_num("YB:", y_ball, 0, 20);
	print_num("BA", ball_angle, 0, 30);
	print_num("SH", shoot_sens, 0, 40);


	if(battery_volt < 100 && battery_volt > 50){
		if(battery_blink_cnt > 10) {
			battery_blink = !battery_blink;
			battery_blink_cnt = 0;
		}
		battery_blink_cnt++;
	}
	else battery_blink = 1;
	SSD1306_GotoXY(0, 50);
	SSD1306_Puts("BAT:", &Font_7x10, battery_blink);
	SSD1306_Putc((battery_volt/100)%10+'0', &Font_7x10, battery_blink);
	SSD1306_Putc((battery_volt/10)%10+'0', &Font_7x10, battery_blink);
	SSD1306_Puts(".", &Font_7x10, battery_blink);
	SSD1306_Putc((battery_volt/1)%10+'0', &Font_7x10, battery_blink);



	SSD1306_UpdateScreen();
}
void OUT(){
	if(kaf[15] > LDR_Sens || kaf[14] > LDR_Sens || kaf[13] > LDR_Sens || kaf[9] > LDR_Sens || kaf[6] > LDR_Sens) arrived_to_goal = 1;
	if(kaf[0] > LDR_Sens || kaf[1] > LDR_Sens || kaf[2] > LDR_Sens || kaf[4] > LDR_Sens){
		fast_stop();
		moveForSec(180, 20);
		while(is_ball){
		  read_sensors();
		  stop();
		}
	}
	else if(kaf[3] > LDR_Sens){
		fast_stop();
		moveForSec(225, 20);
		while(is_ball){
			read_sensors();
			stop();
		}
	}
	else if(kaf[6] > LDR_Sens || kaf[7] > LDR_Sens){
		fast_stop();
		moveForSec(270, 20);
		while(is_ball){
		  read_sensors();
		  stop();
		}
	}
	else if(kaf[10] > LDR_Sens){
		fast_stop();
		moveForSec(315, 20);
		while(is_ball){
			read_sensors();
			stop();
		}
	}
	else if(kaf[13] > LDR_Sens || kaf[14] > LDR_Sens || kaf[15] > LDR_Sens || kaf[11] > LDR_Sens){
	  fast_stop();
	  moveForSec(0, 20);
	  while(is_ball){
		  read_sensors();
		  stop();
	  }
	}
	else if(kaf[12] > LDR_Sens){
		fast_stop();
		moveForSec(45, 20);
		while(is_ball){
			read_sensors();
			stop();
		}
	}
	else if(kaf[9] > LDR_Sens || kaf[8] > LDR_Sens){
	  fast_stop();
	  moveForSec(90, 20);
	  while(is_ball){
		  read_sensors();
		  stop();
	  }
	}
	else if(kaf[5] > LDR_Sens){
		fast_stop();
		moveForSec(135, 20);
		while(is_ball){
			read_sensors();
			stop();
	  }
	}

}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	for(int i=0; i<RX2_Size; i++){
		if(Rx2_Buff[i] == 0xAA){
			Heading = (int16_t)(Rx2_Buff[(i+1)%8]<<8 | Rx2_Buff[(i+2)%8])/100.00 + rotation_fix;
			Pitch = (int16_t)(Rx2_Buff[(i+3)%8]<<8 | Rx2_Buff[(i+4)%8])/100.00;
			Roll = (int16_t)(Rx2_Buff[(i+5)%8]<<8 | Rx2_Buff[(i+6)%8])/100.00;

			if(Heading > 180) Heading -= 360;
			if(Heading <-180) Heading += 360;

		}
	}
	GY_updated = 0;
}
void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef * hi2c)
{
	is_ball = 0;
	is_goal = 0;
	for (int i = sizeof(i2c_rx_data)-10; i >=0 ; i--) {
		if(i2c_rx_data[i] == 0x55 && i2c_rx_data[i+1] == 0xaa && i2c_rx_data[i+2] != 0xaa && i2c_rx_data[i+2] != 0x55 ){
			checksum  = i2c_rx_data[i+2]  | (i2c_rx_data[i+3] << 8);
			if(checksum == (i2c_rx_data[i+4]  | (i2c_rx_data[i+5] << 8)) + (i2c_rx_data[i+6]  | (i2c_rx_data[i+7] << 8)) + (i2c_rx_data[i+8]  | (i2c_rx_data[i+9] << 8)) + (i2c_rx_data[i+10] | (i2c_rx_data[i+11]<< 8)) + (i2c_rx_data[i+12] | (i2c_rx_data[i+13]<< 8))){
				signature = i2c_rx_data[i+4]  | (i2c_rx_data[i+5] << 8);
				if(signature == 1){
					x_ball    = i2c_rx_data[i+6]  | (i2c_rx_data[i+7] << 8);
					y_ball 	  = i2c_rx_data[i+8]  | (i2c_rx_data[i+9] << 8);
					ball_angle     = atan2(y_ball - y_robot, x_robot - x_ball) * 180 / M_PI;
					if(ball_angle < 0)   ball_angle = ball_angle + 360;
					ball_dist = sqrt(pow(x_robot - x_ball , 2) + pow(y_robot - y_ball , 2));
					is_ball = 1;
				}
				else if(signature == 2){
					x_goal    = i2c_rx_data[i+6]  | (i2c_rx_data[i+7] << 8);
					y_goal 	  = i2c_rx_data[i+8]  | (i2c_rx_data[i+9] << 8);
					w_goal 	  = i2c_rx_data[i+10] | (i2c_rx_data[i+11]<< 8);
					h_goal 	  = i2c_rx_data[i+12] | (i2c_rx_data[i+13]<< 8);
					goal_angle     = atan2(y_goal - y_robot, x_robot - x_goal) * 180 / M_PI;
					if(goal_angle < 0)   goal_angle = goal_angle + 360;
					goal_dist = sqrt(pow(x_robot - x_goal , 2) + pow(y_robot - y_goal , 2));
					is_goal = 1;
				}
			}
		}
	}
	pixy_updated = 0;
}
void goal_keeper(){
//	if(kaf[0] > LDR_Sens || kaf[1] > LDR_Sens || kaf[2] > LDR_Sens || kaf[3] > LDR_Sens || kaf[4] > LDR_Sens || kaf[5] > LDR_Sens || kaf[6] > LDR_Sens || kaf[7] > LDR_Sens || kaf[8] > LDR_Sens || kaf[9] > LDR_Sens || kaf[10] > LDR_Sens || kaf[11] > LDR_Sens || kaf[12] > LDR_Sens || kaf[13] > LDR_Sens ||kaf[14] > LDR_Sens || kaf[15] > LDR_Sens)	out_state=1;
//	else out_state=0;
//
//	if(is_ball=1 && out_state=1){
//		if (10 < ball_angle < 180) 			move(90);
//		else if (180 < ball_angle < 350)	move(270);
//		else								stop();
//	}
//	else if(is_ball=1 && out_state=0)
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_GPIO_WritePin(MOTORS_EN_GPIO_Port, MOTORS_EN_Pin, 1);
  SSD1306_Init();
  SSD1306_GotoXY(25, 20);
  SSD1306_Puts("METIS", &Font_16x26, 1);
  SSD1306_UpdateScreen();
  for(int i=0; i<2; i++){
	  BUZZER_ON;
	  GREEN_ON;
	  HAL_Delay(100);
	  BUZZER_OFF;
	  GREEN_OFF;
	  HAL_Delay(100);
  }
  initGY(&huart2);

  HAL_UART_Transmit(&huart2, &GY_Request_Command[0], 2, PHY_FULLDUPLEX_10M);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, &Rx2_Buff[0], RX2_Size);
__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  SSD1306_Clear();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  read_sensors();
	  OUT();
	  shift();


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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hi2c2.Init.ClockSpeed = 400000;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, L2A_Pin|L1A_Pin|L1B_Pin|L2B_Pin
                          |MOTORS_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SHOOT_Pin|BUZZER_Pin|BLUE_Pin|RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SPIN_Pin|ADD3_Pin|ADD2_Pin|ADD1_Pin
                          |ADD0_Pin|R2B_Pin|R2A_Pin|R1B_Pin
                          |GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(R1A_GPIO_Port, R1A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIP2_Pin DIP1_Pin */
  GPIO_InitStruct.Pin = DIP2_Pin|DIP1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : L2A_Pin L1A_Pin L1B_Pin L2B_Pin
                           MOTORS_EN_Pin */
  GPIO_InitStruct.Pin = L2A_Pin|L1A_Pin|L1B_Pin|L2B_Pin
                          |MOTORS_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SW3_Pin SW2_Pin SW1_Pin */
  GPIO_InitStruct.Pin = SW3_Pin|SW2_Pin|SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SHOOT_Pin BUZZER_Pin BLUE_Pin RED_Pin */
  GPIO_InitStruct.Pin = SHOOT_Pin|BUZZER_Pin|BLUE_Pin|RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SPIN_Pin ADD3_Pin ADD2_Pin ADD1_Pin
                           ADD0_Pin R2B_Pin R2A_Pin R1B_Pin
                           GREEN_Pin */
  GPIO_InitStruct.Pin = SPIN_Pin|ADD3_Pin|ADD2_Pin|ADD1_Pin
                          |ADD0_Pin|R2B_Pin|R2A_Pin|R1B_Pin
                          |GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : R1A_Pin */
  GPIO_InitStruct.Pin = R1A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(R1A_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
