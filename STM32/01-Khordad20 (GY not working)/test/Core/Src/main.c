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
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

uint8_t i2c_rx_data[32];
uint16_t checksum, signature, x_ball, y_ball, w, h;
uint8_t data, uart_rx_data[9];
int ball_angle , ball_dist, is_ball = 0;
int x_robot = 153 , y_robot = 120;
int v = 150;
int kick_cnt = 0, out_cnt = 0;
int shb,shr, shl, kf, kr, kb, kl, ball_in_kicker, kf_set=0, kr_set=0, kb_set=0, kl_set=0;
int pixy_updated = 0, GY_updated = 0;
#define LDR_Sens 200

int16_t Pitch=0, Roll=0, Heading=0;
uint16_t adc_result[9];
uint8_t GY_A5[] = {0xA5};
uint8_t GY_54[] = {0x54};
uint8_t GY_51[] = {0x51};
uint8_t GY_55[] = {0x55};
uint8_t GY_Init_Command[]    = {0xA5, 0x54, 0xA5, 0x51};
uint8_t GY_Request_Command[] = {0xA5, 0x51};
uint8_t set_compeleted = 0;
uint8_t i2c_tx_data[1] = {0};
#define RX2_Size 8
uint8_t Rx2_Buff[RX2_Size];

void delay(int sec){
	for(register long int i=0; i<sec; i++);
}
void initGY(){
	HAL_Delay(500);
	HAL_UART_Transmit(&huart1, GY_A5, 1, 100);
	HAL_UART_Transmit(&huart1, GY_54, 1, 100);
	HAL_Delay(500);
	HAL_UART_Transmit(&huart1, GY_A5, 1, 100);
	HAL_UART_Transmit(&huart1, GY_51, 1, 100);
	HAL_Delay(500);
	HAL_UART_Transmit(&huart1, GY_A5, 1, 100);
	HAL_UART_Transmit(&huart1, GY_55, 1, 100);
}
void readGY(){
	HAL_UART_Transmit(&huart1, GY_Request_Command, 2, 100);
	HAL_UART_Receive(&huart1, uart_rx_data, 8, 100);
	if(uart_rx_data[0] == 0x55) {
		Heading = (int16_t)(uart_rx_data[2]<<8 | uart_rx_data[3])/100.00;
		Pitch = (int16_t)(uart_rx_data[4]<<8 | uart_rx_data[5])/100.00;
		Roll = (int16_t)(uart_rx_data[6]<<8 | uart_rx_data[7])/100.00;
	}else if(uart_rx_data[0] == 0xaa) {
		Heading = (int16_t)(uart_rx_data[1]<<8 | uart_rx_data[2])/100.00;
		Pitch = (int16_t)(uart_rx_data[3]<<8 | uart_rx_data[4])/100.00;
		Roll = (int16_t)(uart_rx_data[5]<<8 | uart_rx_data[6])/100.00;
	}
}
void motor(int L1 , int L2 , int R2 , int R1){
	L1 += Heading;
	L2 += Heading;
	R1 += Heading;
	R2 += Heading;


	R1=R1*255;
	R2=R2*255;
	L1=L1*255;
	L2=L2*255;
	if (R1 > 65535) R1 = 65535;
	if (R2 > 65535) R2 = 65535;
	if (L1 > 65535) L1 = 65535;
	if (L2 > 65535) L2 = 65535;
	if (R1 < -65535) R1 = -65535;
	if (R2 < -65535) R2 = -65535;
	if (L1 < -65535) L1 = -65535;
	if (L2 < -65535) L2 = -65535;

	////////////////L1:
	if(L1 > 0){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
		TIM1->CCR1=L1;
	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
		TIM1->CCR1=L1+65535;
	}
	////////////////L2:
	if(L2 > 0){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
		TIM4->CCR3=L2;
	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
		TIM4->CCR3=L2+65535;
	}
	////////////////R2:
	if(R2 > 0){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
		TIM4->CCR2=R2;
	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
		TIM4->CCR2=R2+65535;
	}
	////////////////R1:
	if(R1 > 0){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
		TIM4->CCR1=R1;
	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
		TIM4->CCR1=R1+65535;
	}
}
void read_pixy(){
    is_ball = 0;
	if (HAL_I2C_Master_Transmit(&hi2c2, 0x54 << 1, i2c_tx_data, sizeof(i2c_tx_data), 100) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_I2C_Master_Receive(&hi2c2, 0x54 << 1,i2c_rx_data, sizeof(i2c_rx_data), 100) == HAL_OK){
		for (int i = 0; i < sizeof(i2c_rx_data)-1; i++) {
			if(i2c_rx_data[i] == 0x55 && i2c_rx_data[i+1] == 0xaa){
				is_ball = 1;
				checksum  = i2c_rx_data[i+2]  | (i2c_rx_data[i+3] << 8);
				signature = i2c_rx_data[i+4]  | (i2c_rx_data[i+5] << 8);
				x_ball    = i2c_rx_data[i+6]  | (i2c_rx_data[i+7] << 8);
				y_ball 	  = i2c_rx_data[i+8]  | (i2c_rx_data[i+9] << 8);
				w 		  = i2c_rx_data[i+10] | (i2c_rx_data[i+11] << 8);
				h 		  = i2c_rx_data[i+12] | (i2c_rx_data[i+13] << 8);
				ball_angle     = atan2(y_ball - y_robot, x_robot - x_ball) * 180 / M_PI;
//				ball_angle     = ball_angle + 180;
				if(ball_angle < 0)   ball_angle = ball_angle + 360;
				ball_dist = sqrt(pow(x_robot - x_ball , 2) + pow(y_robot - y_ball , 2));
			}
		}
	}
	HAL_Delay(1);
}
void stop(){
	motor(0,0,0,0);
}
void move(int a){
	if(a>360)     a-=360;
	if(a<0)       a+=360;
	int x = v * cos(a * M_PI / 180);
	int y = v * sin(a * M_PI / 180);
	motor((x + y) , (x - y) , (- x - y), (y - x));

}
void moveForSec(int angle, int sec){
	for(int i=0; i<sec; i++){
		read_pixy();
		move(angle);
	}
}
void moveInside(){
	if (kf > LDR_Sens && kr > LDR_Sens) 		move(225);
	else if (kf > LDR_Sens && kl > LDR_Sens) 	move(135);
	else if (kb > LDR_Sens && kr > LDR_Sens) 	move(315);
	else if (kb > LDR_Sens && kl > LDR_Sens) 	move(45);
	else if (kf > LDR_Sens) 					move(180);
	else if (kb > LDR_Sens) 					move(0);
	else if (kr > LDR_Sens) 					move(270);
	else if (kl > LDR_Sens) 					move(90);
	else stop();
}
void OUT(){
	out_cnt = 0;
	v = 190;
	if(kf > LDR_Sens && kr > LDR_Sens){
		moveForSec(225 , 10);
		while((ball_angle < 170 || ball_angle > 280) && is_ball == 1 && out_cnt < 100){
			read_pixy();
			if(kf > LDR_Sens || kr > LDR_Sens || kb > LDR_Sens || kl > LDR_Sens)	 moveInside();
			else 																	 stop();
			out_cnt++;
		}
	}
	if(kf > LDR_Sens && kl > LDR_Sens){
		moveForSec(135 , 10);
		while((ball_angle < 80 || ball_angle > 190) && is_ball == 1 && out_cnt < 100){
			read_pixy();
			if(kf > LDR_Sens || kr > LDR_Sens || kb > LDR_Sens || kl > LDR_Sens)	 moveInside();
			else 																	 stop();
			out_cnt++;
		}
	}
	if(kb > LDR_Sens && kr > LDR_Sens){
		moveForSec(315 , 10);
		while((ball_angle < 260 || ball_angle > 10) && is_ball == 1 && out_cnt < 100){
			read_pixy();
			if(kf > LDR_Sens || kr > LDR_Sens || kb > LDR_Sens || kl > LDR_Sens)	 moveInside();
			else 																	 stop();
			out_cnt++;
		}
	}
	if(kb > LDR_Sens && kl > LDR_Sens){
		moveForSec(45 , 10);
		while((ball_angle < 170 || ball_angle > 100) && is_ball == 1 && out_cnt < 100){
			read_pixy();
			if(kf > LDR_Sens || kr > LDR_Sens || kb > LDR_Sens || kl > LDR_Sens)	 moveInside();
			else 																	 stop();
			out_cnt++;
		}
	}
	if(kf > LDR_Sens){
		moveForSec(180 , 15);
		while((ball_angle < 90 || ball_angle > 270) && is_ball == 1 && out_cnt < 100){
			read_pixy();
			if(kf > LDR_Sens || kr > LDR_Sens || kb > LDR_Sens || kl > LDR_Sens)	 moveInside();
			else 																	 stop();
			out_cnt++;
		}
	}
	if(kb > LDR_Sens){
		moveForSec(0 , 15);
		while((ball_angle < 270 || ball_angle > 90) && is_ball == 1 && out_cnt < 100){
			read_pixy();
			if(kf > LDR_Sens || kr > LDR_Sens || kb > LDR_Sens || kl > LDR_Sens)	 moveInside();
			else 																	 stop();
			out_cnt++;
		}
	}
	if(kr > LDR_Sens){
		moveForSec(270 , 10);
		while((ball_angle < 180) && is_ball == 1 && out_cnt < 100){
			read_pixy();
			if(kf > LDR_Sens || kr > LDR_Sens || kb > LDR_Sens || kl > LDR_Sens)	 moveInside();
			else 																	 stop();
			out_cnt++;
		}
	}
	if(kl > LDR_Sens){
		moveForSec(90 , 10);
		while((ball_angle > 180) && is_ball == 1 && out_cnt < 100){
			read_pixy();
			if(kf > LDR_Sens || kr > LDR_Sens || kb > LDR_Sens || kl > LDR_Sens)	 moveInside();
			else 																	 stop();
			out_cnt++;
		}
	}
}
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
void spin(int spin){
	if      (spin==1)  		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
	else if (spin==0)		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
}
void shift(){
	if (is_ball){
		//kick_cnt=0
		if(ball_dist > 58){ //far
			v = 150;
			move(ball_angle);
			spin(0);

		}
		else {//near
			v = 100;
			spin(1);
			if 		(ball_angle<10 || ball_angle > 350)   move(ball_angle);
			else if (ball_angle<30)       				  move(ball_angle+20);
			else if (ball_angle<90)       				  move(ball_angle+50);
			else if (ball_angle<180)       				  move(ball_angle+80);
			else if (ball_angle<270)       				  move(ball_angle-80);
			else if (ball_angle<330)       				  move(ball_angle-25);
			else if (ball_angle<=360)       			  move(ball_angle-20);
		}
	}
	else {
	//	kick_cnt = 0;
		spin(0);
		stop();
	}

}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	shb = adc_result[8];
	shr = adc_result[0];
	shl = adc_result[1];
	kf = adc_result[3] - kf_set;
	kr = adc_result[4] - kr_set;
	kb = adc_result[5] - kb_set;
	kl = adc_result[6] - kl_set;
	ball_in_kicker = adc_result[2];
	__NOP();
}
//void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef * hi2c)
//{
//	HAL_I2C_Master_Receive_IT(&hi2c2, 0x54 << 1, i2c_rx_data, sizeof(i2c_rx_data));
//
//}
void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef * hi2c)
{
	is_ball = 0;
	for (int i = 0; i < sizeof(i2c_rx_data)-1; i++) {
		if(i2c_rx_data[i] == 0x55 && i2c_rx_data[i+1] == 0xaa){
			is_ball = 1;
			checksum  = i2c_rx_data[i+2]  | (i2c_rx_data[i+3] << 8);
			signature = i2c_rx_data[i+4]  | (i2c_rx_data[i+5] << 8);
			x_ball    = i2c_rx_data[i+6]  | (i2c_rx_data[i+7] << 8);
			y_ball 	  = i2c_rx_data[i+8]  | (i2c_rx_data[i+9] << 8);
			w 		  = i2c_rx_data[i+10] | (i2c_rx_data[i+11] << 8);
			h 		  = i2c_rx_data[i+12] | (i2c_rx_data[i+13] << 8);
			ball_angle     = atan2(y_ball - y_robot, x_robot - x_ball) * 180 / M_PI;
			if(ball_angle < 0)   ball_angle = ball_angle + 360;
			ball_dist = sqrt(pow(x_robot - x_ball , 2) + pow(y_robot - y_ball , 2));
		}
	}
	pixy_updated = 0;
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
	for(int i=0; i<RX2_Size; i++){
		if(Rx2_Buff[i] == 0xAA){
			Heading = (int16_t)(Rx2_Buff[(i+1)%8]<<8 | Rx2_Buff[(i+2)%8])/100.00;
			Pitch = (int16_t)(Rx2_Buff[(i+3)%8]<<8 | Rx2_Buff[(i+4)%8])/100.00;
			Roll = (int16_t)(Rx2_Buff[(i+5)%8]<<8 | Rx2_Buff[(i+6)%8])/100.00;
		}
	}
	HAL_UART_Transmit(&huart1, GY_Request_Command, 2, PHY_FULLDUPLEX_10M);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Rx2_Buff, RX2_Size);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
   HAL_Delay(2000);
   SSD1306_Init();

   initGY();

   HAL_ADC_Start(&hadc1);
   kf_set = 900;
   kr_set = 1300;
   kb_set = 250;
   kl_set = 500;
   i2c_tx_data[0] = 0x54;
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
   HAL_Delay(500);
   HAL_UART_Transmit(&huart1, GY_Request_Command, 2, PHY_FULLDUPLEX_10M);
   HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Rx2_Buff, RX2_Size);
   __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_result, 9);
	if(pixy_updated == 0) {
		HAL_I2C_Master_Transmit(&hi2c2, 0x54 << 1, i2c_tx_data, sizeof(i2c_tx_data), 100);
		HAL_I2C_Master_Receive_IT(&hi2c2, 0x54 << 1, i2c_rx_data, sizeof(i2c_rx_data));
		pixy_updated = 1;
	}
//	OUT();
//	shift();
	print_num("SI:", signature,  5, 10);
	print_num("XB:", x_ball,     5, 20);
	print_num("YB:", y_ball,     5, 30);
	print_num("BA:", ball_angle, 5, 40);
	print_num("GY:", Heading,    5, 50);
	print_num("KF:", kf,	    64, 10);
	print_num("KL:", kl,	    64, 20);
	print_num("KR:", kr,	    64, 30);
	print_num("KB:", kb,	    64, 40);


	SSD1306_UpdateScreen();
	//else 		stop();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 9;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_9;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  huart1.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
