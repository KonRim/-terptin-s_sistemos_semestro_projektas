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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "ssd1306.h"
#include "VEML7700_driver.h"
#include "stdio.h"
#include "Statechart.h"
#include "Statechart_required.h"
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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c2_rx;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

//Flags for I2C DMA callback
volatile uint8_t i2c1_done=0;
volatile uint8_t i2c2_done=0;

static uint8_t  Rx_buffer[4];          // 2 bytes per channel
static uint16_t value_raw[2];
static float    normal_value[2];
static float    sum_sensor[2];
static float    uart_min[2];
static float    uart_max[2];
static float 	average_sensor[2];
static float 	sum_sensor_oled[2];


static I2C_HandleTypeDef *hi2c_ch[2] = {&hi2c1, &hi2c2};

float oled_min[2];
float oled_max[2];

//For debugging
static char msg[200];
volatile int uart_counter=0;



// Current gain tracking (one per channel)
static uint8_t current_gain_step[2] = {1, 1};

// Bits, needed to be sent for different gains (gain 1/8x, 1/4x, 1x, 2x)
static const uint16_t gain_steps[] = {VEML7700_GAIN_1_8, VEML7700_GAIN_1_4, VEML7700_GAIN_1,  VEML7700_GAIN_2};

// Bits, needed to be sent for different integration times (25ms and 100ms)
static const uint16_t it_time[] = {VEML7700_IT_25MS,   VEML7700_IT_100MS};

//Lux resolutions for 25ms integration time (gain 1/8x, 1/4x, 1x, 2x)
static const float lux_resolution[] = {2.1504,  1.0752, 0.2688,   0.1344};

Statechart sc_handle; // Statechart pointer

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */





void statechart_read_i2c_sensors( Statechart* handle){
	 HAL_I2C_Mem_Read_DMA(&hi2c1, VEML7700_ADDR, VEML7700_REG_ALS, I2C_MEMADD_SIZE_8BIT, &Rx_buffer[0], 2);
	 HAL_I2C_Mem_Read_DMA(&hi2c2, VEML7700_ADDR, VEML7700_REG_ALS, I2C_MEMADD_SIZE_8BIT, &Rx_buffer[2], 2);
}

 void statechart_send_data_uart( Statechart* handle){
	 /*
	 int len = snprintf(msg, sizeof(msg),
			 "\r\nCh1: %.4f\r\nMin: %.4f\r\nMax: %.4f \r\nCh2: %.4f\r\nMin: %.4f\r\nMax: %.4f\r\n Gain1 %d\r\n Gain2 %d\r\n UART counter: %d\r\n",
			 average_sensor[0], uart_min[0], uart_max[0], average_sensor[1], uart_min[1], uart_max[1], current_gain_step[0], current_gain_step[1], uart_counter);
	 */
	 int len = snprintf(msg, sizeof(msg),
	 			 "\r\nCh1: %.4f\r\nMin: %.4f\r\nMax: %.4f \r\nCh2: %.4f\r\nMin: %.4f\r\nMax: %.4f\r\n",
	 			 average_sensor[0], uart_min[0], uart_max[0], average_sensor[1], uart_min[1], uart_max[1]);
	 HAL_UART_Transmit_DMA(&huart2, (uint8_t*)msg, len);
	 sum_sensor[0]=sum_sensor[1]=0;
 }


 void statechart_send_data_oled( Statechart* handle){
	 float average_oled[2];
	 char ch_value[2][50];
	 char ch_min[2][50];
	 char ch_max[2][50];
	 int precision[2];

	 for (int i=0; i<2; i++){
		 average_oled[i]=sum_sensor_oled[i]/10.0f;
		 switch (current_gain_step[i]) {
		 	 	 default:
		 	 	 case 0:
		 	 	 	 precision[i]=0;
		 	 		 break;

		 	 	case 1:
					 precision[i]=2;
					 break;

		 	 	 case 2:
					 precision[i]=3;
					 break;

				case 3:
					 precision[i]=4;
					 break;
		 }

		 sprintf(ch_value[i], "%.*f", precision[i], average_oled[i]);
		 sprintf(ch_min[i],   "%.*f", precision[i], oled_min[i]);
		 sprintf(ch_max[i],   "%.*f", precision[i], oled_max[i]);

		 SSD1306_GotoXY (35,0+33*i); // goto 0, 0
		 SSD1306_Puts (ch_value[i], &Font_7x10, 1); // print Ch1:

		 SSD1306_GotoXY (35, 10+33*i);
		 SSD1306_Puts (ch_min[i], &Font_7x10, 1); // print Hello

		 SSD1306_GotoXY (35, 20+33*i);
		 SSD1306_Puts (ch_max[i], &Font_7x10, 1); // print Hello
		 sum_sensor_oled[i]=0;
	 }

	 SSD1306_UpdateScreen();
	 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);

	 //uint32_t timerValue = __HAL_TIM_GET_COUNTER(&htim7);
	 /*
	 int len = snprintf(msg, sizeof(msg), "\r\nuart VALUE: %d \r\n ", uart_counter);
	 HAL_UART_Transmit_DMA(&huart2, (uint8_t*)msg, len);
	 __HAL_TIM_SET_COUNTER(&htim7, 0);
	*/
 }


 void statechart_process_i2c_samples( Statechart* handle, const sc_integer sample_no, const sc_integer mode_type){
	 for (int i= 0; i< 2; i++) {
	     value_raw[i] = Rx_buffer[i* 2] | (Rx_buffer[i* 2 + 1] << 8);
	     if(mode_type==2){
	    	 normal_value[i] = value_raw[i] * 0.0336f;
	     }
	     else {
	    	 normal_value[i] = value_raw[i] *  lux_resolution[current_gain_step[i]];
	     }
	     if(normal_value[i]>1000.0f){
	    	 float x = normal_value[i];
	    	 normal_value[i] = (VEML7700_COEF_A * x*x*x*x)
	    	                  + (VEML7700_COEF_B * x*x*x)
	    	                  + (VEML7700_COEF_C * x*x)
	    	                  + (VEML7700_COEF_D * x);
	    	 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
	     }
	     if (mode_type==0) {
	    	 uint8_t new_step;
	    	 float lux = normal_value[i];
	    	     if (lux < 25.0f) {
	    	         new_step = 3;  // 2x — maximum sensitivity for low light
	    	     } else if (lux < 100.0f) {
	    	         new_step = 2;  // 1x
	    	     } else if (lux < 10000.0f) {
	    	         new_step = 1;  // 1/4x
	    	     } else {
	    	         new_step = 0;  // 1/8x — minimum sensitivity for bright light
	    	     }

	    	     // only write to sensor if gain actually needs to change
	    	     if (new_step != current_gain_step[i]) {
	    	         current_gain_step[i] = new_step;
	    	         VEML7700_Set_Gain(hi2c_ch[i], gain_steps[current_gain_step[i]], it_time[0]);
	    	     }
	     	}

	     sum_sensor[i] = sum_sensor[i] + normal_value[i];

	     if (sample_no == 0) {
	         uart_min[i] = uart_max[i] = normal_value[i];
	     } else if (normal_value[i] < uart_min[i]) {
	         uart_min[i] = normal_value[i];
	     } else if (normal_value[i] > uart_max[i]) {
	         uart_max[i] = normal_value[i];
	     }
	 }
	 /*
	 uint32_t timerValue = __HAL_TIM_GET_COUNTER(&htim7);
		 int len = snprintf(msg, sizeof(msg), "\r\nTIMER VALUE: %lu \r\n ", timerValue);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)msg, len);
	*/

	 /*
	 int len = snprintf(msg, sizeof(msg), "\r\nNormal value1: %.4f \r\nNormal value2: %.4f \r\n ", normal_value[0], normal_value[1]);
	 		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)msg, len);
	*/
 }
  void statechart_display_mode_type( Statechart* handle, const sc_integer mode_type){
	  SSD1306_Clear();
	  switch (mode_type) {
	  	  default:
	  	  case 0:
	    	    SSD1306_GotoXY (0, 0);
			    SSD1306_Puts ("Paprastas rezimas", &Font_7x10, 1); // print Hello

				SSD1306_GotoXY (0,20); // goto 0, 0
				SSD1306_Puts ("Integravimas: 25ms", &Font_7x10, 1); // print lx

				SSD1306_GotoXY (0, 30);
				SSD1306_Puts ("Matavimo per: 40ms", &Font_7x10, 1); // print Hello

				SSD1306_GotoXY (0, 40);
				SSD1306_Puts ("Stiprinimas: kint.", &Font_7x10, 1); // print Hello

				SSD1306_GotoXY (0, 50);
				SSD1306_Puts ("(priklauso nuo lx)", &Font_7x10, 1); // print Hell

				break;

	      case 1:
	    	    SSD1306_GotoXY (0, 0);
				SSD1306_Puts ("Velniskas greitis", &Font_7x10, 1); // print Hello

				SSD1306_GotoXY (0,20); // goto 0, 0
				SSD1306_Puts ("Integravimas: 25ms", &Font_7x10, 1); // print lx

				SSD1306_GotoXY (0, 30);
				SSD1306_Puts ("Matavimo per.:25ms", &Font_7x10, 1); // print Hello

				SSD1306_GotoXY (0, 40);
				SSD1306_Puts ("Stiprinimas: 0.25", &Font_7x10, 1); // print Hello

				break;

	      case 2:
	    	  	SSD1306_GotoXY (0, 0);
				SSD1306_Puts ("Mazas apsvietimas", &Font_7x10, 1); // print Hello

				SSD1306_GotoXY (0,20); // goto 0, 0
				SSD1306_Puts ("Integravimas:100ms", &Font_7x10, 1); // print lx

				SSD1306_GotoXY (0, 30);
				SSD1306_Puts ("Matavimo per:100ms", &Font_7x10, 1); // print Hello

				SSD1306_GotoXY (0, 40);
				SSD1306_Puts ("Stiprinimas: 2", &Font_7x10, 1); // print Hello
				break;

	  }
	  SSD1306_UpdateScreen();

  }

  void statechart_change_timer_and_sensor( Statechart* handle, const sc_integer mode_type){
	  HAL_TIM_Base_Stop_IT(&htim6);

	    switch (mode_type) {
	        case 0:
			default:
				__HAL_TIM_SET_AUTORELOAD(&htim6, 39);
				for(int i=0; i<2; i++){
						current_gain_step[i]=1;
						VEML7700_Set_Gain(hi2c_ch[i], gain_steps[current_gain_step[i]], it_time[0]);
					  }
				break;

	        case 1:
	            __HAL_TIM_SET_AUTORELOAD(&htim6, 24);
	            for(int i=0; i<2; i++){
						current_gain_step[i]=1;
						VEML7700_Set_Gain(hi2c_ch[i], gain_steps[current_gain_step[i]], it_time[0]);
					  }
	            break;

	        case 2:
	            __HAL_TIM_SET_AUTORELOAD(&htim6, 99);
	            for(int i=0; i<2; i++){
						current_gain_step[i]=3;
						VEML7700_Set_Gain(hi2c_ch[i], gain_steps[current_gain_step[i]], it_time[1]);
					  }
	            break;
	    }

	    __HAL_TIM_SET_COUNTER(&htim6, 0);
	    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
	    TIM6->EGR = TIM_EGR_UG;
	    HAL_TIM_Base_Start_IT(&htim6);
  }

  void statechart_zero_data( Statechart* handle){
	  for(int i=0; i<2; i++){
		  sum_sensor_oled[i]=sum_sensor[i]=0;
	  }
  }


 void statechart_process_data_oled( Statechart* handle, const sc_integer required_sample_no, const sc_integer iterration_number){
	 for (int i= 0; i< 2; i++) {
	     average_sensor[i] = sum_sensor[i] / (float)required_sample_no;
	     sum_sensor_oled[i] = sum_sensor_oled[i] + average_sensor[i];

	     if (iterration_number == 0) {
	         oled_min[i] = uart_min[i];
	         oled_max[i] = uart_max[i];
	     } else if (uart_min[i] < oled_min[i]) {
	         oled_min[i] = uart_min[i];
	     }
	     if (uart_max[i] > oled_max[i]) {
	         oled_max[i] = uart_max[i];
	     }
	 }
 }

 void statechart_prepare_oled_screen( Statechart* handle){
	 SSD1306_DrawFilledRectangle(35, 0, 75, 63, 0);
	 SSD1306_UpdateScreen();
 }


 void statechart_display_welcome_page( Statechart* handle){
	SSD1306_GotoXY (0, 5);
	SSD1306_Puts ("Semestro projektas", &Font_7x10, 1); // print Hello

	SSD1306_GotoXY (3,15); // goto 0, 0
	SSD1306_Puts ("Sviesos matuoklis", &Font_7x10, 1); // print lx

	SSD1306_GotoXY (20, 30);
	SSD1306_Puts ("Konstantinas", &Font_7x10, 1); // print Hello

	SSD1306_GotoXY (40, 40);
	SSD1306_Puts ("Rimkus", &Font_7x10, 1); // print Hello

	SSD1306_GotoXY (37, 50);
	SSD1306_Puts ("EEI-3/1", &Font_7x10, 1); // print Hello

	SSD1306_UpdateScreen(); // update screen

 }

 void statechart_start_program( Statechart* handle){
	SSD1306_Clear();

	SSD1306_GotoXY (5,0); // goto 0, 0
	SSD1306_Puts ("Ch1:", &Font_7x10, 1); // print Ch1:
	SSD1306_GotoXY (110,0); // goto 0, 0
	SSD1306_Puts ("lx", &Font_7x10, 1); // print lx

	SSD1306_GotoXY (5, 10);
	SSD1306_Puts ("Min:", &Font_7x10, 1); // print Hello
	SSD1306_GotoXY (110,10); // goto 0, 0
	SSD1306_Puts ("lx", &Font_7x10, 1); // print lx

	SSD1306_GotoXY (5, 20);
	SSD1306_Puts ("Max:", &Font_7x10, 1); // print Hello
	SSD1306_GotoXY (110,20); // goto 0, 0
	SSD1306_Puts ("lx", &Font_7x10, 1); // print lx

	SSD1306_GotoXY (5, 33);
	SSD1306_Puts ("Ch2:", &Font_7x10, 1); // print Hello
	SSD1306_GotoXY (110,33); // goto 0, 0
	SSD1306_Puts ("lx", &Font_7x10, 1); // print lx

	SSD1306_GotoXY (5, 43);
	SSD1306_Puts ("Min:", &Font_7x10, 1); // print Hello
	SSD1306_GotoXY (110,43); // goto 0, 0
	SSD1306_Puts ("lx", &Font_7x10, 1); // print lx

	SSD1306_GotoXY (5, 53);
	SSD1306_Puts ("Max:", &Font_7x10, 1); // print Hello
	SSD1306_GotoXY (110,53); // goto 0, 0
	SSD1306_Puts ("lx", &Font_7x10, 1); // print lx

	SSD1306_UpdateScreen(); // update screen
	__HAL_TIM_SET_COUNTER(&htim6, 0);

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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  //For debugging
  HAL_TIM_Base_Start_IT(&htim7);

  SSD1306_Init (); // initialise the display





  statechart_init(&sc_handle); //Inicializuoti būtenų automatą
  statechart_enter(&sc_handle); //Pradėti vykdyti būsenų automatą

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  statechart_exit(&sc_handle);
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
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x0010020A;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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
  hi2c2.Init.Timing = 0x2010091A;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 47999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 49;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 47999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* DMA1_Ch2_3_DMA2_Ch1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch2_3_DMA2_Ch1_2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch2_3_DMA2_Ch1_2_IRQn);
  /* DMA1_Ch4_7_DMA2_Ch3_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch4_7_DMA2_Ch3_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMA2_Ch3_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C2) {
        i2c2_done=1;
    }
    if (hi2c->Instance == I2C1) {
           i2c1_done=1;
       }
    if (i2c1_done==1 && i2c2_done==1) {
               statechart_raise_i2c_callback_sensors(&sc_handle);
               i2c1_done=i2c2_done=0;
           }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
    	uart_counter++;
    	statechart_raise_timer_interrupt(&sc_handle);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == B1_Pin) {
    	statechart_raise_user_button(&sc_handle);
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
