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
#define DEBOUNCE_MS 100
#define CHANNEL_NUMBER 2
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
volatile uint8_t i2c_done[2]={0,0};
//For button debounce
volatile uint32_t last_button_time = 0;

static uint8_t  Rx_buffer[2*CHANNEL_NUMBER];          // 2 bytes per channel
static uint16_t value_raw[CHANNEL_NUMBER];
static float    normal_value[CHANNEL_NUMBER];
static float    sum_sensor[CHANNEL_NUMBER];
static float    uart_min[CHANNEL_NUMBER];
static float    uart_max[CHANNEL_NUMBER];
static float 	average_sensor[CHANNEL_NUMBER];
static float 	sum_sensor_oled[CHANNEL_NUMBER];


static I2C_HandleTypeDef *hi2c_ch[CHANNEL_NUMBER] = {&hi2c1, &hi2c2};

float oled_min[CHANNEL_NUMBER];
float oled_max[CHANNEL_NUMBER];

//For debugging
static char msg[200];
volatile int uart_counter=0;



// Current gain tracking (one per channel)
static uint8_t current_gain_step[CHANNEL_NUMBER] = {1, 1};

// Bits, needed to be sent for different gains (gain 1/8x, 1/4x, 1x, 2x)
static const uint16_t gain_steps[] = {VEML7700_GAIN_1_8, VEML7700_GAIN_1_4, VEML7700_GAIN_1,  VEML7700_GAIN_2};

// Bits, needed to be sent for different integration times (25ms and 100ms)
static const uint16_t it_time[] = {VEML7700_IT_25MS,   VEML7700_IT_100MS};

//Lux resolutions for 25ms integration time (gain 1/8x, 1/4x, 1x, 2x)
static const float lux_resolution[CHANNEL_NUMBER][4] = {{2.1504,  1.0752, 0.2688,   0.1344}, {2.1504,  1.0752, 0.2688,   0.1344}};
static const float lux_resolution_100ms[CHANNEL_NUMBER] = {0.0336, 0.0336};


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

//Functions to describe statechart

//Function to display initial page, waiting for button
void statechart_display_welcome_page( Statechart* handle){
	SSD1306_GotoXY (0, 5); //goto 0, 5
	SSD1306_Puts ("Semestro projektas", &Font_7x10, 1); //put required text

	SSD1306_GotoXY (3,15); // goto 3, 15
	SSD1306_Puts ("Sviesos matuoklis", &Font_7x10, 1); //put required text

	SSD1306_GotoXY (20, 30); // goto 20,30
	SSD1306_Puts ("Konstantinas", &Font_7x10, 1); //put required text

	SSD1306_GotoXY (40, 40); //goto 40, 40
	SSD1306_Puts ("Rimkus", &Font_7x10, 1); //put required text

	SSD1306_GotoXY (37, 50); //goto 37, 50
	SSD1306_Puts ("EEI-3/1", &Font_7x10, 1); //put required text

	SSD1306_UpdateScreen(); // update screen
}

// Function to display current measurement mode, based on user button input
void statechart_display_mode_type( Statechart* handle, const sc_integer mode_type){
	  SSD1306_Clear(); // clear screen
	  switch (mode_type) {
	  	  default: // display normal mode intro screen
	  	  case 0:
	    	    SSD1306_GotoXY (0, 0);//goto 0,0
			    SSD1306_Puts ("Paprastas rezimas", &Font_7x10, 1); // display required text

				SSD1306_GotoXY (0,20); // goto 0, 20
				SSD1306_Puts ("Integravimas: 25ms", &Font_7x10, 1); // display required text

				SSD1306_GotoXY (0, 30); // goto 0, 30
				SSD1306_Puts ("Matavimo per: 40ms", &Font_7x10, 1); // display required text

				SSD1306_GotoXY (0, 40); // goto 0, 40
				SSD1306_Puts ("Stiprinimas: kint.", &Font_7x10, 1); // display required text

				SSD1306_GotoXY (0, 50); // goto 0, 50
				SSD1306_Puts ("(priklauso nuo lx)", &Font_7x10, 1); // display required text

				break;

	      case 1: // display high speed mode intro screen
	    	    SSD1306_GotoXY (0, 0); // goto 0, 0
				SSD1306_Puts ("Velniskas greitis", &Font_7x10, 1); // display required text

				SSD1306_GotoXY (0,20); // goto 0, 20
				SSD1306_Puts ("Integravimas: 25ms", &Font_7x10, 1); // display required text

				SSD1306_GotoXY (0, 30); // goto 0, 30
				SSD1306_Puts ("Matavimo per.:25ms", &Font_7x10, 1); // display required text

				SSD1306_GotoXY (0, 40); // goto 0, 40
				SSD1306_Puts ("Stiprinimas: 0.25", &Font_7x10, 1); // display required text

				break;

	      case 2: //display low light mode intro screen
	    	  	SSD1306_GotoXY (0, 0); // goto 0, 0
				SSD1306_Puts ("Mazas apsvietimas", &Font_7x10, 1); // display required text

				SSD1306_GotoXY (0,20); // goto 0, 20
				SSD1306_Puts ("Integravimas:100ms", &Font_7x10, 1); // display required text

				SSD1306_GotoXY (0, 30); // goto 0, 30
				SSD1306_Puts ("Matavimo per:100ms", &Font_7x10, 1); // display required text

				SSD1306_GotoXY (0, 40); // goto 0, 40
				SSD1306_Puts ("Stiprinimas: 2", &Font_7x10, 1); // display required text
				break;

	  }
	  SSD1306_UpdateScreen(); //update screen
}

//Function to change timer interrupt speed and sensor integration mode
void statechart_change_timer_and_sensor( Statechart* handle, const sc_integer mode_type){
	//Stop timer to prevent glitches
	HAL_TIM_Base_Stop_IT(&htim6);
	//Which the timer preload based on the current mode selected
	    switch (mode_type) {
	        case 0: //Normal mode
			default:
				__HAL_TIM_SET_AUTORELOAD(&htim6, 39); //Set required timer interrupt speed (40ms)
				for(int i=0; i<CHANNEL_NUMBER; i++){
						current_gain_step[i]=1; //Set current sensor gain in program to 1/4 for both channels
						//Set sensor gain to 1/4 and integration time to 25ms for both sensors
						VEML7700_Set_Gain(hi2c_ch[i], gain_steps[current_gain_step[i]], it_time[0]);
					  }
				break;

	        case 1: // High speed mode
	            __HAL_TIM_SET_AUTORELOAD(&htim6, 24); //Set timer overload to 25ms
	            for(int i=0; i<CHANNEL_NUMBER; i++){
						current_gain_step[i]=1;//Set current sensor gain in program to 1/4 for both channels
						//Set sensor gain to 1/4 and integration time to 25ms for both sensors
						VEML7700_Set_Gain(hi2c_ch[i], gain_steps[current_gain_step[i]], it_time[0]);
					  }
	            break;

	        case 2: // Low light mode
	            __HAL_TIM_SET_AUTORELOAD(&htim6, 99); //Set timer overload to 100ms
	            for(int i=0; i<CHANNEL_NUMBER; i++){
						current_gain_step[i]=3;//Set current sensor gain in program to 2 for both channels
						//Set sensor gain to 2 and integration time to 100ms for both sensors
						VEML7700_Set_Gain(hi2c_ch[i], gain_steps[current_gain_step[i]], it_time[1]);
					  }
	            break;
	    }

	    __HAL_TIM_SET_COUNTER(&htim6, 0); //Set timer counter to 0 (reset timer)
	    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE); //Clear IT flag
	    TIM6->EGR = TIM_EGR_UG; //Update the cleared flag
	    HAL_TIM_Base_Start_IT(&htim6); //And finally start the timer, when it is configured
 }

//Function to set UART sum (sum of samples) to zero
void statechart_zero_uart_sum( Statechart* handle){
	for(int i=0; i<CHANNEL_NUMBER; i++){
			sum_sensor[i]=0; //Set sensor sum data of UART refresh to zero
	}
}

//Function to set OLED sum (sum of iterrations) to zero
void statechart_zero_oled_sum( Statechart* handle){
	for(int i=0; i<CHANNEL_NUMBER; i++){
			  sum_sensor_oled[i]=0; //Set sensor sum data of OLED refresh to zero
	}
}

//Function to read both sensors in I2C DMA (gain and integration time was set before)
void statechart_read_i2c_sensors( Statechart* handle){
	for (int i= 0; i< CHANNEL_NUMBER; i++) {
	//Save all 4 bytes to same RX buffer (2 bytes per channel)
	 HAL_I2C_Mem_Read_DMA(hi2c_ch[i], VEML7700_ADDR, VEML7700_REG_ALS, I2C_MEMADD_SIZE_8BIT, &Rx_buffer[i*2], 2);
	}
}

//Function to process raw I2C sample data
void statechart_process_i2c_samples( Statechart* handle, const sc_integer sample_no, const sc_integer mode_type){
	 for (int i= 0; i< CHANNEL_NUMBER; i++) {
	     value_raw[i] = Rx_buffer[i* 2] | (Rx_buffer[i* 2 + 1] << 8); //Save raw sample data
	     if(mode_type==2){ //If measuring mode is "Low light"
	    	 normal_value[i] = value_raw[i] * lux_resolution_100ms[i]; //Convert to lux based on convertion table for 100ms (gain of 2)
	     }
	     else {
	    	 //Convert to lux based on 25ms integration and current gain value
	    	 normal_value[i] = value_raw[i] *  lux_resolution[i][current_gain_step[i]];
	     }
	     //If we have lux value above 1000
	     if(normal_value[i]>1000.0f){
	    	 float x = normal_value[i];
	    	 //Apply correction formula from datasheet
	    	 normal_value[i] = (VEML7700_COEF_A * x*x*x*x)
	    	                  + (VEML7700_COEF_B * x*x*x)
	    	                  + (VEML7700_COEF_C * x*x)
	    	                  + (VEML7700_COEF_D * x);
	    	 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1); //Set test LED high to indicate correction
	     }

	     if (mode_type==0) { //If measuring in "Normal mode"
	    	 uint8_t new_step;
	    	 float lux = normal_value[i];
	    	 	 //Based on the current lux value, set the gain for better measurement precision
	    	     if (lux < 25.0f) {
	    	         new_step = 3;  // New gain is 2, maximum precision
	    	     } else if (lux < 100.0f) {
	    	         new_step = 2;  // New gain is 1, slightly worse precision
	    	     } else if (lux < 10000.0f) {
	    	         new_step = 1;  // New gain is 1/4, more linear measurement
	    	     } else {
	    	         new_step = 0;  // New gain is 1/8, highest possible maximum value (to not max out readings)
	    	     }

	    	     // Write to sensor for gain change if new gain is different from previous
	    	     if (new_step != current_gain_step[i]) {
	    	         current_gain_step[i] = new_step; //Change current gain value
	    	         VEML7700_Set_Gain(hi2c_ch[i], gain_steps[current_gain_step[i]], it_time[0]); //Send new gain to sensor
	    	     }
	     	}
	     //Increase the sum by current value (for average calculation)
	     sum_sensor[i] = sum_sensor[i] + normal_value[i];

	     //If sample is first in iterration
	     if (sample_no == 0) {
	         uart_min[i] = uart_max[i] = normal_value[i]; //It sets both min and max
	     } else if (normal_value[i] < uart_min[i]) { //If it is less than current min
	         uart_min[i] = normal_value[i]; //Becomes new min
	     } else if (normal_value[i] > uart_max[i]) { //If it more than current max
	         uart_max[i] = normal_value[i]; //Becomes new max
	     }
	 }
}

//Function to send the required data over UART to PC
void statechart_send_data_uart(Statechart* handle){
	//Local variables
	int len = 0;
	int precision[2];
	len += snprintf(msg + len,sizeof(msg) - len, "\r\n");

	//For loop to loop over both channels
	for (int i = 0; i < CHANNEL_NUMBER; i++)
	{
		//Select the required precision based on current gain
		switch (current_gain_step[i]) {
			default:
			case 0:
				precision[i] = 0;
				break;

			case 1:
				precision[i] = 2;
				break;

			case 2:
				precision[i] = 3;
				break;

			case 3:
				precision[i] = 4;
				break;
		}
		//Format the UART message from the data and precision values


		len += snprintf(msg + len,sizeof(msg) - len,"Ch%d: %.*f\r\nMin: %.*f\r\nMax: %.*f\r\n",
			i + 1, precision[i], average_sensor[i], precision[i], uart_min[i], precision[i], uart_max[i]);
	}

	//Transmit the formated message over UART with DMA
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)msg, len);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0); //Reset the test LED
}

//Function to display required data on OLED display
void statechart_send_data_oled( Statechart* handle){
	 float average_oled[2];
	 char ch_value[2][50];
	 char ch_min[2][50];
	 char ch_max[2][50];
	 int precision[2];
	 SSD1306_DrawFilledRectangle(35, 0, 75, 63, 0); //Block out old data
	 //Set for loop to format both channels
	 for (int i=0; i<CHANNEL_NUMBER; i++){
		 average_oled[i]=sum_sensor_oled[i]/10.0f;

		 //Change the displayed precision based on current gain value
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
		 //Format the required data with sprintf
		 sprintf(ch_value[i], "%.*f", precision[i], average_oled[i]);
		 sprintf(ch_min[i],   "%.*f", precision[i], oled_min[i]);
		 sprintf(ch_max[i],   "%.*f", precision[i], oled_max[i]);

		 SSD1306_GotoXY (35,0+33*i); // Go to 35, and 0 or 33
		 SSD1306_Puts (ch_value[i], &Font_7x10, 1); // print average value:

		 SSD1306_GotoXY (35, 10+33*i); // Go to 35, and 10 or 43
		 SSD1306_Puts (ch_min[i], &Font_7x10, 1); // print min value

		 SSD1306_GotoXY (35, 20+33*i); //Go to 35 and 20 or 53
		 SSD1306_Puts (ch_max[i], &Font_7x10, 1); // print max value
	 }
	 SSD1306_UpdateScreen(); //Update the screen
}




//Function to process data for OLED
void statechart_process_data_oled( Statechart* handle, const sc_integer required_sample_no, const sc_integer iterration_number){
	//For loop to check both channels
	for (int i= 0; i<CHANNEL_NUMBER; i++) {
	     //Calculate average value, based number of samples from the sum
		 average_sensor[i] = sum_sensor[i] / (float)required_sample_no;
	     //Add it to OLED sum
		 sum_sensor_oled[i] = sum_sensor_oled[i] + average_sensor[i];

		 //Check min max values
		 //If we have first iteration, oled_min is min, oled_max is max
		 if (iterration_number == 0) {
		     oled_min[i] = uart_min[i];
		     oled_max[i] = uart_max[i];
		 }
		 //If not first iteration, check if new min/max is smaller/bigger than current min/max
		 else {
		     if (uart_min[i] < oled_min[i]) {
		         oled_min[i] = uart_min[i];
		     }

		     if (uart_max[i] > oled_max[i]) {
		         oled_max[i] = uart_max[i];
		     }
		 }
	 }
}



//Function to display the screen while waiting for measurements/start timers
void statechart_start_program( Statechart* handle){
	//Heavy blocking write operations, stop timer before
	HAL_TIM_Base_Stop_IT(&htim6);

	SSD1306_Clear(); // Clear screen
	 //For loop to loop over both channels
	 for (int i = 0; i < CHANNEL_NUMBER; i++)
	 {
	     char ch_label[20];
	     //Format current channel number
	     sprintf(ch_label, "Ch%d:  -------  lx", i + 1);

	     SSD1306_GotoXY(5, 0+i*33); //Go to the required position, based on channel number
	     SSD1306_Puts(ch_label, &Font_7x10, 1); //Place text

	     SSD1306_GotoXY(5, 10+i*33); //Go to the required position, based on channel number
	     SSD1306_Puts("Min:  -------  lx", &Font_7x10, 1); //Place text

	     SSD1306_GotoXY(5, 20+i*33); //Go to the required position, based on channel number
	     SSD1306_Puts("Max:  -------  lx", &Font_7x10, 1); //Place text
	 }
	 SSD1306_UpdateScreen(); // update screen
	 __HAL_TIM_SET_COUNTER(&htim6, 0); //Reset counter (should be reset to 0 anyways, but good practice after update)
	 HAL_TIM_Base_Start_IT(&htim6); //Restart counter after writing is done
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
  huart2.Init.BaudRate = 460800;
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

//I2C DMA CALLBACK
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == I2C1) {
	    i2c_done[0]=1; //If channel 1 done, set flag
	}
	if (hi2c->Instance == I2C2) {
        i2c_done[1]=1; //If channel 2 done, set flag
	}
    if (i2c_done[0]==1 && i2c_done[1]==1) {
		//If both channels done, reset flags
    	i2c_done[0]=i2c_done[1]=0;
    	//Raise I2C callback event in statechart
    	statechart_raise_i2c_callback_sensors(&sc_handle);
    }
}

//TIM6 reload callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
    	uart_counter++;
    	//Raise timer interrupt event in statechart
    	statechart_raise_timer_interrupt(&sc_handle);
    }
}

//Button external interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == B1_Pin) { //If user button pin
		uint32_t now = HAL_GetTick(); // Variable to get current time

		//If time elapsed since last interrupt more than DEBOUNCE_MS
		if (now - last_button_time > DEBOUNCE_MS) {
			last_button_time = now;//Update last time
			statechart_raise_user_button(&sc_handle); //Raise user_button event in statechart
		}
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
