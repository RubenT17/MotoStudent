/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */


/*
 * **********************************************************************************
 *
 *  Created on: 	00.00.0000
 *  Author: 		Rubén Torres Bermúdez <rubentorresbermudez@gmail.com>
 *  Organism:		Sevilla Racing. Universidad de Sevilla.
 *
 *	Description:
 *		This C source file provides the Sevilla Racing STM32F4x project to be included
 *		into a electric racing bike for MotoStudent International Competition.
 *		Sevilla Racing is representing the motorcycle team of Universidad de Sevilla.
 *
 *		MPU6050 library's credits belongs to its respective owner.
 *
 *		Copyright (C) 2022 Rubén Torres Bermúdez
 *
 * **********************************************************************************
 */



/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "RT_ds18b20.h"
#include "mpu6050.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//++++++++++++++++++++++++++++ ANALOG TO DIGITAL CONVERTER ++++++++++++++++++++++++++++
#define Avg_Slope .0025
#define V25 0.76


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

//++++++++++++++++++++++++++++ MPU6050 ++++++++++++++++++++++++++++
MPU6050_t MPU6050; //Structure type MPU6050_t name MPU6050

//++++++++++++++++++++++++++++ DS18B20 ++++++++++++++++++++++++++++
volatile uint8_t pin_1wire = 0;
typedef struct
{
	float Pack1;
	float Pack2;
	float Pack3;
	float Pack4;
}DS18B20_t;
DS18B20_t Temp;
uint8_t dir1[] = {0x28,0xB4,0x0C,0x94,0x97,0x0C,0x03,0xF6};


//++++++++++++++++++++++++++++ ANALOG TO DIGITAL CONVERTER ++++++++++++++++++++++++++++
typedef struct
{
	uint32_t Channel1;
	uint32_t Channel2;
	uint32_t Channel3;
	uint32_t Channel4;
	float Temp_uC;
} ADC_value_t;
ADC_value_t ADC1_value;

//++++++++++++++++++++++++++++ PWM MEASURE ++++++++++++++++++++++++++++
typedef struct
{
	uint32_t preICValue;
	uint32_t ICValue;
	uint32_t Frequency;
	float Duty;
	float Rf;
} PWM_measure_t;
PWM_measure_t IMD;

//++++++++++++++++++++++++++++ OTHERS VAR ++++++++++++++++++++++++++++
uint8_t flag_TIM11 = 0;
uint8_t dataRx [8] = {0};
uint8_t flag_app = 0;
typedef struct
{
	uint8_t DS18B20;
	uint8_t MPU6050;
	uint8_t IMD;
	uint8_t BMS;
	uint8_t Contactor;
} flag_Errors_t;
flag_Errors_t flag_error;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM11_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */

void delayMicro (uint32_t delay);
uint64_t millis(void);
void read_AllTemp (void);
void read_ADC1 (void);
uint8_t info_user_transmit (void);
uint8_t info_app_transmit (void);
void reset_flags ();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//################################## MPU6050 ##################################


//################################## DELAY MICROSECONDS ##################################
/**
 * @brief	Delay us.
 * @param 	Delay in microseconds.
 * @return  None.
 * @note 	Sólo cuenta hasta 65535/f microsegundos. Siendo f la frecuencia en MHz.
 * 		 	Asegurarse de inicializar  HAL_TIM_Base_Start(&htim).
 * 		 	Modificar la frecuencia del timer en MHz (pasado por prescaler).
 */
void delayMicro (uint32_t delay)
{
#define RT_FREQUENCY_delayMicro 100
	__HAL_TIM_SET_COUNTER(&htim10, 0);
	while (__HAL_TIM_GET_COUNTER(&htim10)<(delay*RT_FREQUENCY_delayMicro));
}



//################################## MILLIS FUNCTION ##################################
/**
  * @brief	millis().
  * @param 	None.
  * @return Time elapse since timer init or set counter.
  * @note 	Sólo cuenta hasta 65535/f microsegundos. Siendo f la frecuencia en MHz.
  * 		 	Asegurarse de inicializar  HAL_TIM_Base_Start(&htim).
  * 		 	Modificar la frecuencia del timer en KHz (pasado por prescaler).
  */
uint64_t millis(void)
{
#define RT_FREQUENCY_millis 2
    return (__HAL_TIM_GET_COUNTER(&htim9)/RT_FREQUENCY_millis);
}


//################################## DS18B20 ##################################
/**
  * @brief	Read all values of DS18B20 sensor temperature.
  * @param 	None
  * @return None.
  */
void read_AllTemp (void)
{
	if (!DS18B20_All_Convert(TempDS1_GPIO_Port, TempDS1_Pin))
	{
		Temp.Pack1 = DS18B20_Read_Temp(TempDS1_GPIO_Port, TempDS1_Pin, dir1);
	//	Temp.Pack3 = DS18B20_Read_Temp(TempDS1_GPIO_Port, TempDS1_Pin, dir1);
	//	Temp.Pack2 = DS18B20_Read_Temp(TempDS1_GPIO_Port, TempDS1_Pin, dir1);
	//	Temp.Pack4 = DS18B20_Read_Temp(TempDS1_GPIO_Port, TempDS1_Pin, dir1);
		if ((Temp.Pack1 == -250.0) | (Temp.Pack2 == -250.0) | (Temp.Pack3 == -250.0) | (Temp.Pack4 == -250.0))
		{
			flag_error.DS18B20 = 1;
		}

	}
		else	flag_error.DS18B20 = 1;
}

//################################## ANALOG TO DIGITAL CONVERTER ##################################
/**
  * @brief	Read all values of ADC1.
  * @param 	None
  * @return None.
  */
void read_ADC1 (void)
{
	HAL_ADC_Start(&hadc1);// start the ADC

	HAL_ADC_PollForConversion(&hadc1, 1);
	ADC1_value.Channel1 = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);
	ADC1_value.Channel2 = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);
	ADC1_value.Channel3 = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);
	ADC1_value.Channel4 = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);
	ADC1_value.Temp_uC = ((3.3*HAL_ADC_GetValue(&hadc1)/4095 - V25)/Avg_Slope)+25;

	HAL_ADC_Stop(&hadc1); // stop the adc
}


//################################## SEND INFO UART2 ##################################
/**
  * @brief	Sends info to UART.
  * @param 	None.
  * @return Return 1 when transmission is completed.
  */
uint8_t info_user_transmit (void)
{
	char buffer[400] = {0};
	sprintf
	(
			buffer,
			"#########   INICIO ENVIO DE DATOS   #########\r\n\nIMD: \r\n- Frecuencia: %lu\r\n- Duty: %.1f\r\n\nGiroscopio: \r\nAx\tAy\tAz\tGx\tGy\tGz\r\n%.2f\t%.2f\t%.2f\t%.2f\t%.2f\r\n\nTemperaturas: \r\n- Temp. Pack1: %.1f\r\n- Temp. Pack2: %.1f\r\n\nValores ADC: \r\n- Canal 1: %lu\r\n- Canal 2: %lu\r\n- Canal 3: %lu\r\n- Canal 4: %lu\r\n- Temp. uC: %.1f\r\n\n#########   FIN ENVIO DE DATOS   #########\r\n\n\n",
			IMD.Frequency, IMD.Duty, MPU6050.Ax, MPU6050.Ay, MPU6050.Az, MPU6050.KalmanAngleX,
			MPU6050.KalmanAngleY, Temp.Pack1, Temp.Pack2, ADC1_value.Channel1, ADC1_value.Channel2, ADC1_value.Channel3,
			ADC1_value.Channel4, ADC1_value.Temp_uC
	);

	HAL_UART_Transmit(&huart2, buffer, strlen(buffer), 10);
	return 1;
}


/**
  * @brief	Sends info to app via UART.
  * @param 	None.
  * @return Return 1 when transmission is completed.
  */
uint8_t info_app_transmit (void)
{
	char buffer[100] = {0};
	sprintf
	(
			buffer,
			"+++++\r\n%lu\r\n%.1f\r\n%.2f\r\n%.2f\r\n%.2f\r\n%.2f\r\n%.2f\r\n%.1f\r\n%.1f\r\n%lu\r\n%lu\r\n%lu\r\n%lu\r\n%.1f\r\n*****\r\n",
			IMD.Frequency, IMD.Duty, MPU6050.Ax, MPU6050.Ay, MPU6050.Az, MPU6050.KalmanAngleX,
			MPU6050.KalmanAngleY, Temp.Pack1, Temp.Pack2, ADC1_value.Channel1, ADC1_value.Channel2, ADC1_value.Channel3,
			ADC1_value.Channel4, ADC1_value.Temp_uC
	);

	HAL_UART_Transmit(&huart2, buffer, strlen(buffer), 10);
	return 1;
}


/**
  * @brief	Restart stored flag errors.
  * @param	None.
  * @return None.
  */
void reset_flags ()
{
	flag_error = (flag_Errors_t){0};
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
  MX_I2C1_Init();
  MX_IWDG_Init();
  MX_ADC1_Init();
  MX_TIM11_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM10_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */

  //################################## INITIAL CHECKS ##################################

    HAL_Delay(500);
    HAL_IWDG_Refresh(&hiwdg);
    HAL_Delay(500);
    HAL_IWDG_Refresh(&hiwdg);
    HAL_Delay(500);
    HAL_IWDG_Refresh(&hiwdg);
    HAL_Delay(500);
    HAL_IWDG_Refresh(&hiwdg);
    HAL_Delay(500);
    HAL_IWDG_Refresh(&hiwdg);

    if (HAL_GPIO_ReadPin(IMD_OK_GPIO_Port, IMD_OK_Pin) == GPIO_PIN_RESET)
    {
  	  HAL_GPIO_WritePin(Contactor_out_GPIO_Port, Contactor_out_Pin, GPIO_PIN_RESET);
  	  flag_error.IMD = 1;
    }

    if (HAL_GPIO_ReadPin(BMS_OK_GPIO_Port, BMS_OK_Pin) == GPIO_PIN_RESET)
    {
  	  flag_error.BMS = 1;
    }

    if (HAL_GPIO_ReadPin(CONTACTOR_IN_GPIO_Port, CONTACTOR_IN_Pin) == GPIO_PIN_RESET)
    {
  	  flag_error.Contactor = 1;
    }


    //################################## SETUP PERIPHERIALS ##################################

  for (uint8_t a = 1; a<5; a++)
  {
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  HAL_Delay(300);
	  HAL_IWDG_Refresh(&hiwdg);
  }

  HAL_IWDG_Refresh(&hiwdg);

  HAL_TIM_Base_Start (&htim10);		//delayMicro ()

  HAL_TIM_Base_Start (&htim9);			//millis()

  	  if (!MPU6050_Init (&hi2c1))			//MPU6050 Incialización
  		  HAL_UART_Transmit(&huart2, (uint8_t *) "MPU6050 inicializada\r\n", strlen ("MPU6050 inicializada\r\n"), 1);
  	  else
  	  {
  		  HAL_UART_Transmit(&huart2, (uint8_t *) "MPU6050 fallo\r\n", strlen ("MPU6050 fallo\r\n"), 1);
  		  flag_error.MPU6050 = 1;
  	  }

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);	//Medidor de PWM
  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);

  HAL_UART_Receive_DMA(&huart2, dataRx, 1);		//Recepción de comandos
  HAL_UART_Receive_DMA(&huart1, dataRx, 1);		//Orden App

  HAL_UART_Transmit(&huart2, (uint8_t *) "#########   INICIALIZADO   #########\r\n\n\n", strlen ((char *) "#########   INICIALIZADO   #########\r\n\n\n"), 3);
  HAL_TIM_Base_Start_IT(&htim11);		//Interrupciones Internas
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  for (uint16_t i = 0; i < 500; i++)	//Delay con micros
		  delayMicro (100);	//Delay 100us

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 5;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_8;
  hiwdg.Init.Reload = 3999;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 50000-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 1-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 10000-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 300-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, TempDS2_Pin|TempDS1_Pin|Contactor_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_USER_Pin */
  GPIO_InitStruct.Pin = B1_USER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_USER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TempDS2_Pin */
  GPIO_InitStruct.Pin = TempDS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TempDS2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TempDS1_Pin */
  GPIO_InitStruct.Pin = TempDS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(TempDS1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IMD_OK_Pin CONTACTOR_IN_Pin BMS_OK_Pin */
  GPIO_InitStruct.Pin = IMD_OK_Pin|CONTACTOR_IN_Pin|BMS_OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Contactor_out_Pin */
  GPIO_InitStruct.Pin = Contactor_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Contactor_out_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//char buffer[30] = {0};
    if (htim->Instance==TIM11) //check if the interrupt comes from TIM11
        {
        flag_TIM11++;
        HAL_IWDG_Refresh(&hiwdg);
        __HAL_TIM_SET_COUNTER(&htim11, 0);
        switch (flag_TIM11)
        {
        case 1:
        	if(!flag_error.MPU6050){
        		MPU6050_Read_All(&hi2c1, &MPU6050);
        		HAL_IWDG_Refresh(&hiwdg);
        		break;
        	}
        	else
        	{
        		flag_TIM11++; //NO PONER BREAK PARA SALTAR
        	}

        case 2: read_AllTemp (); HAL_IWDG_Refresh(&hiwdg); break; // sprintf(buffer, "Case 2: %lu\n\r",__HAL_TIM_GET_COUNTER(&htim11)); HAL_UART_Transmit(&huart2, buffer, strlen(buffer), 1);break;

        case 3: read_ADC1 (); HAL_IWDG_Refresh(&hiwdg); break; // sprintf(buffer, "Case 4: %lu\n\r",__HAL_TIM_GET_COUNTER(&htim11)); HAL_UART_Transmit(&huart2, buffer, strlen(buffer), 1); break;

        default:
        	if(flag_app)
        		info_app_transmit();
        	flag_TIM11 = 0;
        	HAL_IWDG_Refresh(&hiwdg);
        	break;	//SI NO HAY APP, HAY UNOS MILISEGUNDOS SIN USAR
        }
        }
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if ((htim->Instance==TIM2) & (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
		IMD.ICValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
	if (IMD.ICValue !=0)
	{
		IMD.Duty = (HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) * 100.0) / IMD.ICValue;
		IMD.Frequency = 100000000 / IMD.ICValue;
	}

	if ((IMD.Frequency<12) && (IMD.Frequency>8))
		IMD.Rf = ((90*1200)/(IMD.Duty-5))-1200;

	else if ((IMD.Frequency<33) && (IMD.Frequency>27))
	{
		if (!((IMD.Duty<11) && (IMD.Duty>4)))
				flag_error.IMD = 1;
	}

	else	flag_error.IMD = 1;

	//Si va muy rápido, cambiar  --> sConfigIC.ICPrescaler = TIM_ICPSC_DIV1 -->8;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance==USART2)
	{
		if (!strcmp ((char*) dataRx, "i"))
			info_user_transmit ();

		else if (!strcmp ((char*) dataRx, "r"))
			reset_flags ();

		else if (!strcmp ((char*) dataRx, "P"))
			flag_app = 1;

		else
		{
			HAL_UART_Transmit(&huart2, dataRx, 8, 1);
			HAL_UART_Transmit(&huart2, (uint8_t *) " --> Comando no reconocido\r\n", strlen ((char *) " --> Comando no reconocido\r\n"), 5);
		}
	}

	else if (huart->Instance==USART1)
	{
		if (!strcmp ((char*) dataRx, "P"))
			flag_app = 1;

		else if (!strcmp ((char*) dataRx, "C"))
			flag_app = 0;

		else
		{
			HAL_UART_Transmit(&huart1, dataRx, 8, 1);
			HAL_UART_Transmit(&huart1, (uint8_t *) " --> Comando no reconocido\r\n", strlen ((char *) " --> Comando no reconocido\r\n"), 5);
		}
	}
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == IMD_OK_Pin) {
		if (HAL_GPIO_ReadPin(IMD_OK_GPIO_Port, IMD_OK_Pin) == GPIO_PIN_RESET)
		{
			HAL_GPIO_WritePin(Contactor_out_GPIO_Port, Contactor_out_Pin, GPIO_PIN_RESET);
			flag_error.IMD = 1;
			while(1){}
		}
	}

	else if (GPIO_Pin == BMS_OK_Pin) {
		if (HAL_GPIO_ReadPin(BMS_OK_GPIO_Port, BMS_OK_Pin) == GPIO_PIN_RESET)
		{
			flag_error.BMS = 1;
		}
	}

	else if (GPIO_Pin == CONTACTOR_IN_Pin) {
		if (HAL_GPIO_ReadPin(CONTACTOR_IN_GPIO_Port, CONTACTOR_IN_Pin) == GPIO_PIN_RESET)
		{
			flag_error.Contactor = 1;
		}
	}

	else if (GPIO_Pin == B1_USER_Pin) {
		reset_flags();
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
	HAL_UART_Transmit(&huart2, (char *) "Fallo en parametros de la linea ", strlen ((char *) "Fallo en parametros de la linea "), 1);
	HAL_UART_Transmit(&huart2, (uint8_t *) line, strlen ((uint8_t *) line), 1);

	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

