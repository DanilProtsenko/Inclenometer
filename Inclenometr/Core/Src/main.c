/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "math.h"
#include "arm_math.h"

#include "incl.h"

/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N 32
#define gra_to_rad(a) a * 3.1415926535f /180.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

//static q15_t dataInput[LENGTH_SAMPLES];
//static q15_t dataOutput[LENGTH_SAMPLES];
//static q15_t firState[BLOCK_SIZE+NUM_TAPS];
//static q15_t firCoeffs[NUM_TAPS];

//static float32_t firCoeffsFloat[NUM_TAPS];
//static float32_t dataOutputFloat[LENGTH_SAMPLES];
//static float32_t dataInputFloat[LENGTH_SAMPLES];
//static float32_t firStateFloat[BLOCK_SIZE+NUM_TAPS];

static float32_t angl[3];
static float32_t angl1[3];
static float32_t anglmm[3];

static float32_t g[3];

static float32_t point[3];
static float32_t point2[3];

static float32_t temp[3];

static float32_t tempx2;
static float32_t tempz2;
static float32_t tempy2;

static float32_t X;
static float32_t Y;
static float32_t Z;

static uint8_t CRCSPI;

//static uint8_t data_rx[4] = {0};
static uint8_t data_m[4]={0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

uint16_t filter_x(uint16_t x);
uint16_t filter_z(uint16_t x);
uint16_t filter_y(uint16_t x);
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
//  sIncl* s;
//  uint32_t i = s->Read_ACC_X ;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  
  /* USER CODE END 2 */
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  arm_fir_init_q15(&firStruct, NUM_TAPS, &firCoeffs[0], &firState[0], BLOCK_SIZE);
//  arm_fir_init_f32(&firStructFloat, NUM_TAPS, &firCoeffsFloat[0], &firStateFloat[0], BLOCK_SIZE);
//  for(uint32_t i = 0; i < NUM_TAPS; i++){
//     firCoeffsFloat[i] = 1.0f/(NUM_TAPS+1.0f); 
//    }
  Incl_init();
  
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//    for(uint32_t i = 0; i < LENGTH_SAMPLES; i++){
//      dataInputFloat[i] = Data_spi(0x240000C7);
//    }
//    delay_temp(20000);
//    movAverageFloat();
//    for(uint32_t i = 0; i < LENGTH_SAMPLES; i++){
//      //anglx[i] = (dataOutputFloat[i] / 16384.0f) * 90.0f;
//      //anglx1[i] = ((float)dataInput[i] / 16384.0f) * 90.0f;
//      anglx[i] = tan(dataOutputFloat[i] * 3.1415926535 /180.0f) * 1600;
//    }

    temp[0] = ((float)filter_x(Incl_Data_SPI(INCL_READ_ANG_X, 1))/ 16384.0f)* 90.0f;
    temp[1] = ((float)filter_y(Incl_Data_SPI(INCL_READ_ANG_Y, 1))/ 16384.0f)* 90.0f;
    temp[2] = ((float)filter_z(Incl_Data_SPI(INCL_READ_ANG_Z, 1))/ 16384.0f)* 90.0f;
       
    for(uint32_t i = 0; i < 3; i++){
    angl[i] = temp[i];
    anglmm[i] = tan(gra_to_rad(angl[i])) * 1600;
    }
    
    
    
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  
  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//***************************
//***************************

uint16_t filter_x(uint16_t x)
{
  static uint32_t n;
  static uint32_t m[N];
  static uint32_t y;
  y +=x-m[n];
  m[n]=x;
  n=(n+1)%N;
  
  return y/N;
  
}

uint16_t filter_z(uint16_t x)
{
  static uint32_t n;
  static uint32_t m[N];
  static uint32_t y;
  y +=x-m[n];
  m[n]=x;
  n=(n+1)%N;
  
  return y/N;
  
}

uint16_t filter_y(uint16_t x)
{
  static uint32_t n;
  static uint32_t m[N];
  static uint32_t y;
  y +=x-m[n];
  m[n]=x;
  n=(n+1)%N;
  
  return y/N;
  
}
//***************************
//***************************

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
