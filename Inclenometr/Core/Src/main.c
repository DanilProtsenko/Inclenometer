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
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LENGTH_SAMPLES 320
#define BLOCK_SIZE 320
#define NUM_TAPS 320
#define NUM_BLOCKS LENGTH_SAMPLES/BLOCK_SIZE
#define N 120
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

//static uint8_t data_rx[4] = {0};
static uint8_t data_m[4]={0};

typedef struct Incl_Data{
  uint32_t Read_ACC_X;
  uint32_t Read_ACC_Y;
  uint32_t Read_ACC_Z;
  uint32_t Read_STO;
  uint32_t Enable_ANGLE_outputs;
  uint32_t Read_ANG_X ;
  uint32_t Read_ANG_Y;
  uint32_t Read_ANG_Z;
  uint32_t Read_Temperature;
  uint32_t Read_Status_Summary;
  uint32_t Read_ERR_FLAG1;
  uint32_t Read_ERR_FLAG2;
  uint32_t Read_CMD;
  uint32_t Change_to_mode_1; 
  uint32_t Change_to_mode_2; 
  uint32_t Change_to_mode_3; 
  uint32_t Change_to_mode_4; 
  uint32_t Set_power_down_mode; 
  uint32_t Wake_up_from_power_down_mode; 
  uint32_t SW_Reset; 
  uint32_t Read_WHOAMI; 
  uint32_t Read_SERIAL1; 
  uint32_t Read_SERIAL2;
  uint32_t Read_current_bank;
  uint32_t Switch_to_bank_0;
  uint32_t Switch_to_bank_1;
} sIncl;

static sIncl hincl1 = {
  0x040000F7,
  0x080000FD,
  0x0C0000FB,
  0x100000E9,
  0xB0001F6F,
  0x240000C7,
  0x280000CD,
  0x2C0000CB,
  0x140000EF,
  0x180000E5,
  0x1C0000E3,
  0x200000C1,
  0x340000DF,
  0xB400001F,
  0xB4000102,
  0xB4000225,
  0xB4000338,
  0xB400046B,
  0xB400001F,
  0xB4002098,
  0x40000091,
  0x640000A7,
  0x680000AD,
  0x7C0000B3,
  0xFC000073,
  0xFC00016E
};

//static arm_fir_instance_q15 firStruct;
//static arm_fir_instance_f32 firStructFloat;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t CalculateCRC(uint32_t Data);
static uint8_t CRC8(uint8_t BitValue, uint8_t CRCSPI);
void Spi_init(void);
//static void delay_temp(uint32_t t);
uint16_t Data_spi(sIncl* hincl, uint32_t command, uint32_t delay_ms);
void movAverage(void);
void movAverageFloat(void);
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
  Spi_init();
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

    for(uint32_t i = 0; i < N; i++){
    temp[0] = ((float)filter_x(Data_spi(&hincl1, hincl1.Read_ANG_X, 1))/ 16384.0f)* 90.0f;
    temp[1] = ((float)filter_y(Data_spi(&hincl1, hincl1.Read_ANG_Y, 1))/ 16384.0f)* 90.0f;
    temp[2] = ((float)filter_z(Data_spi(&hincl1,hincl1.Read_ANG_Z, 1))/ 16384.0f)* 90.0f;
    }
       
    for(uint32_t i = 0; i < 3; i++){
    angl[i] = temp[i];
    anglmm[i] = tan(gra_to_rad(angl[i])) * 1600;
    }
    
//    g[0] = (Data_spi(&hincl1, hincl1.Read_ACC_X, 1) / 6000.0f);
//    g[1] = (Data_spi(&hincl1, hincl1.Read_ACC_Y, 1) / 6000.0f);
//    g[2] = (Data_spi(&hincl1, hincl1.Read_ACC_Z, 1) / 6000.0f);
    
//    point[0] = (atan2(g[0], 0.1) * 180.0f/3.1415926535f);
//    point2[0] = tan(gra_to_rad(angl[0]));
//    point2[1] = tan(gra_to_rad(angl[0])) * tan(gra_to_rad(angl[1]));
//    point2[2] = tan(gra_to_rad(angl[0])) * tan(gra_to_rad(angl[1]))*tan(gra_to_rad(angl[2]));

//    
//    point[0] = acos(cos)
    
//    point[1] = (cos(gra_to_rad(angl[1])) - sin(gra_to_rad(angl[1])));
//    point[0] = atan2f(1,sqrt(point[1]*point[1]+point[2]*point[2])) * 180.0f/3.1415926535f;
    
//     
//    
//    point[0] = tan(gra_to_rad(angl[0]))*(cos(gra_to_rad(angl[1])) * cos(gra_to_rad(angl[2])));
////    
//    point[1] = g[0]*((sin(gra_to_rad(angl[0]))*sin(gra_to_rad(angl[1]))*cos(gra_to_rad(angl[2]))+sin(gra_to_rad(angl[2]))*cos(gra_to_rad(angl[0]))) +
//    g[1]*(-sin(gra_to_rad(angl[0]))*sin(gra_to_rad(angl[1]))*sin(gra_to_rad(angl[2]))+cos(gra_to_rad(angl[0]))*cos(gra_to_rad(angl[2])))+
//    g[2]*(-sin(gra_to_rad(angl[0]))*cos(gra_to_rad(angl[1]))));
////    
//    point[2] = g[0]*(sin(gra_to_rad(angl[0]))*sin(gra_to_rad(angl[2]))+(-sin(gra_to_rad(angl[1]))*cos(gra_to_rad(angl[0]))*cos(gra_to_rad(angl[2])))) +
//    g[1]*(sin(gra_to_rad(angl[0]))*cos(gra_to_rad(angl[2]))+(sin(gra_to_rad(angl[1]))*sin(gra_to_rad(angl[2]))*cos(gra_to_rad(angl[0]))))+
//    g[2]*(cos(gra_to_rad(angl[0]))*cos(gra_to_rad(angl[1])));
//    
//    point[1] = (sin(gra_to_rad(angl[0]))*sin(gra_to_rad(angl[1]))*cos(gra_to_rad(angl[2])));
//    point[2] = tan(gra_to_rad(angl[0]))*sin(gra_to_rad(angl[0]))*sin(gra_to_rad(angl[2]));
    
//    angl1[0] = (atan2(sin(gra_to_rad(angl[0])),point[0] )* 180.0f/3.1415926535f);
//      angl1[0] = (atan2(sin(gra_to_rad(angl[0])), point[0])* 180.0f/3.1415926535f);
//      angl1[0] = (asin(sin(gra_to_rad(angl[0])))* 180.0f/3.1415926535f);

//    angl1[1] = (atan(point[0]/point[1]) * 180.0f/3.1415926535f);
//    angl1[2] = (atan(point[1]/point[2]) * 180.0f/3.1415926535f);
//    angl1[2] = a(fabs(point[0]*1 + point[1]*1 + point[2] * 0)/(sqrt(point[2]*point[2]+point[1]*point[1]+point[0]*point[0])*sqrt(1+1+1))) * 180.0f/3.1415926535f;
//    angl1[2] = atan2(point[0],sqrt(point[2]*point[2]+point[1]*point[1]));
    
//    angl1[1] = asin(fabs(point[0]*0 + point[1]*1 + point[2] * 1)/(sqrt(point[2]*point[2]+point[1]*point[1]+point[0]*point[0])*sqrt(0+1+1))) * 180.0f/3.1415926535f;
    
//    angl1[0] = angl[0]+ angl1[2];
    
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

uint8_t CalculateCRC(uint32_t Data)
{
  uint8_t BitIndex;
  uint8_t BitValue;
  uint8_t CRCSPI;
  CRCSPI = 0xFF;
  for (BitIndex = 31; BitIndex > 7; BitIndex--)
  {
  BitValue = (uint8_t)((Data >> BitIndex) & 0x01);
  CRCSPI = CRC8(BitValue, CRCSPI);
  }
  CRCSPI = (uint8_t)~CRCSPI;
  return CRCSPI;
}
static uint8_t CRC8(uint8_t BitValue, uint8_t CRCSPI)
{
  uint8_t Temp;
  Temp = (uint8_t)(CRCSPI& 0x80);
  if (BitValue == 0x01)
  {
  Temp ^= 0x80;
  }
  CRCSPI <<= 1;
  if (Temp > 0)
  {
  CRCSPI ^= 0x1D;
  }
  return CRCSPI;
}

//***************************
//***************************

//static void delay_temp(uint32_t t){
//  uint32_t i; 
//  for(i = 0; i<t*128000; i++){
//    __NOP();
//    }
//}

//***************************
//***************************

uint16_t Data_spi(sIncl* hincl, uint32_t command, uint32_t delay_ms){
  
  uint16_t temp;
  uint8_t  data_tx[4];
  uint8_t data_rx[4];
  
  data_tx[3] = (command & 0xFF);
  data_tx[2] = (command>>8) & 0xFF;
  data_tx[1] = (command>>16) & 0xFF;
  data_tx[0] = (command>>24) & 0xFF;
  
  if ((command == hincl->Read_ANG_X) || (command == hincl->Read_ANG_Y) || (command == hincl->Read_ANG_Z)){
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
      HAL_SPI_Transmit(&hspi1, data_tx, 4, 0x100);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
      HAL_Delay(delay_ms);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi1, data_tx, data_rx, 4, 0x100);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
      HAL_Delay(delay_ms);
      
      temp = (data_rx[1] << 8) + data_rx[2];
      return temp;
    }
  }else if((command == hincl->Read_ACC_X) || (command == hincl->Read_ACC_Y) || (command == hincl->Read_ACC_Z)){
     if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
      HAL_SPI_Transmit(&hspi1, data_tx, 4, 0x100);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
      HAL_Delay(delay_ms);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi1, data_tx, data_rx, 4, 0x100);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
      HAL_Delay(delay_ms);
      
      temp = (data_rx[1] << 8) + data_rx[2];
      return temp;
     }
   } else {
     if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET){
       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
       HAL_SPI_Transmit(&hspi1, data_tx, 4, 0xffff);
       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
       HAL_Delay(delay_ms);
     }
   }
   
   }
  
//***************************
//***************************

void Spi_init(void){
  //выход из спящего режима
  Data_spi(&hincl1, hincl1.Wake_up_from_power_down_mode, 1);
  //Сброс настроек по умолчанию
  Data_spi(&hincl1, hincl1.SW_Reset, 1);
  //выбор режима
  Data_spi(&hincl1, hincl1.Change_to_mode_1, 1);
  //включить измерения
  Data_spi(&hincl1, hincl1.Enable_ANGLE_outputs, 25);
  //прочитать статус
  Data_spi(&hincl1, hincl1.Read_Status_Summary, 1);
  Data_spi(&hincl1, hincl1.Read_Status_Summary, 1);
}

//***************************
//***************************

//void movAverage(void){
//  q15_t *input, *output;
//  uint32_t i;
//  
//  input = &dataInput[0];
//  output = &dataOutput[0];
//  
//  for(i = 0; i < NUM_BLOCKS; i++){
//    arm_fir_q15(&firStruct, input + (i * BLOCK_SIZE), output + (i * BLOCK_SIZE), BLOCK_SIZE);
//  }
//}

//void movAverageFloat(void){
//  float32_t *input, *output;
//  uint32_t i;
//  
//  input = &dataInputFloat[0];
//  output = &dataOutputFloat[0];
//  
//  for(i = 0; i < NUM_BLOCKS; i++){
//    arm_fir_f32(&firStructFloat, input + (i * BLOCK_SIZE), output + (i * BLOCK_SIZE), BLOCK_SIZE);
//  }
//}

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
