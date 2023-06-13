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
#include "arm_math.h"

#include "incl.h"
#include "InclMathFix.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define gra_to_rad(a) (a) * 3.141592653f /180.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */

static float32_t angls_grad[6];
static float32_t angls_rad[3];
static float32_t anglsmm[3];
static float32_t rotangl[3];
static float32_t rotangl2;
static sInclData incl;

static float32_t sinangl[3];
static float32_t sinangl_1[3];
static float32_t sinangl_2[3];
static float32_t sinangl_3[3];

static float32_t sinangl_21[3];
static float32_t sinangl_31[3];

static float32_t angls[6];
static float32_t g[3];
static float32_t n[3];
static float32_t gcal[3];
static float32_t gnew[3];
static float32_t gnew1[3];
static float32_t gnew2[3];
//static float32_t gcal[3];
//static float32_t gnew[3];
static float32_t G[3] = {0.0f,0.0f,1.0f};
static float32_t MX[3][3];
static float32_t MY[3][3];
static float32_t MZ[3][3];
static float32_t MZ2[3][3];

//МАТРИЦЫ
//........//
static float32_t rotMatrixVector[9];
static  arm_matrix_instance_f32 rotr;
static float32_t rotMatrixVector2[9];
static  arm_matrix_instance_f32 rotr2;
//

static volatile uint32_t pres_pc13; //Нажатие 
static volatile uint32_t prev_pc13 = GPIO_IDR_ID13; //Предыдущее значение кнопки
static volatile uint32_t cnt_pc13 = 0; //счетчик
static volatile uint32_t timer_intr_counter = 0;
static volatile uint8_t key_press_counter = 0;
static volatile uint8_t key_press = 0;
static volatile uint8_t* ptr_key_press = &key_press;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
static void debounce(volatile uint32_t *  pres_px, volatile uint32_t *  cnt_px, volatile uint32_t *  prev_px, uint32_t GPIOx_IDR);

void derotX(float32_t M[3][3], float32_t phi);
void derotY(float32_t M[3][3], float32_t theta);
void derotZ(float32_t M[3][3], float32_t psi);
void rotvect(float32_t M[3][3], float32_t gin[3], float32_t gout[3]);


void crossProduct3(const float32_t a[3], const float32_t b[3], float32_t result[3]);
void normalize3(const float32_t vector[3], float32_t res[3]);
void rotationMatrix3(const float32_t* a, const float32_t* b, float32_t* rotMatrix);



void 	HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim){
  if(htim->Instance == TIM7){
    TIM7 -> SR &= ~TIM_SR_UIF;
    timer_intr_counter++;
    debounce(&pres_pc13, &cnt_pc13, &prev_pc13, GPIOC->IDR);
    if (pres_pc13)
    {
      pres_pc13 = 0;
      key_press_counter++;
      timer_intr_counter = 0;
    }
    else if(timer_intr_counter>=50 && key_press_counter != 0)
    { 
      *ptr_key_press = key_press_counter;
      key_press_counter = 0;
      timer_intr_counter = 0;
    }
  
  }
}
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
  MX_TIM7_Init();
  
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
  GPIOC->MODER &= 0xF3FFFFFF;
  GPIOC->PUPDR |= 0x04000000;
  
  
  NVIC_EnableIRQ(TIM7_IRQn);
  HAL_TIM_Base_Start_IT(&htim7);
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  Incl_init();
   
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */  

    //получаем значения с датчика
    Incl_Data_ANGL(angls_grad);
    for(uint32_t i = 0; i < 3; i++){
      angls_rad[i] = gra_to_rad(angls_grad[i]);
      anglsmm[i] = tanf(angls_rad[i]) * 1600;
//      g[i] = angls_grad[i+3];
      sinangl[i] = arm_sin_f32(angls_rad[i]);
    }
    
    
    //расчет углов через акселерометр
//    angl[0] = atan2f(g[0],sqrtf(powf(g[1],2)+powf(g[2],2)));
//    angl[1] = atan2f(g[1],sqrtf(powf(g[0],2)+powf(g[2],2)));
//    angl[2] = atan2f(g[2],sqrtf(powf(g[0],2)+powf(g[1],2)));
    
    // события по кнопке
    if (key_press == 1){
      key_press = 0; 
      
//      //вращение вокруг оси X на угол Y
//      rotangl[1] = angls_rad[1];
//      derotX(MX, rotangl[1]); 
//      rotvect(MX,sinangl,sinangl_1);
//      n[0] = sqrtf(pow(sinangl_1[0],2)+pow(sinangl_1[1],2)+pow(sinangl_1[2],2));
//      
//      //вращение вокруг оси Y на угол Xштрих
//      rotangl[0] = asinf(sinangl_1[0]);
//      derotY(MY, rotangl[0]);
//      rotvect(MY,sinangl_1,sinangl_2);
//      n[1] = sqrtf(pow(sinangl_2[0],2)+pow(sinangl_2[1],2)+pow(sinangl_2[2],2));
      Incl_Data_Init_1(angls_rad,&incl);
    }
    else if (key_press == 2){
      key_press = 0; 
      Incl_Data_Init_2(angls_rad,&incl);
    }
    fixangl(angls_rad, &incl);
//    rotvect(MX,sinangl,sinangl_1);
//    rotvect(MY,sinangl_1,sinangl_2);
//    
//    if (key_press == 2){
//      key_press = 0;
//    if(sinangl_2[0] > sinangl_2[1])
//    rotangl[2] = atan2f(sinangl_2[1],sinangl_2[0]);
//    else
//    rotangl2 = 3.141592653f/2.0f - atan2f(sinangl_2[0],sinangl_2[1]);
//    
//    derotZ(MZ, rotangl[2]);
//    derotZ(MZ2, rotangl2);
//    }
//    rotvect(MZ,sinangl_2,sinangl_3);
//    rotvect(MZ2,sinangl_2,sinangl_31);
    angls[0] = asin(incl.data_out[0]) * 180.0f/3.14;
    angls[1] = asin(incl.data_out[1]) * 180.0f/3.14;
    angls[2] = asin(incl.data_out[2]) * 180.0f/3.14;
//    angls[3] = asin(sinangl_31[0]) * 180.0f/3.14;
//    angls[4] = asin(sinangl_31[1]) * 180.0f/3.14;
//    angls[5] = asin(sinangl_31[2]) * 180.0f/3.14;     
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
  htim7.Init.Prescaler = 63999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10;
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
static void debounce(volatile uint32_t * pres_px, volatile uint32_t * cnt_px, volatile uint32_t * prev_px, uint32_t GPIOx_IDR)
{
    uint32_t cur_px = GPIOx_IDR;
	if (cur_px != *prev_px) 
    {
      (*cnt_px)++;
      if (*cnt_px >= 4)
      {
        *prev_px = cur_px;
        *cnt_px = 0;
        if(cur_px == 0)
        {
          *pres_px = 1;
        }
      }
    }
    else
    {
        *cnt_px = 0;
    }
}


//***************************
//***************************

void derotX(float32_t M[3][3], float32_t phi)
{
    float32_t m11 = 1.0f;
    float32_t m12 = 0.0f;
    float32_t m13 = 0.0f;
    float32_t m21 = 0.0f;
    float32_t m22 = arm_cos_f32(phi);
    float32_t m23 = -arm_sin_f32(phi);
    float32_t m31 = 0.0f;
    float32_t m32 = arm_sin_f32(phi);
    float32_t m33 = arm_cos_f32(phi);
  
    M[0][0] = m11;
    M[0][1] = m12;
    M[0][2] = m13;
    M[1][0] = m21;
    M[1][1] = m22;
    M[1][2] = m23;
    M[2][0] = m31;
    M[2][1] = m32;
    M[2][2] = m33;
}


void derotY(float32_t M[3][3], float32_t theta)
{
    float32_t m11 = arm_cos_f32(theta);
    float32_t m12 = 0.0f;
    float32_t m13 = -arm_sin_f32(theta);
    float32_t m21 = 0.0f;
    float32_t m22 = 1.0f;
    float32_t m23 = 0.0f;
    float32_t m31 = arm_sin_f32(theta);
    float32_t m32 = 0.0f;
    float32_t m33 = arm_cos_f32(theta);
  
    M[0][0] = m11;
    M[0][1] = m12;
    M[0][2] = m13;
    M[1][0] = m21;
    M[1][1] = m22;
    M[1][2] = m23;
    M[2][0] = m31;
    M[2][1] = m32;
    M[2][2] = m33;
}

void derotZ(float32_t M[3][3], float32_t psi)
{
    float32_t m11 = arm_cos_f32(psi);
    float32_t m12 = arm_sin_f32(psi);
    float32_t m13 = 0.0f;
    float32_t m21 = -arm_sin_f32(psi);
    float32_t m22 = arm_cos_f32(psi);
    float32_t m23 = 0.0f;
    float32_t m31 = 0.0f;
    float32_t m32 = 0.0f;
    float32_t m33 = 1.0f;
  
    M[0][0] = m11;
    M[0][1] = m12;
    M[0][2] = m13;
    M[1][0] = m21;
    M[1][1] = m22;
    M[1][2] = m23;
    M[2][0] = m31;
    M[2][1] = m32;
    M[2][2] = m33;
}

// Функция для выполнения де-ротации вектора данных с использованием де-ротационной матрицы
void rotvect(float32_t M[3][3], float32_t gin[3], float32_t gout[3])
{
    gout[0] = M[0][0] * gin[0] + M[0][1] * gin[1] + M[0][2] * gin[2];
    gout[1] = M[1][0] * gin[0] + M[1][1] * gin[1] + M[1][2] * gin[2];
    gout[2] = M[2][0] * gin[0] + M[2][1] * gin[1] + M[2][2] * gin[2];
}

void crossProduct3(const float32_t a[3], const float32_t b[3], float32_t result[3]) {
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

void normalize3(const float32_t vector[3], float32_t res[3]) {
  
    float32_t dot_prod;
    arm_dot_prod_f32(&vector[0], &vector[0], 3, &dot_prod);
    arm_sqrt_f32(dot_prod,&dot_prod);
    arm_scale_f32(&vector[0], (1.0f / dot_prod), &res[0], 3);
}


void rotationMatrix3(const float32_t* a, const float32_t* b, float32_t* rotMatrixV) {
  
    float32_t cross_res[3];
    float32_t cross_res_ABS;
    float32_t dotProduct;
    float32_t crossProduct; 
    
    float32_t norma[3];
    float32_t normb[3];
  
    float32_t I_matrixVector[9] = {
     1.0f,    0.0f,    0.0f,
     0.0f,    1.0f,    0.0f,
     0.0f,    0.0f,    1.0f
    };
    float32_t AMA_matrixVector[9];
    float32_t sumAB_matrixVector[9];
  
    arm_matrix_instance_f32 A_matrix;
    arm_matrix_instance_f32 AMA_matrix;
    arm_matrix_instance_f32 I_matrix;
    arm_matrix_instance_f32 sumIA_matrix;
    arm_matrix_instance_f32 rotMatrix;
    
    arm_mat_init_f32(&I_matrix,3,3, &I_matrixVector[0]);
    arm_mat_init_f32(&AMA_matrix,3,3, &AMA_matrixVector[0]);
    arm_mat_init_f32(&sumIA_matrix,3,3, &sumAB_matrixVector[0]);

  

    // Normalize vectors
  
    normalize3(&a[0],&norma[0]);
    normalize3(&b[0],&normb[0]);
    
    crossProduct3(norma,normb,cross_res);
    
    float32_t A_matrixVector[9] = {
      0.0f,           -cross_res[2],   cross_res[1],
      cross_res[2],   0.0f,            -cross_res[0],
     -cross_res[1],  cross_res[0],    0.0f
    };  
    arm_mat_init_f32(&A_matrix,3,3, &A_matrixVector[0]);
    
    // pow(A,2)
    arm_mat_mult_f32(&A_matrix, &A_matrix, &AMA_matrix);

    // Compute dot product
    arm_dot_prod_f32(a,b,3,&dotProduct);
    dotProduct =1 - dotProduct;
    
    // Compute cross product  
    arm_dot_prod_f32(&cross_res[0], &cross_res[0], 3, &cross_res_ABS);
    
//    crossProduct = 1.0f / cross_res_ABS;
    
    //add matrix
    
    arm_mat_add_f32(&I_matrix,&A_matrix,&sumIA_matrix);
    arm_mat_scale_f32(&AMA_matrix,dotProduct / cross_res_ABS,&AMA_matrix);
    
    //result
     arm_mat_init_f32(&rotMatrix,3,3,rotMatrixV);
     arm_mat_add_f32(&AMA_matrix,&sumIA_matrix, &rotMatrix);
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
