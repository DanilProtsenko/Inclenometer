/******************************************************************************
 * @file     InclMathFix.h
 * @brief    Публичный загловок для InclMathFix, содержит объявления функций для исправления кривого расположения инелинометра
 * необходимо подключить библиотеку DSP CMSIS ("arm_math.h")
 * @version  V1.0
 * @date     08 Июнь 2023
 * Собирался на : STM32L476RGTx c инклинометром SCL3300-D01
 ******************************************************************************/

#ifndef __INCLMATHFIX_H
#define __INCLMATHFIX_H
#include "arm_math.h"

   typedef struct{
     //входные данные датчика, углы в радианах
     float32_t data_in[3];
     //значения синусов скорректированных углов в радианах, чтобы получить угол используйте функцию арксинуса
     float32_t data_out[3];
     //углы на которые вращаем
     float32_t rot_angls[3];
     //данные матрицы поворота
     float32_t dataM[9];
   }sInclData;
   
  void fixangl(float32_t* input_angls, sInclData* sStruct);
  void Incl_Data_Init_1(float32_t* pData, sInclData* sStruct);
  void Incl_Data_Init_2(float32_t* pData, sInclData* sStruct);    
  static void dataMx_init(float32_t* pData, float32_t phi);
  static void dataMy_init(float32_t* pData, float32_t theta);
  static void dataMz_init(float32_t* pData, float32_t psi);
   
#endif /* INCLMATHFIX_H_ */
/**
 *
 * End of file.
 */
