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

typedef struct {
    float32_t dataIn[3];     // Входные данные датчика, углы в радианах
    float32_t dataOut[3];    // Значения синусов скорректированных углов в радианах, чтобы получить угол, используйте функцию арксинуса
    float32_t rotAngles[3];  // Углы на которые вращаем
    float32_t dataM[9];      // Данные матрицы поворота
} sInclData;

   
void inclDataInit1(float32_t* inputAngles, sInclData* sIncl);
void inclDataInit2(float32_t* inputAngles, sInclData* sIncl);
static void dataMxInit(float32_t* pData, float32_t phi);    
static void dataMxInit(float32_t* pData, float32_t phi);
static void dataMyInit(float32_t* pData, float32_t theta);
static void dataMzInit(float32_t* pData, float32_t psi);
   
#endif /* INCLMATHFIX_H_ */
/**
 *
 * End of file.
 */
