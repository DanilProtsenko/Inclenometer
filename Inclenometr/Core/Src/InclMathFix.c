/******************************************************************************
 * @file InclMathFix.c
 * @brief Содержит определения функций для исправления неправильного расположения инклинометра SCL3300-D01.
 * @version V1.0
 * @date 08 Июнь 2023
 * @platform STM32L476RGTx с использованием инклинометра SCL3300-D01
******************************************************************************/
/**

Собрано на: STM32L476RGTx с использованием инклинометра SCL3300-D01.
*/
 
#include "InclMathFix.h"
 
/**
 * @brief   Перемножает вектор из 3 элементов на матрицу поворота 3x3.
 * @note    Используйте после инициализации матрицы поворота (функции inclDataInit1 и inclDataInit2).
 *
 * @param[in]   inputAngles     Указатель на первый элемент массива, состоящего из трех углов в радианах.
 * @param[out]  outputData      Указатель на структуру данных sInclData датчика.
 */
void fixAngles(float32_t* inputAngles, sInclData* sIncl) {
    // Инициализация матрицы поворота 3x3 с использованием данных из массива outputData->dataM (формат данных: float32_t)
    arm_matrix_instance_f32 rotM;
    arm_mat_init_f32(&rotM, 3, 3, sIncl->dataM);
    
    // Гарантия, что данные не изменятся во время расчетов.
    sIncl->dataIn[0] = arm_sin_f32(inputAngles[0]);
    sIncl->dataIn[1] = arm_sin_f32(inputAngles[1]);
    sIncl->dataIn[2] = arm_sin_f32(inputAngles[2]);
    
    // Выполняется умножение матрицы на вектор.
    arm_mat_vec_mult_f32(&rotM, sIncl->dataIn, sIncl->dataOut);
}
 /**
 * @brief   Инициализирует матрицу поворота.
 * @note    Данная функция должна быть вызвана один раз перед использованием.
 *          Рекомендуется вызывать функцию при установке устройства на ровную поверхность.
 *          Затем необходимо поднять устройство вдоль оси X на любой угол, предпочтительно не очень маленький (идеально на 45 градусов),
 *          и вызвать функцию InclDataInit2 один раз.
 *
 * @param[in]   inputAngles      Указатель на первый элемент массива, состоящего из трех углов в радианах.
 * @param[out]  outputData       Указатель на структуру данных sInclData датчика.
 */
void inclDataInit1(float32_t* inputAngles, sInclData* sIncl) {
    arm_matrix_instance_f32 rotMx;
    arm_matrix_instance_f32 rotMy;
    arm_matrix_instance_f32 rotM;
    float32_t dataMx[9];
    float32_t dataMy[9];
    float32_t tempData[3];
   
    arm_mat_init_f32(&rotMx, 3, 3, dataMx);
    arm_mat_init_f32(&rotMy, 3, 3, dataMy);
    arm_mat_init_f32(&rotM, 3, 3, sIncl->dataM);
   
    sIncl->dataIn[0] = arm_sin_f32(inputAngles[0]);
    sIncl->dataIn[1] = arm_sin_f32(inputAngles[1]);
    sIncl->dataIn[2] = arm_sin_f32(inputAngles[2]);
    // Угол вращения вокруг оси X равен углу Y инклинометра.
    sIncl->rotAngles[0] = inputAngles[1];
    dataMxInit(dataMx, sIncl->rotAngles[0]);
   
    arm_mat_vec_mult_f32(&rotMx, sIncl->dataIn, tempData);
   
    sIncl->rotAngles[1] = asinf(tempData[0]);
    dataMyInit(dataMy, sIncl->rotAngles[1]);
   
    arm_mat_mult_f32(&rotMy, &rotMx, &rotM); 
}

/**
 Тут находим и запоминаем угол вращения по Z когда подняли устройство
**/
 void inclDataInit2(float32_t* inputAngles, sInclData* sIncl) {
    arm_matrix_instance_f32 rotMz;
    arm_matrix_instance_f32 rotM;
    arm_matrix_instance_f32 rotMtemp;
    float32_t dataMz[9];
    float32_t dataMtemp[9];
    float32_t temp;

    arm_mat_init_f32(&rotMz, 3, 3, dataMz);
    arm_mat_init_f32(&rotM, 3, 3, sIncl->dataM);
    arm_mat_init_f32(&rotMtemp, 3, 3, dataMtemp);

    fixAngles(inputAngles, sIncl);

    // Первый вариант срабатывает при угле поворота по Z не больше, чем на 45 градусов, второй — когда больше 45 градусов
    if (fabsf(sIncl->dataOut[0]) > fabsf(sIncl->dataOut[1])) {
        sIncl->rotAngles[2] = atan2f(sIncl->dataOut[1], sIncl->dataOut[0]);
    } else {
        temp = sIncl->dataOut[0] / sIncl->dataOut[1];
        sIncl->rotAngles[2] = acosf(temp / sqrtf(1.0f + temp * temp));
    }

    dataMzInit(dataMz, sIncl->rotAngles[2]);
    arm_copy_f32(sIncl->dataM, dataMtemp, 9);
    arm_mat_mult_f32(&rotMz, &rotMtemp, &rotM);
}
 /**
  * @brief  Функции, инициализирующие массивы из 9 элементов для матриц вращения 3x3 на известные углы phi,theta,psi.
  * @note   Если вращает в неправильном направлении, можно транспонировать матрицу для изменения направления вращения.
  * @param[in]  input_angls     Указатель на первый элемент массива, состоящего из 9 элементов.
  * @param[out] output_angls    Углы phi,theta,psi
  */   
 static void dataMxInit(float32_t* pData, float32_t phi){
   pData[0] = 1.0f;
   pData[1] = 0.0f;
   pData[2] = 0.0f;
   pData[3] = 0.0f;
   pData[4] = arm_cos_f32(phi);
   pData[5] = -arm_sin_f32(phi);
   pData[6] = 0.0f;
   pData[7] = arm_sin_f32(phi);
   pData[8] = arm_cos_f32(phi);
 }
 static void dataMyInit(float32_t* pData, float32_t theta){
   pData[0] = arm_cos_f32(theta);
   pData[1] = 0.0f;
   pData[2] = -arm_sin_f32(theta);
   pData[3] = 0.0f;
   pData[4] = 1.0f;
   pData[5] = 0.0f;
   pData[6] = arm_sin_f32(theta);
   pData[7] = 0.0f;
   pData[8] = arm_cos_f32(theta);
 }
 static void dataMzInit(float32_t* pData, float32_t psi){   
   pData[0] = arm_cos_f32(psi);
   pData[1] = arm_sin_f32(psi);
   pData[2] = 0.0f;
   pData[3] = -arm_sin_f32(psi);
   pData[4] = arm_cos_f32(psi);
   pData[5] = 0.0f;
   pData[6] = 0.0f;
   pData[7] = 0.0f;
   pData[8] = 1.0f;
 }
/**
 *
 * End of file.
 */
