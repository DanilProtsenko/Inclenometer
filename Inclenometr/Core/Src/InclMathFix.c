/******************************************************************************
 * @file     InclMathFix.c
 * @brief    Cодержит определения функций для исправления кривого расположения инелинометра SCL3300-D01
 * @version  V1.0
 * @date     08 Июнь 2023
 * Собирался на : STM32L476RGTx c инклинометром SCL3300-D01
 ******************************************************************************/
 
#include "InclMathFix.h"
 
 /**
  * @brief  Главная функция, перемножающая вектор из 3х элементов на матрицу поворота 3x3.
  * @note   Используйте после инициализации матрицы поворота (функции Incl_Data_Init_1 и Incl_Data_Init_2).
  * @param[in]  input_angls     Указатель на первый элемент массива, состоящего из 3х углов в РАДИАНАХ
  * @param[out] output_angls    Указатель на структуру данных sInclDat датчика
  */
 void fixangl(float32_t* input_angls, sInclData* sStruct){
   arm_matrix_instance_f32 rotM;
   arm_mat_init_f32(&rotM,3,3,sStruct->dataM);
   
   //гарантия что данные не изменятся во время расчетов. Функции, которые работают с матрицами принимают константные входные значения.
   sStruct->data_in[0] = arm_sin_f32(input_angls[0]);
   sStruct->data_in[1] = arm_sin_f32(input_angls[1]);
   sStruct->data_in[2] = arm_sin_f32(input_angls[2]);
   
   arm_mat_vec_mult_f32(&rotM,sStruct->data_in,sStruct->data_out);
 }
 /**
  * @brief  Функции инициализирующие матрицу поворта.
  * @note   Сначала необходимо вызвать функцию с цифрой 1 ОДИН раз(например по нажатию кнопки), когда устройство стоит на ровной поверхности.
   Даллее, необходимо поднять устройство вдоль оси X на любой угол, жедательно не слишком маленький (идеально на 45 градусов) и вызвать
   функцию с цифрой 2 ОДИН раз(например по нажатию кнопки).
  * @param[in]  input_angls     Указатель на первый элемент массива, состоящего из 3х углов в РАДИАНАХ
  * @param[out] output_angls    Указатель на структуру данных sInclDat датчика
  */  
 void Incl_Data_Init_1(float32_t* pData, sInclData* sStruct){
   
   arm_matrix_instance_f32 rotMx;
   arm_matrix_instance_f32 rotMy;
   arm_matrix_instance_f32 rotM;
   float32_t dataMx[9];
   float32_t dataMy[9];
   float32_t tempData[3];
   
   arm_mat_init_f32(&rotMx,3,3,dataMx);
   arm_mat_init_f32(&rotMy,3,3,dataMy);
   arm_mat_init_f32(&rotM,3,3,sStruct->dataM);
   
   sStruct->data_in[0] = arm_sin_f32(pData[0]);
   sStruct->data_in[1] = arm_sin_f32(pData[1]);
   sStruct->data_in[2] = arm_sin_f32(pData[2]);
   //угол вращения вокруг оси X равен углу Y инклинометра
   sStruct->rot_angls[0] = pData[1];
   dataMx_init(dataMx,sStruct->rot_angls[0]);
   
   arm_mat_vec_mult_f32(&rotMx,sStruct->data_in,tempData);
   
   sStruct->rot_angls[1] = asinf(tempData[0]);
   dataMy_init(dataMy,sStruct->rot_angls[1]);
   
   arm_mat_mult_f32(&rotMx,&rotMy,&rotM); 
 }
/**
 Тут находим и запоминаем угол вращения по Z когда подняли устройство
**/
 void Incl_Data_Init_2(float32_t* pData, sInclData* sStruct){
   
   arm_matrix_instance_f32 rotMz;
   arm_matrix_instance_f32 rotM;
   arm_matrix_instance_f32 rotMtemp;
   float32_t dataMz[9];
   float32_t dataMtemp[9];
   
   arm_mat_init_f32(&rotMz,3,3,dataMz);
   arm_mat_init_f32(&rotM,3,3,sStruct->dataM);
   arm_mat_init_f32(&rotMtemp,3,3,dataMtemp);
   
   fixangl(pData, sStruct);
   /**Первый вариант срабатывает при угле поворота по Z не больше чем на 45 градусов,второй, когда больше 45, но это уже фантастика
   **/
   if(fabsf(sStruct->data_out[0]) > fabsf(sStruct->data_out[1]))
    sStruct->rot_angls[2] = atan2f(sStruct->data_out[1],sStruct->data_out[0]);
   else{
    float32_t temp = sStruct->data_out[0]/sStruct->data_out[1];
    sStruct->rot_angls[2] = acosf(temp/sqrtf(1.0f*temp*temp));
   }
   
   dataMz_init(dataMz,sStruct->rot_angls[2]);
   arm_copy_f32(sStruct->dataM,dataMtemp,9);
   arm_mat_mult_f32(&rotMtemp,&rotMz,&rotM);
   
 }
 /**
  * @brief  Функции инициализирующие массивы из 9 элементов для матриц вращения 3x3 на известные углы phi,theta,psi.
  * @note   Если вращает куда то нетуда можете транспонировать матрицу, тогда будет вращать в другую сторону
  * @param[in]  input_angls     Указатель на первый элемент массива, состоящего из 9 элементов.
  массив в дальнейшем будет использоваться для инициализации матрицы 3x3
  * @param[out] output_angls    углы phi,theta,psi
  */   
 static void dataMx_init(float32_t* pData, float32_t phi){
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
 static void dataMy_init(float32_t* pData, float32_t theta){
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
 static void dataMz_init(float32_t* pData, float32_t psi){   
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
