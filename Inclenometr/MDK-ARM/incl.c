#include "incl.h"


//uint8_t Error_CRC = 0;

uint16_t Incl_Data_SPI(uint32_t command, uint32_t delay_ms){
  
  uint8_t data_tx[4];
  uint8_t data_rx[4] = {0};
  uint16_t temp;

//  uint32_t tempDataCRC;
  
//  data_tx[3] = (command & 0xFF);
//  data_tx[2] = (command>>8) & 0xFF;
//  data_tx[1] = (command>>16) & 0xFF;
//  data_tx[0] = (command>>24) & 0xFF;
  
  parse_command(data_tx,4,(uint8_t*)&command);
  
  if( (command == INCL_READ_ANG_X) || (command == INCL_READ_ANG_Y) || (command == INCL_READ_ANG_Z)  ){
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET){
      //отправка пакета без принятия
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
      HAL_SPI_Transmit(&hspi1, data_tx, 4, 0x100);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
      HAL_Delay(delay_ms);
      //получение предыдущего пакета/отправка следующего
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi1, data_tx, data_rx, 4, 0x100);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
      HAL_Delay(delay_ms);
      
//     check_crc(data_rx, data_tx);
      
      temp = (data_rx[1] << 8) + data_rx[2];
      return temp;
    }
     
  }else {
     if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET){
       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
       HAL_SPI_Transmit(&hspi1, data_tx, 4, 0xffff);
       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
       HAL_Delay(delay_ms);
     }
    }
}

void Incl_init(void){
  
  //выход из спящего режима
  Incl_Data_SPI(INCL_WAKE_UP, 1);
  //Сброс настроек по умолчанию
  Incl_Data_SPI(INCL_SW_RESET, 1);
  //выбор режима
  Incl_Data_SPI(INCL_CHANGE_TO_MODE_4, 1);
  //включить измерения
  Incl_Data_SPI(INCL_ENABLE_ANGL_OUTPUTS, 100);
  //прочитать статус
  Incl_Data_SPI(INCL_READ_STATUS, 1);
  
  Incl_Data_SPI(INCL_READ_STATUS, 1);
}
//uint8_t calculate_crc(uint32_t Data)
//{
//  uint8_t BitIndex;
//  uint8_t BitValue;
//  uint8_t CRCSPI;
//  CRCSPI = 0xFF;
//  for (BitIndex = 31; BitIndex > 7; BitIndex--)
//  {
//  BitValue = (uint8_t)((Data >> BitIndex) & 0x01);
//  CRCSPI = CRC8(BitValue, CRCSPI);
//  }
//  CRCSPI = (uint8_t)~CRCSPI;
//  return CRCSPI;
//}
//static uint8_t CRC8(uint8_t BitValue, uint8_t CRCSPI)
//{
//  uint8_t Temp;
//  Temp = (uint8_t)(CRCSPI& 0x80);
//  if (BitValue == 0x01)
//  {
//  Temp ^= 0x80;
//  }
//  CRCSPI <<= 1;
//  if (Temp > 0)
//  {
//  CRCSPI ^= 0x1D;
//  }
//  return CRCSPI;
//}

void parse_command(uint8_t* buffer, uint32_t buffer_size, uint8_t* command){
  for(int32_t i = buffer_size-1; i >= 0; i--){
  buffer[i] = *command;
  command++;
  }
}
void Incl_Data_ANGL(float32_t* angls_buff){
  
  uint8_t data_tx_x[4];
  uint8_t data_tx_y[4];
  uint8_t data_tx_z[4];
  
  uint8_t data_rx_x[4];
  uint8_t data_rx_y[4];
  uint8_t data_rx_z[4];
  
//  uint8_t data_tx_Gx[4];
//  uint8_t data_tx_Gy[4];
//  uint8_t data_tx_Gz[4];
//  
//  uint8_t data_rx_Gx[4];
//  uint8_t data_rx_Gy[4];
//  uint8_t data_rx_Gz[4];
  
  uint32_t tx_comm_x = INCL_READ_ANG_X;
  uint32_t tx_comm_y = INCL_READ_ANG_Y;
  uint32_t tx_comm_z = INCL_READ_ANG_Z;
  
//  uint32_t tx_comm_Gx = INCL_READ_ACC_X;
//  uint32_t tx_comm_Gy = INCL_READ_ACC_Y;
//  uint32_t tx_comm_Gz = INCL_READ_ACC_Z;
  
  parse_command(data_tx_x,4,(uint8_t*)&tx_comm_x);
  parse_command(data_tx_y,4,(uint8_t*)&tx_comm_y);
  parse_command(data_tx_z,4,(uint8_t*)&tx_comm_z);
  
//  parse_command(data_tx_Gx,4,(uint8_t*)&tx_comm_Gx);
//  parse_command(data_tx_Gy,4,(uint8_t*)&tx_comm_Gy);
//  parse_command(data_tx_Gz,4,(uint8_t*)&tx_comm_Gz);
  
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, data_tx_x, 4, 0xffff);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(1);
    
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, data_tx_y, data_rx_x, 4, 0xffff);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(1);
    
//    check_crc(data_rx_x, data_tx_x);
    
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, data_tx_z, data_rx_y, 4, 0xffff);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(1);
    
//    check_crc(data_rx_y, data_tx_y);
    
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Receive(&hspi1, data_rx_z, 4, 0xffff);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(1);
    
//    check_crc(data_rx_z, data_tx_z);

//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//    HAL_SPI_TransmitReceive(&hspi1,data_tx_Gy, data_rx_Gx, 4, 0xffff);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
//    HAL_Delay(1);
//    
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//    HAL_SPI_TransmitReceive(&hspi1,data_tx_Gz, data_rx_Gy, 4, 0xffff);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
//    HAL_Delay(1);
//    
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//    HAL_SPI_Receive(&hspi1,data_rx_Gz, 4, 0xffff);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
//    HAL_Delay(1);
    
  }
  
    angls_buff[0] = (((float32_t)filter_x((data_rx_x[1] << 8) + data_rx_x[2]))/ 16384.0f)* 90.0f;
    angls_buff[1] = (((float32_t)filter_y((data_rx_y[1] << 8) + data_rx_y[2]))/ 16384.0f)* 90.0f;
    angls_buff[2] = (((float32_t)filter_z((data_rx_z[1] << 8) + data_rx_z[2]))/ 16384.0f)* 90.0f;
  
//    angls_buff[3] = (float32_t)filter_Gx((data_rx_Gx[1] << 8) + data_rx_Gx[2])/ 12000;
//    angls_buff[4] = (float32_t)filter_Gy((data_rx_Gy[1] << 8) + data_rx_Gy[2])/ 12000;
//    angls_buff[5] = (float32_t)filter_Gz((data_rx_Gz[1] << 8) + data_rx_Gz[2])/ 12000;
  
}

//void check_crc(uint8_t* data_rx, uint8_t* data_tx){
//  uint32_t temp_crc;
//  uint8_t temp_data_rx[4];
//  
//  temp_data_rx[3] = data_rx[3];
//  
////  temp_crc = calculate_crc((data_rx[0] << 24) + (data_rx[1] << 16) + (data_rx[2] << 8) + data_rx[3]);
//  temp_crc =1;
//  while (temp_data_rx[3] != temp_crc){
//   Error_CRC+=1;
//   
//   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//   HAL_SPI_Transmit(&hspi1, data_tx, 4, 0xffff);
//   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
//   HAL_Delay(1);
//    
//   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//   HAL_SPI_Receive(&hspi1, temp_data_rx, 4, 0xffff);
//   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
//   HAL_Delay(1);
//    
//   temp_crc = calculate_crc((temp_data_rx[0] << 24) + (temp_data_rx[1] << 16) + (temp_data_rx[2] << 8) + temp_data_rx[3]);

//   }
//}

int16_t filter_x(int16_t x)
{
  static uint32_t n;
  static int32_t m[N];
  static int32_t y;
  y +=x-m[n];
  m[n]=x;
  n=(n+1)%N;
  one_sec_flag++;
  return y/N;
}

int16_t filter_z(int16_t x)
{
  static uint32_t n;
  static int32_t m[N];
  static int32_t y;
  y +=x-m[n];
  m[n]=x;
  n=(n+1)%N;
  
  return y/N;
}

int16_t filter_y(int16_t x)
{
  static uint32_t n;
  static int32_t m[N];
  static int32_t y;
  y +=x-m[n];
  m[n]=x;
  n=(n+1)%N;
  
  return y/N;
}

//int16_t filter_Gx(int16_t x)
//{
//  static uint32_t n;
//  static int32_t m[N];
//  static int32_t y;
//  y +=x-m[n];
//  m[n]=x;
//  n=(n+1)%N;
//  
//  return y/N;
//}

//int16_t filter_Gz(int16_t x)
//{
//  static uint32_t n;
//  static int32_t m[N];
//  static int32_t y;
//  y +=x-m[n];
//  m[n]=x;
//  n=(n+1)%N;
//  
//  return y/N;
//}

//int16_t filter_Gy(int16_t x)
//{
//  static uint32_t n;
//  static int32_t m[N];
//  static int32_t y;
//  y +=x-m[n];
//  m[n]=x;
//  n=(n+1)%N;
//  
//  return y/N;
//}


