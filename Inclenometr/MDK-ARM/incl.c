#include "incl.h"


uint8_t Error_CRC =2;

uint16_t Incl_Data_SPI(uint32_t command, uint32_t delay_ms){
  
  uint8_t data_tx[4];
  uint8_t data_rx[4] = {0};
  uint16_t temp;

  uint32_t tempDataCRC;
  uint8_t INCLCRCSPI;
  
  data_tx[3] = (command & 0xFF);
  data_tx[2] = (command>>8) & 0xFF;
  data_tx[1] = (command>>16) & 0xFF;
  data_tx[0] = (command>>24) & 0xFF;
  
  if( (command == INCL_READ_ANG_X) || (command == INCL_READ_ANG_Y) || (command == INCL_READ_ANG_Z)  ){
    
     if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
      HAL_SPI_Transmit(&hspi1, data_tx, 4, 0x100);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
      HAL_Delay(delay_ms);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi1, data_tx, data_rx, 4, 0x100);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
      HAL_Delay(delay_ms);
      
      tempDataCRC = (data_rx[0] << 24) + (data_rx[1] << 16) + (data_rx[2] << 8) + data_rx[3];
      INCLCRCSPI = CalculateCRC(tempDataCRC);
      while (data_rx[3] != INCLCRCSPI){
        Error_CRC+=1;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&hspi1, data_tx, data_rx, 4, 0x100);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_Delay(delay_ms);
        tempDataCRC = (data_rx[0] << 24) + (data_rx[1] << 16) + (data_rx[2] << 8) + data_rx[3];
        INCLCRCSPI = CalculateCRC(tempDataCRC);
      }
      
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
  Incl_Data_SPI(INCL_CHANGE_TO_MODE_1, 1);
  //включить измерения
  Incl_Data_SPI(INCL_ENABLE_ANGL_OUTPUTS, 25);
  //прочитать статус
  Incl_Data_SPI(INCL_READ_STATUS, 1);
  Incl_Data_SPI(INCL_READ_STATUS, 1);
}

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






