#ifndef __INCL_H
#define __INCL_H

#include "main.h"
#include "stdint.h"
#include "arm_math.h"

#define INCL_READ_ACC_X           0x040000F7
#define INCL_READ_ACC_Y           0x080000FD
#define INCL_READ_ACC_Z           0x0C0000FB
#define INCL_READ_STO             0x100000E9
#define INCL_ENABLE_ANGL_OUTPUTS  0xB0001F6F
#define INCL_READ_ANG_X           0x240000C7
#define INCL_READ_ANG_Y           0x280000CD
#define INCL_READ_ANG_Z           0x2C0000CB
#define INCL_READ_TEMP            0x140000EF
#define INCL_READ_STATUS          0x180000E5
#define INCL_READ_ERR_FLAG1       0x1C0000E3
#define INCL_READ_ERR_FLAG2       0x200000C1
#define INCL_READ_CMD             0x340000DF
#define INCL_CHANGE_TO_MODE_1     0xB400001F
#define INCL_CHANGE_TO_MODE_2     0xB4000102
#define INCL_CHANGE_TO_MODE_3     0xB4000225
#define INCL_CHANGE_TO_MODE_4     0xB4000338
#define INCL_SET_POWER_DOWN_MODE  0xB400046B
#define INCL_WAKE_UP              0xB400001F
#define INCL_SW_RESET             0xB4002098
#define INCL_WHOAMI               0x40000091
#define INCL_READ_SERIAL_1        0x640000A7
#define INCL_READ_SERIAL_2        0x680000AD
#define INCL_READ_CURRENT_BANK    0x7C0000B3
#define INCL_SWITCH_TO_BANK_0     0xFC000073
#define INCL_SWITCH_TO_BANK_1     0xFC00016E

#define N 32


typedef struct Incl_Data{
 
  float32_t angl_x;
  float32_t angl_y;
  float32_t angl_z;
  float32_t temp;

}sIncl;

extern SPI_HandleTypeDef hspi1;

extern uint8_t Error_CRC;
extern uint32_t one_sec_flag;

uint16_t Incl_Data_SPI(uint32_t command, uint32_t delay_ms);
void Incl_init(void);
uint8_t calculate_crc(uint32_t Data);
static uint8_t CRC8(uint8_t BitValue, uint8_t CRCSPI);

void check_crc(uint8_t* data_rx, uint8_t* data_tx);

void Incl_Data_ANGL(float32_t* angls_buff);

void parse_command(uint8_t* buffer, uint32_t buffer_size, uint8_t* command);

int16_t filter_x(int16_t x);
int16_t filter_z(int16_t x);
int16_t filter_y(int16_t x);

//int16_t filter_Gx(int16_t x);
//int16_t filter_Gy(int16_t x);
//int16_t filter_Gz(int16_t x);

#endif /* INCL_H_ */
