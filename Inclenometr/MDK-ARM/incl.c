#include "incl.h"

void Incl_Data_SPI(uint32_t command, uint32_t delay_ms) {
    uint8_t data_tx[4];
    uint8_t data_rx[4] = {0};

    parse_command(data_tx, 4, (uint8_t*)&command);

    if ((command == INCL_READ_ANG_X) || (command == INCL_READ_ANG_Y) || (command == INCL_READ_ANG_Z)) {
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET) {
            // Отправка пакета без принятия
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
            HAL_SPI_Transmit(&hspi1, data_tx, 4, 0x100);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
            HAL_Delay(delay_ms);

            // Получение предыдущего пакета/отправка следующего
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
            HAL_SPI_TransmitReceive(&hspi1, data_tx, data_rx, 4, 0x100);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
            HAL_Delay(delay_ms);

            // check_crc(data_rx, data_tx);

            // Обработка полученных данных (если необходимо)
        }
    } else {
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
            HAL_SPI_Transmit(&hspi1, data_tx, 4, 0xffff);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
            HAL_Delay(delay_ms);
        }
    }
}


void Incl_init(void) {
    // Выход из спящего режима
    Incl_Data_SPI(INCL_WAKE_UP, 1);
    // Сброс настроек по умолчанию
    Incl_Data_SPI(INCL_SW_RESET, 1);
    // Выбор режима
    Incl_Data_SPI(INCL_CHANGE_TO_MODE_4, 1);
    // Включить измерения
    Incl_Data_SPI(INCL_ENABLE_ANGL_OUTPUTS, 100);
    // Прочитать статус
    Incl_Data_SPI(INCL_READ_STATUS, 1);

    Incl_Data_SPI(INCL_READ_STATUS, 1);
}

void parse_command(uint8_t* buffer, uint32_t buffer_size, uint8_t* command) {
    for (int32_t i = buffer_size - 1; i >= 0; i--) {
        buffer[i] = *command;
        command++;
    }
}

void Incl_Data_ANGL(float32_t* angls_buff) {
    uint8_t data_tx_x[4];
    uint8_t data_tx_y[4];
    uint8_t data_tx_z[4];

    uint8_t data_rx_x[4];
    uint8_t data_rx_y[4];
    uint8_t data_rx_z[4];

    uint32_t tx_comm_x = INCL_READ_ANG_X;
    uint32_t tx_comm_y = INCL_READ_ANG_Y;
    uint32_t tx_comm_z = INCL_READ_ANG_Z;

    parse_command(data_tx_x, 4, (uint8_t*)&tx_comm_x);
    parse_command(data_tx_y, 4, (uint8_t*)&tx_comm_y);
    parse_command(data_tx_z, 4, (uint8_t*)&tx_comm_z);

    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, data_tx_x, 4, 0xffff);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_Delay(1);

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&hspi1, data_tx_y, data_rx_x, 4, 0xffff);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_Delay(1);

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&hspi1, data_tx_z, data_rx_y, 4, 0xffff);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_Delay(1);

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_SPI_Receive(&hspi1, data_rx_z, 4, 0xffff);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    angls_buff[0] = (((float32_t)filter_x((data_rx_x[1] << 8) + data_rx_x[2])) / 16384.0f) * 90.0f;
    angls_buff[1] = (((float32_t)filter_y((data_rx_y[1] << 8) + data_rx_y[2])) / 16384.0f) * 90.0f;
    angls_buff[2] = (((float32_t)filter_z((data_rx_z[1] << 8) + data_rx_z[2])) / 16384.0f) * 90.0f;
}

int32_t filter_x(int16_t x)
{
  static uint32_t n;
  static int32_t m[N];
  static int32_t y;
  static int32_t inverse_N = (1 << 16) / N;

  y += x - m[n];
  m[n] = x;
  n = (n + 1) % N;
  
  return (y * inverse_N) >> 16;
}

int32_t filter_y(int16_t x)
{
  static uint32_t n;
  static int32_t m[N];
  static int32_t y;
  static int32_t inverse_N = (1 << 16) / N;

  y += x - m[n];
  m[n] = x;
  n = (n + 1) % N;
  
  return (y * inverse_N) >> 16;
}

int32_t filter_z(int16_t x)
{
  static uint32_t n;
  static int32_t m[N];
  static int32_t y;
  static int32_t inverse_N = (1 << 16) / N;

  y += x - m[n];
  m[n] = x;
  n = (n + 1) % N;
  
  return (y * inverse_N) >> 16;
}
/**
 *
 * End of file.
 */
