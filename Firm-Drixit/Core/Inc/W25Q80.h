/*
 * W25Q80.h
 *
 *  Created on: 2 abr. 2024
 *      Author: mdelu
 */

#ifndef INC_W25Q80_H_
#define INC_W25Q80_H_

#include "stdint.h"
#include "stm32f1xx_hal.h"

/********************************
 * Variables
 ********************************/
SPI_HandleTypeDef hspi_flash;
GPIO_TypeDef flash_CH_PORT;
uint16_t flash_CH_PIN;

#define W25Q80_READ_SR1     0x05
#define W25Q80_PAGE_PROGRAM 0x02
#define W25Q80_FAST_READ    0x0B

#define W25Q80_WRITE_ENABLE  0x06
#define W25Q80_WRITE_DISABLE 0x04
#define W25Q80_ERASE         0xC7

/*
 *  Functions definitions
 */
void W25Q_Init(SPI_HandleTypeDef* spi, GPIO_TypeDef *cs_port, uint16_t cs_pin);

HAL_StatusTypeDef W25Q_Read_data (uint8_t *ReadData , uint32_t ADD , int16_t SIZE);
HAL_StatusTypeDef W25Q_Write_data(uint8_t *WriteData, uint32_t ADD , int16_t SIZE);

void W25Q80_Set_Write_State(uint8_t STATE);
void W25Q80_Is_Busy(void);
void W25Q80_Full_Erase(void);

#endif /* INC_W25Q80_H_ */
