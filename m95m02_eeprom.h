#ifndef __M95M02_EEPROM__H_
#define __M95M02_EEPROM__H_

/*
 * Header files
 */ 

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "sdkconfig.h"


/**
 * @Read a byte from M95M02-DR EEPROM Module.
 *
 * @param address      Address to read.
 * @param rxByte  Buffer to output the read data.
 * @param rxLength Buffer to output the rxByte Length.
 * @return return value failure or success.
 */
esp_err_t EEPROM_ReadByte(uint8_t address, uint8_t* rxByte, uint8_t*rxLength);

/**
 * @Write a byte from M95M02-DR EEPROM Module.
 *
 * @param address  Address to write.
 * @param txByte  The byte to write.
 * @return
 *  - ESP_OK: on success
 */
esp_err_t EEPROM_WriteByte( uint8_t address, uint8_t *txByte);

/**
 * @brief Enable following write/erase to M95M02-DR EEPROM Module.
 *
 * @param handle Context of EEPROM communication.
 * @return return value from `spi_device_polling_transmit()`.
 */
esp_err_t spi_m95m02_writeRead_enable();

#endif
