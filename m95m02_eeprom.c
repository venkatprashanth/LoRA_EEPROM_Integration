/*
 * Header Files
 */

#include "m95m02_eeprom.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <unistd.h>
#include <sys/param.h>
#include <stdio.h>

/*
 * SPI Chip Select Pin for EEPROM module
 */
#define PIN_NUM_CS_EEPROM         34  

/*
 * Register definitions for M95M02-DR EEPROM Module.
 */
#define ADDR_MASK               0xff
#define REG_EWEN                0x06
#define REG_READ                0x03
#define REG_WRITE               0x02
#define ADD_EWEN                0x60
#define REG_WRDI                0x04
#define REG_RDSR                0x05
#define REG_WRSR                0x01
#define REG_READ_ID             0x83
#define REG_WRITE_ID            0x82
#define REG_READ_LOCK_STATUS    0x83
#define REG_LOCK_ID             0x82

/*
 * SPI_Handle
 */
spi_device_handle_t _spi;    

/*
 * Enable Read and Write
 */
static esp_err_t m95m02_writeRead_enable(uint16_t cmd)
{
    spi_transaction_t t = {
        .cmd = cmd,
    };
    gpio_set_level(PIN_NUM_CS_EEPROM, 0);
    return spi_device_polling_transmit(_spi, &t);
    gpio_set_level(PIN_NUM_CS_EEPROM, 1);
}

/*
 *Read the data from M95M02 EEPROM reg 
 */
esp_err_t EEPROM_ReadByte(uint8_t address, uint8_t* rxByte, uint8_t*rxLength)
{
    spi_transaction_t t = {
        .cmd = REG_READ | (address & ADDR_MASK),
        .rxlength = 8,
        .flags = 0,
    };
    gpio_set_level(PIN_NUM_CS_EEPROM, 0);
    esp_err_t err = spi_device_polling_transmit(_spi, &t);
    gpio_set_level(PIN_NUM_CS_EEPROM, 1);
    if (err!= ESP_OK) return err;

    *rxByte = t.rx_data[0];
    uint8_t len_out_data = (sizeof(rxByte)); 
    rxLength = &len_out_data;
    return err;
}

/*
 * Write the data to M95M02 EEPROM reg 
 */
esp_err_t EEPROM_WriteByte( uint8_t address, uint8_t *txByte)
{
    esp_err_t err;
    err = spi_device_acquire_bus(_spi, portMAX_DELAY);
    if (err != ESP_OK) return err;

    spi_transaction_t t = {
        .cmd = REG_WRITE | (address & ADDR_MASK),
        .length = 8,
        .flags = 0,
        .*tx_buffer = {&txByte},
    };
    gpio_set_level(PIN_NUM_CS_EEPROM, 0);
    err = spi_device_polling_transmit(_spi, &t);
    gpio_set_level(PIN_NUM_CS_EEPROM, 1);
    spi_device_release_bus(_spi);
    return err;
}

/*
 *  Function to call read or enable register of M95M02 EEPROM 
 */
esp_err_t spi_m95m02_writeRead_enable()
{
    return m95m02_writeRead_enable(REG_EWEN);
}



