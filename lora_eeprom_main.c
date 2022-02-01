/*
 * Header files
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "m95m02_eeprom.h"
#include "sx1272_lora.h"

/*
 * SPI Interface Pins Declaration
 */
#define EEPROM_HOST    SPI1_HOST
#define DMA_CHAN            0
#define PIN_NUM_MISO        7
#define PIN_NUM_MOSI        8
#define PIN_NUM_CLK         6

/*
 * Defining the tag
 */
static const char TAG[] = "main_code";

/*
 * SPI Handle function
 */
static spi_device_handle_t __spi;

/*
 * Defining the struct for SPI Type and read and write 
 */
struct {
    uint8_t SPI_TYPE;
    uint8_t read_write_type;
}config_spi_type;

/*
 * Initializing the struct members
 */
config_spi_type config_spi_t = {
        .SPI_TYPE = 0,
        .read_write_type = 0,
    };
        
/*
 * SPI interface bus Initialisation and adding the device
 */
void spi_init(void){
    esp_err_t ret;
    spi_bus_config_t bus = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };
           
    ret = spi_bus_initialize(VSPI_HOST, &bus, 0);
    assert(ret == ESP_OK);

    spi_device_interface_config_t dev = {
        .clock_speed_hz = 9000000,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
        .flags = 0,
        .pre_cb = NULL
    };
    ret = spi_bus_add_device(VSPI_HOST, &dev, &__spi);
    assert(ret == ESP_OK);
}
/*
 * SX1272 Physical Layer Initialisation
 */
void lora_init_task(void){

    uint8_t lora_retu = lora_init();
    sx1272_lora_set_frequency(865e6);         //Set the frequency in range IN(865-867)MHz
    sx1272_lora_enable_crc();                 //CRC Check
    /*
     * Set the Spread factor to 12 to achieve the max range
     * High the Spread factor high will be latency and high data can be transmitted.
     * Each symbol can hold 2^SF chips.where sf is spread factor
     */
    sx1272_lora_set_spreading_factor(12); 
    /*
     * In lndia LORA bandwidth can only configured for 125kHz
     * BW = 125Khz = 125000 chips/second
     * For example: SF = 9 indicates that Symbol carries 9 raw bits of information and symbol holds 2SF = 29 = 512 chips.
     * Symbol rate Rs (symbols/sec) = BW / 2^SF
     * So, increasing the spread factor will increase the latency and higher data can be transmiited and high range of tranmssion. 
     */    
    sx1272_lora_set_bandwidth(125E3);
    /*
     * The Coding rate refers to the proportion of the transmitted bits that actually carries the Information.
     * where CR = 5,6,7,8. High the CR, high the rate
     */        
    sx1272_lora_set_coding_rate(6);
}

/*
 * Brief send payload to gateway using LORA protocol
 * @param payloadBytes payload in bytes
 * @param return 0 for success and 1 for failure
 */
uint8_t task_lora_tx(uint8_t *payloadBytes)
{
    vTaskDelay(pdMS_TO_TICKS(5000));
    sx1272_lora_send_packet(&payloadBytes,sizeof(payloadBytes));
    printf("packet sent successfully\n");
    _delay_ms(300);            //Block the function for 3 seconds once after sending the data
    sx1272_lora_receive();
    while(sx1272_lora_receive())
    {
        unsigned char *buf_receive_ack;
        received_payload = sx1272_lora_receive_packet(&buf_receive_ack,sizeof(buf_receive_ack))
        if (*buf_receive_ack == "ACK")
        {
            return 0;
        }
        else
        {
            return 1
        }
    }
}

/*
 * Brief receive payload from gateway using LORA protocol
 * @param buf received payload in bytes
 * @param return 0 for success and 1 for failure
 */
uint8_t task_lora_rx(void)
{
    unsigned char buf_receive_payload[];
    uint8_t len;
    uint8_t i = 0;
    sx1272_lora_receive();    // put into receive mode
    while(sx1272_lora_received())
    {
        received_payload_len = sx1272_lora_receive_packet(&buf_receive_payload[i], sizeof(buf_receive_payload));
        i++;
        sx1272_lora_receive();
    }
    printf("Received_data_is: %d\n",*buf_receive_payload);
    sx1272_lora_sleep();
    vTaskDelay(1);
    if(received_payload_len > 0)
    {
        return 0;
    }
    else
    {
        return 1;
    
    }
}

/*
 * Brief function sends and reads the data from EEPROM M95M02 module and SX1272 Lora module through spi communication
 * @param txByte data that will be transmitted through lora and EEPROM Module 
 * @param config_spt_t lora,eeprom,read and write selection
 * @param ret returns o for success and 1 for failure
 */
uint8_t SPI_ReadAndWriteByte(uint8_t *txByte,const config_spi_type *config_spi_t)
{

    if(config_spi_t->SPI_TYPE == 0)  //zero defines LORA TYPE communication.
    { 
        lora_init_task();
        if (config_spi_t->read_write_type== 0) //zero defines to write or send the data using spi through lora module.
        {
            uint8_t ret = task_lora_tx(&txbyte);
            if (ret == 0)
            {
                printf("Sent the data successfully to gateway:%d\n",ret);
            }
            if (ret == 1)
            {
                printf("Failed to send the data to gateway:%d\n",ret);
            }
        }
        else if(config_spi_t->read_write_type == 1) //One defines to read the data from lora module.
        {
           uint8_t ret == task_lora_rx();
           if(ret == 0)
           {
                printf("Received the data from gateway:%d\n",ret);
           }
            if(ret == 1)
            {
                printf("Failed to receive the data:%d\n",ret);
            }
        }
    else if(config_spi_t->SPI_TYPE == 1) //One defines the EEPROM spi communication
    { 
        ret = spi_m95m02_writeRead_enable(eeprom_handle); // Enable the EEPROM to read to write
        ESP_ERROR_CHECK(ret);
        if (config_spi_t->read_write_type == 0)  //Zero defined to write the data to EEPROM Module
        {
            for(int i = 0; i <=20; i++) 
            {
                ret = EEPROM_WriteByte(i, &txByte);
                ESP_ERROR_CHECK(ret);
            }
        }
        else if(config_spi_t->read_write_type == 1)  //One defined to read the data from EEPROM Module
        {
            uint8_t *rxLength;
            for (int i = 0; i < sizeof(test_str); i++) 
            {
                ret = EEPROM_ReadByte(i, &txbyte[i],&rxLength);
                ESP_ERROR_CHECK(ret);
             }
        }
    }
    }
    return ret;
}

/*
 * Brief calling the functions to read and write from eeprom and Lora module
 */
void task_lora_eeprom(*p)
{
    for(::)
    {   
        /*
         * Declaration of variables
         */
        uint8_t payloadBytes[20] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};  //20 bytes payload for sending on lora and eeprom
        uint8_t EEPROMReceive[] ="";                                //For receving the data through eeprom module
        uint8_t PayloadReceive[] ="";                               //For receiving the data though lora module
        uint8_t *payload = payloadBytes;
        esp_err_t ret;  
        
        spi_init();
        config_spi_t.SPI_TYPE = 0;                                  //Defining the spi type as lora by passing zero
        config_spi_t.read_write_type = 0;                           //Defining the lora module as write by passing zero
        ret = SPI_ReadAndWriteByte(&payload,&config_spi_t);         //SPI type lora and write to reg
        _delay_ms(1000);                                            // sleep for 1 sec
        config_spi_t.read_write_type = 1;                           //Defining the lora module as read by passing one
        ret = SPI_ReadAndWriteByte(&PayloadReceive,&config_spi_t);  // SPI type lora and read from reg
        _delay_ms(1000);                                            //sleep for 1 sec


        config_spi_t.SPI_TYPE = 1;                                  //Defining the SPI type as EEPROM by passing 1
        config_spi_t.read_write_type = 0;                           //Defining the EEPROM module as write by passing zero
        ret = SPI_ReadAndWriteByte(&payload,&config_spi_t);         //SPI type eeprom and write to reg
        _delay_ms(1000);                                            //sleep for 1 sec
        config_spi_t.read_write_type = 1;                           //Defining the EEPROM module as read by passing one
        ret = SPI_ReadAndWriteByte(&EEPROMReceive,&config_spi_t);   //SPI Type eeprom and read from reg
        _delay_ms(1000);                                            //sleep for 1 sec
    }
}

/*
 * Brief configuring the task for main function
 */
void app_main(void)
{
    ESP_LOGI(TAG, "Main code has started");
    xTaskCreate(&task_lora_eeprom, "task_tx", 2048, NULL, 5, NULL);
}
