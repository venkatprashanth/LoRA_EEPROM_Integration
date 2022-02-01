/*
 * Header Files
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include <string.h>
#include "sx1272_LORA.h"
/*
 * SPI Chip Select pin for SX1272 Lora soc Module
 */
#  define PIN_NUM_CS_LORA     13

/*
 * Register definitions for SX1272 Lora chip for Initial configuration
 */
#define REG_FIFO                       0x00
#define REG_OP_MODE                    0x01
#define REG_FRF_MSB                    0x06
#define REG_FRF_MID                    0x07
#define REG_FRF_LSB                    0x08
#define REG_PA_CONFIG                  0x09
#define REG_LNA                        0x0c
#define REG_FIFO_ADDR_PTR              0x0d
#define REG_FIFO_TX_BASE_ADDR          0x0e
#define REG_FIFO_RX_BASE_ADDR          0x0f
#define REG_FIFO_RX_CURRENT_ADDR       0x10
#define REG_IRQ_FLAGS                  0x12
#define REG_RX_NB_BYTES                0x13
#define REG_PKT_SNR_VALUE              0x19
#define REG_PKT_RSSI_VALUE             0x1a
#define PA_BOOST                       0x80
#define REG_MODEM_CONFIG_1             0x1d
#define REG_MODEM_CONFIG_2             0x1e
#define REG_PAYLOAD_LENGTH             0x22
#define REG_MODEM_CONFIG_3             0x26
#define REG_DETECTION_OPTIMIZE         0x31
#define REG_DETECTION_THRESHOLD        0x37
#define REG_VERSION                    0x42

/*
 * IRQ masks
 */
#define IRQ_TX_DONE_MASK               0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK     0x20
#define IRQ_RX_DONE_MASK               0x40

/*
 * Transceiver mode configuration
 */
#define MODE_LONG_RANGE_MODE           0x80
#define MODE_SLEEP                     0x00
#define MODE_STDBY                     0x01
#define MODE_TX                        0x03
#define MODE_RX_CONTINUOUS             0x05


/*
 * Timeout Reset 
 */
#define TIMEOUT_RESET                  100

/*
 * SPI Handle function
 */
static spi_device_handle_t __spi;

static int __implicit;
static long __frequency;

void sx1272_lora_write_reg(int reg, int data)
{
   uint8_t out[2] = { 0x80 | reg, data };
   uint8_t in[2];

   spi_transaction_t t = {
      .flags = 0,
      .length = 8 * sizeof(out),
      .tx_buffer = out,
      .rx_buffer = in  
   };

   gpio_set_level(PIN_NUM_CS_LORA, 0);
   spi_device_transmit(__spi, &t);
   gpio_set_level(PIN_NUM_CS_LORA, 1);
}


int sx1272_lora_read_reg(int reg)
{
   uint8_t out[2] = { reg, 0xff };
   uint8_t in[2];

   spi_transaction_t t = {
      .flags = 0,
      .length = 8 * sizeof(out),
      .tx_buffer = out,
      .rx_buffer = in
   };

   gpio_set_level(PIN_NUM_CS_LORA, 0);
   spi_device_transmit(__spi, &t);
   gpio_set_level(PIN_NUM_CS_LORA, 1);
   return in[1];
}


void sx1272_lora_reset(void)
{
   gpio_set_level(PIN_NUM_CS_LORA, 0);
   vTaskDelay(pdMS_TO_TICKS(1));
   gpio_set_level(PIN_NUM_CS_LORA, 1);
   vTaskDelay(pdMS_TO_TICKS(10));
}


void sx1272_lora_idle(void)
{
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}


void sx1272_lora_sleep(void)
{ 
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}



void sx1272_lora_receive(void)
{
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}



void sx1272_lora_set_tx_power(int level)
{
   if (level < 2) level = 2;
   else if (level > 17) level = 17;
   lora_write_reg(REG_PA_CONFIG, PA_BOOST | (level - 2));
}



void sx1272_lora_set_frequency(long frequency)
{
   __frequency = frequency;
   uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
   lora_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
   lora_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
   lora_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}



void sx1272_lora_set_spreading_factor(int sf)
{
   if (sf < 6) sf = 6;
   else if (sf > 12) sf = 12;

   if (sf == 6) 
   {
      lora_write_reg(REG_DETECTION_OPTIMIZE, 0xc5);
      lora_write_reg(REG_DETECTION_THRESHOLD, 0x0c);
   } else {
      lora_write_reg(REG_DETECTION_OPTIMIZE, 0xc3);
      lora_write_reg(REG_DETECTION_THRESHOLD, 0x0a);
   }

   lora_write_reg(REG_MODEM_CONFIG_2, (lora_read_reg(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}



void sx1272_lora_set_bandwidth(long sbw)
{
   int bw;

   if (sbw <= 7.8E3) bw = 0;
   else if (sbw <= 10.4E3) bw = 1;
   else if (sbw <= 15.6E3) bw = 2;
   else if (sbw <= 20.8E3) bw = 3;
   else if (sbw <= 31.25E3) bw = 4;
   else if (sbw <= 41.7E3) bw = 5;
   else if (sbw <= 62.5E3) bw = 6;
   else if (sbw <= 125E3) bw = 7;
   else if (sbw <= 250E3) bw = 8;
   else bw = 9;
   lora_write_reg(REG_MODEM_CONFIG_1, (lora_read_reg(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}


void sx1272_lora_set_coding_rate(int denominator)
{
   if (denominator < 5) denominator = 5;
   else if (denominator > 8) denominator = 8;

   int cr = denominator - 4;
   lora_write_reg(REG_MODEM_CONFIG_1, (lora_read_reg(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}



void sx1272_lora_enable_crc(void)
{
   lora_write_reg(REG_MODEM_CONFIG_2, lora_read_reg(REG_MODEM_CONFIG_2) | 0x04);
}


int sx1272_lora_init(void)
{

    sx1272_lora_reset();

    /*
     * Check version.
    */
    uint8_t version;
    uint8_t i = 0;
    while(i++ < TIMEOUT_RESET) 
    {
       version = sx1272_lora_read_reg(REG_VERSION);
       if(version == 0x12) break;
       vTaskDelay(2);
    }
    assert(i <= TIMEOUT_RESET + 1); // at the end of the loop above, the max value i can reach is TIMEOUT_RESET + 1

   /*
    * Default configuration.
    */
   sx1272_lora_sleep();
   sx1272_lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0);
   sx1272_lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0);
   sx1272_lora_write_reg(REG_LNA, lora_read_reg(REG_LNA) | 0x03);
   sx1272_lora_write_reg(REG_MODEM_CONFIG_3, 0x04);
   sx1272_lora_set_tx_power(17);
   sx1272_lora_idle();
   return 1;
}


void sx1272_lora_send_packet(uint8_t *buf, int size)
{
   /*
    * Transfer data to radio.
    */
   lora_idle();
   lora_write_reg(REG_FIFO_ADDR_PTR, 0);

   for(int i=0; i<size; i++) 
      lora_write_reg(REG_FIFO, *buf++);
   
   lora_write_reg(REG_PAYLOAD_LENGTH, size);
   
   /*
    * Start transmission and wait for conclusion.
    */
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
   while((lora_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0)
      vTaskDelay(2);

   lora_write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
}


int sx1272_lora_receive_packet(uint8_t *buf, int size)
{
   int len = 0;

   /*
    * Check interrupts.
    */
   int irq = lora_read_reg(REG_IRQ_FLAGS);
   lora_write_reg(REG_IRQ_FLAGS, irq);
   if((irq & IRQ_RX_DONE_MASK) == 0) return 0;
   if(irq & IRQ_PAYLOAD_CRC_ERROR_MASK) return 0;

   /*
    * Find packet size.
    */
   if (__implicit) len = lora_read_reg(REG_PAYLOAD_LENGTH);
   else len = lora_read_reg(REG_RX_NB_BYTES);

   /*
    * Transfer data from radio.
    */
   lora_idle();   
   lora_write_reg(REG_FIFO_ADDR_PTR, lora_read_reg(REG_FIFO_RX_CURRENT_ADDR));
   if(len > size) len = size;
   for(int i=0; i<len; i++) 
      *buf++ = lora_read_reg(REG_FIFO);

   return len;
}


int sx1272_lora_received(void)
{
   if(lora_read_reg(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK) return 1;
   return 0;
}


int sx1272_lora_packet_rssi(void)
{
   return (lora_read_reg(REG_PKT_RSSI_VALUE) - (__frequency < 868E6 ? 164 : 157));
}


float sx1272_lora_packet_snr(void)
{
   return ((int8_t)lora_read_reg(REG_PKT_SNR_VALUE)) * 0.25;
}

	
