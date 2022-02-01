
#ifndef __sx1272_LORA_H__
#define __sx1272_LORA_H__
/*
 * physical reset of the Lora chip
 */
void lora_reset(void);
/*
 * Sets the radio transceiver in idle mode.
 */
void lora_idle(void);
/*
 * Sets the lora chip in sleep mode
 */
void lora_sleep(void); 
/**
 * Sets the radio transceiver in receive mode.
 * Incoming packets will be received.
 */
void lora_receive(void);
/**
 * Configure power level in dBm for transmission
 * @param level 2-17 dbm, from least to most power
 */
void lora_set_tx_power(int level);
/**
 * Set carrier frequency.
 * @param frequency Frequency in Hz(India 865-867MHz)
 */
void lora_set_frequency(long frequency);
/**
 * Set spreading factor.
 * @param sf 6-12, Spreading factor to use.
 */
void lora_set_spreading_factor(int sf);
/**
 * Set bandwidth (bit rate)
 * @param sbw Bandwidth in Hz (up to 500000). In India it will work only with 125KHz
 */
void lora_set_bandwidth(long sbw);
/*
 * Set coding rate which decides in calculating the bit error rate
 */ 
void lora_set_coding_rate(int denominator);
/**
 * Enable appending/verifying packet CRC.
 */
void lora_enable_crc(void);

/**
 * Perform hardware initialization.
 */
int lora_init(void);
/**
 * Send a packet.
 * @param buf Data to be sent
 * @param size Size of data.
 */
void lora_send_packet(uint8_t *buf, int size);

int lora_receive_packet(uint8_t *buf, int size);
/**
 * Returns non-zero if there is data to read (packet received).
 */
int lora_received(void);
/**
 * Return last packet's RSSI.
 */
int lora_packet_rssi(void);
/**
 * Return last packet's SNR (signal to noise ratio).
 */
float lora_packet_snr(void);


#endif
