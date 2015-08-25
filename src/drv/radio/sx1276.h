/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: SX1276 LoRa modem driver.

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#ifndef __SX1276_H_
#define __SX1276_H_

#include "hal-sx1276.h"
#include "sx1276-lora-reg.h"

#define SX1276_ERROR_CRC                            (-1)
#define SX1276_ERROR_HEADER_INFO                    (-2)
#define SX1276_ERROR_HEADER_IRQ                     (-3)

/** Fstep = FXOSC/(2^19) */
#define SX1276_FREQ_STEP	    ((double)SX1276_FXOSC/((double)((uint32_t)1<<19)))

typedef enum{
	FSK,
	LORA,
}sx1276_modem_t;

typedef enum{
	SX1276_SF6 = 6,
	SX1276_SF7,
	SX1276_SF8,
	SX1276_SF9,
	SX1276_SF10,
	SX1276_SF11,
	SX1276_SF12,
}sx1276_sf_t;

typedef enum{
	SX1276_CR1 = 1,		// CR 4/5
	SX1276_CR2,			// CR 4/6
	SX1276_CR3,			// CR 4/7
	SX1276_CR4,			// CR 4/8
}sx1276_coding_rate_t;

typedef enum{
	SX1276_BW_7K8 = 0,
	SX1276_BW_10K4,
	SX1276_BW_15K6,
	SX1276_BW_20K8,
	SX1276_BW_31K25,
	SX1276_BW_41K7,
	SX1276_BW_62K5,
	SX1276_BW_125K,
	SX1276_BW_250K,
	SX1276_BW_500K,
}sx1276_bw_t;

typedef enum{
	SX1276_HEADER_ENABLE,
	SX1276_HEADER_DISABLE,
}sx1276_header_mode_t;

typedef enum{
	SX1276_CRC_ON,
	SX1276_CRC_OFF,
}sx1276_crc_mode_t;

typedef enum{
	SX1276_TX_PREAMBLE,
	SX1276_RX_PREAMBLE,
}sx1276_preamble_t;

typedef struct{
	uint32_t frequency;

	sx1276_sf_t spread_factor;
	sx1276_bw_t bandwidth;
	sx1276_coding_rate_t coding_rate;
	sx1276_crc_mode_t crc_mode;
	sx1276_header_mode_t header_mode;
	uint8_t payload_len;		// Set value when header is disabled

	int8_t tx_power;
	uint16_t tx_preamble_len;
	uint16_t rx_preamble_len;
}sx1276_config_t;

typedef enum{
    SX1276_IDLE,
	SX1276_TX_DONE,
	SX1276_RX_DONE,
    SX1276_CAD_DONE,
	SX1276_CAD_DETECTED,
//	SX1276_TX_TIMEOUT,
	SX1276_RX_TIMEOUT,
//	SX1276_CAD_TIMEOUT,
	SX1276_RX_ERROR,
	SX1276_CAD_ERROR,
}sx1276_event_t;

typedef enum{
	SX1276_IQ_RX_IVT,
    SX1276_IQ_RX_NML,
    SX1276_IQ_TX_IVT,
    SX1276_IQ_TX_NML,
    SX1276_IQ_RXTX_IVT,
    SX1276_IQ_RXTX_NML,
    SX1276_IQ_TXRX_IVT,     // same as SX1276_IQ_RXTX_IVT
    SX1276_IQ_TXRX_NML,     // same as SX1276_IQ_RXTX_NML
}sx1276_iq_t;

typedef enum{
	SX1276_CAD_THRESH_PNR,
    SX1276_CAD_THRESH_PEAK_MIN,
}sx1276_cad_thresh_t;

typedef struct{
	uint8_t *buf;
	uint8_t len;
	int16_t rssi;
    int16_t cad_rssi;
	int8_t snr;
    uint8_t cr;
    int8_t error;
}sx1276_rx_pkt_t;

typedef void (*sx1276_callback_t)(sx1276_event_t, void *);

/** SX1276 low-level functions */
uint8_t sx1276_read(uint8_t addr);
void sx1276_write(uint8_t addr, uint8_t data);
void sx1276_rmw(uint8_t addr, uint8_t mask, uint8_t val);
void sx1276_read_burst(uint8_t addr, uint8_t *buf, uint16_t len);
void sx1276_write_burst(uint8_t addr, uint8_t *buf, uint16_t len);
void sx1276_read_fifo(uint8_t *buf, uint16_t len);
void sx1276_write_fifo(uint8_t *buf, uint16_t len);

/** SX1276 modem mode control functions*/
void sx1276_init(sx1276_modem_t modem, sx1276_callback_t sx1276_cb);
void sx1276_set_modem( sx1276_modem_t modem );
void sx1276_reset(void);
void sx1276_set_mode(uint8_t mode);

/** SX1276 RF settings control functions */
void sx1276_set_config(sx1276_config_t *config);
void sx1276_set_channel( uint32_t freq );
void sx1276_set_sf(sx1276_sf_t sf);
void sx1276_set_bandwidth(sx1276_bw_t bw);
void sx1276_set_coding_rate(sx1276_coding_rate_t cr);
void sx1276_set_crc_mode(sx1276_crc_mode_t crc_mode);
void sx1276_set_header_mode(sx1276_header_mode_t hdr_mode);
void sx1276_set_preamble(sx1276_preamble_t preamble_type, uint16_t len);
void sx1276_set_preamble_reg(uint16_t len);
void sx1276_set_symbol_timeout(uint16_t val);
void sx1276_set_tx_power(int8_t pow);
void sx1276_set_iq(sx1276_iq_t iq);
void sx1276_set_cad_thresh(sx1276_cad_thresh_t cad_thresh, uint8_t val);

/** SX1276 RF settings get functions */
uint8_t sx1276_get_mode(void);

/** SX1276 TX/RX/CAD functions */
void sx1276_send(uint8_t *buf, uint8_t len, uint16_t timeout);
void sx1276_receive(uint32_t timeout);
void sx1276_cad(void);

/** SX1276 read rssi */
int16_t sx1276_read_rssi(void);

/** Convert timeout(ms) to LoRa symbols */
uint16_t sx1276_timeout_to_symbol(uint32_t timeout, sx1276_sf_t sf, sx1276_bw_t bw);



#endif
