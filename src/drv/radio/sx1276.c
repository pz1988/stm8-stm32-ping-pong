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

#include "sx1276.h"
#include "hal-sx1276.h"
#include "hal-spi.h"
#include "sx1276-lora-reg.h"

/** Uncomment to force enable LOW DATA RATE OPTIMIZATION
    Comment means auto adaptive*/
//#define SX1276_LOWDATARATE_OPTIMIZATION_ON

/** Uncomment to force enable LOW DATA RATE OPTIMIZATION
    Comment means auto adaptive*/
#define SX1276_LOWDATARATE_OPTIMIZATION_OFF

/** Uncomment to force disable AGC/DAGC */
//#define SX1276_DISABLE_AGC

#define SX1276_BUFFER_SIZE				(256)

/** Read multiple times after enter CAD mode to get real data,
    Note: 5 is tested with SPI Speed 10MHz */
#define SX1276_CAD_READ_RSSI_TIMES      (5)

typedef struct{
    uint32_t frf;
    sx1276_bw_t bw;
    sx1276_sf_t sf;
    uint32_t rx_timeout;
}sx1276_rf_t;

typedef struct{
    int32_t *frf;
    uint8_t len;

}sx1276_fhss_t;

/** SX1276 low-level interface */
uint8_t sx1276_read(uint8_t addr);
void sx1276_write(uint8_t addr, uint8_t data);
void sx1276_rmw(uint8_t addr, uint8_t mask, uint8_t val);
void sx1276_read_burst(uint8_t addr, uint8_t *buf, uint16_t len);
void sx1276_write_burst(uint8_t addr, uint8_t *buf, uint16_t len);
void sx1276_read_fifo(uint8_t *buf, uint16_t len);
void sx1276_write_fifo(uint8_t *buf, uint16_t len);

#ifndef STM32_SENSERNODE_DELAY
/** DIOx callback definition */
void sx1276_irq_dio0(void);
void sx1276_irq_dio1(void);
void sx1276_irq_dio2(void);
void sx1276_irq_dio3(void);
void sx1276_irq_dio4(void);
void sx1276_irq_dio5(void);
#endif

/** Enable/Disable LowDatarateOptimization depends on SFx and BW */
static void sx1276_set_low_datarate_optimization(sx1276_sf_t sf, sx1276_bw_t bw);

/** Save preamble length for TX and RX */
static uint16_t sx1276_tx_preamble_len_g;
static uint16_t sx1276_rx_preamble_len_g;

/** Save important RF configuration */
static sx1276_rf_t sx1276_rf_g;

/** application callback funciton */
static sx1276_callback_t sx1276_callback_g;

/** Save received packet, copy from FIFO */
static uint8_t sx1276_rx_buf[SX1276_BUFFER_SIZE];

/** Packet information. As a parameter transmit to callback function.
    Applicaiton developer need to decide use this variable directly or make a copy,
    If there is enough memory, it is strongly recommended to make a copy. */
static sx1276_rx_pkt_t sx1276_rx_pkt_g;

//unit: us
static const uint32_t sx1276_symbol_length_tab[10][7]={
//  SF6   SF7    SF8    SF9    SF10    SF11    SF12
    8205, 16410, 32821, 65641, 131282, 262564, 525128,  // SF6~SF12 BW7K8
    6154, 12308, 24615, 49231, 98462,  196923, 393846,  // SF6~SF12 BW10K4
    4103, 8205,  16410, 32821, 65641,  131282, 262564,  // SF6~SF12 BW15K6
    3077, 6154,  12308, 24615, 49231,  98462,  196923,  // SF6~SF12 BW20K8
    2051, 4103,  8205,  16410, 32821,  65641,  131282,  // SF6~SF12 BW31K2
    1535, 3070,  6139,  12278, 24556,  49113,  98225,   // SF6~SF12 BW41K7
    1024, 2048,  4096,  8192,  16384,  32768,  65536,   // SF6~SF12 BW62K5
    512,  1024,  2048,  4096,  8192,   16384,  32768,   // SF6~SF12 BW125K
    256,  512,   1024,  2048,  4096,   8192,   16384,   // SF6~SF12 BW250K
    128,  256,   512,   1024,  2048,   4096,   8192,    // SF6~SF12 BW500K
};

void sx1276_reset(void)
{
	HAL_SX1276_RST_OUTPUT();
	HAL_SX1276_RST_L();
	delay_ms(1);

	HAL_SX1276_RST_INPUT();
	delay_ms(6);
}

void sx1276_init(sx1276_modem_t modem, sx1276_callback_t sx1276_cb)
{
    /** Initial related IO, interrupt mode */
	hal_sx1276_init();

	/** sfotware reset sx1276 */
	sx1276_reset();

    #ifndef STM32_SENSERNODE_DELAY
	/** Initial and enable all DIO interrupt */
	hal_sx1276_set_irq(SX1276_IRQ_DIO0, true, sx1276_irq_dio0);
	hal_sx1276_set_irq(SX1276_IRQ_DIO1, true, sx1276_irq_dio1);
	hal_sx1276_set_irq(SX1276_IRQ_DIO2, true, sx1276_irq_dio2);
	//hal_sx1276_set_irq(SX1276_IRQ_DIO3, true, sx1276_irq_dio3);
	//hal_sx1276_set_irq(SX1276_IRQ_DIO4, true, sx1276_irq_dio4);
	//hal_sx1276_set_irq(SX1276_IRQ_DIO5, true, sx1276_irq_dio5);
	#endif

	/** LoRa or FSK Modem */
	sx1276_set_modem(modem);

	/** Callback function */
	sx1276_callback_g = NULL;
	if(sx1276_cb != NULL){
		sx1276_callback_g = sx1276_cb;
	}

	sx1276_rx_pkt_g.buf = sx1276_rx_buf;
	sx1276_rx_pkt_g.len = 0;

    sx1276_rf_g.frf = 434000000;
    sx1276_rf_g.sf = SX1276_SF7;
    sx1276_rf_g.bw = SX1276_BW_125K;
    sx1276_rf_g.rx_timeout = 0;
}

void sx1276_set_config(sx1276_config_t *config)
{
	sx1276_set_mode(RFLR_OPMODE_STANDBY);

	sx1276_set_channel(config->frequency);
	sx1276_set_sf(config->spread_factor);
	sx1276_set_bandwidth(config->bandwidth);

	sx1276_set_coding_rate(config->coding_rate);
	sx1276_set_header_mode(config->header_mode);
	if(config->header_mode == SX1276_HEADER_DISABLE){
		sx1276_write(REG_LR_PAYLOADLENGTH, config->payload_len);
	}
	sx1276_set_crc_mode(config->crc_mode);

	sx1276_set_tx_power(config->tx_power);

	sx1276_set_preamble(SX1276_TX_PREAMBLE, config->tx_preamble_len);
	sx1276_set_preamble(SX1276_RX_PREAMBLE, config->rx_preamble_len);

	/** LowDataRataOptimization */
	sx1276_set_low_datarate_optimization(config->spread_factor, config->bandwidth);

	/** Check parameter valid */
	if(config->spread_factor == SX1276_SF6){
		if(config->header_mode == SX1276_HEADER_ENABLE){
			/** The header must be set to Implicit(Disabled) mode */
			while(1);
		}
	}

    /** Enable LNA boost for HF band */
    sx1276_write(REG_LR_LNA, 0x23);

#ifdef SX1276_DISABLE_AGC
    sx1276_write( 0x26, 0x00 );     // disable AGC for LoRa
    sx1276_write( 0x59, 0x08 );     // disable DAGC for LoRa
#endif
}

void sx1276_send(uint8_t *buf, uint8_t len, uint16_t timeout)
{
    /** stop on-going operation if any */
    sx1276_set_mode(RFLR_OPMODE_STANDBY);

	/** set preamble to tx preamble length */
	sx1276_set_preamble_reg(sx1276_tx_preamble_len_g);

	/** fill data to fifo */
	// Initializes the payload size
    sx1276_write( REG_LR_PAYLOADLENGTH, len );

	// Full buffer used for Tx
	sx1276_write( REG_LR_FIFOTXBASEADDR, 0 );
	sx1276_write( REG_LR_FIFOADDRPTR, 0 );

	// FIFO operations cannot take place in Sleep mode
	sx1276_set_mode(RFLR_OPMODE_STANDBY);

	// Write payload buffer
	sx1276_write_fifo(buf, len);

	/** set interrupt mode */
	sx1276_write(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
									  RFLR_IRQFLAGS_RXDONE |
									  RFLR_IRQFLAGS_PAYLOADCRCERROR |
									  RFLR_IRQFLAGS_VALIDHEADER |
									  //RFLR_IRQFLAGS_TXDONE |
									  RFLR_IRQFLAGS_CADDONE |
									  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
									  RFLR_IRQFLAGS_CADDETECTED );
	/** DIO0 TX Done */
	sx1276_rmw( REG_LR_DIOMAPPING1, RFLR_DIOMAPPING1_DIO0_MASK, RFLR_DIOMAPPING1_DIO0_01);

	/** start transmitor */
	sx1276_set_mode(RFLR_OPMODE_TX);
}

/**    0   --> RX Continuous
    others --> Timeout */
void sx1276_receive(uint32_t timeout)
{
    /** disable IRQ, and get current IRQ status */
    irq_state_t irq_sta = irq_disable();

    sx1276_set_mode(RFLR_OPMODE_STANDBY);

    /** Make sure STANDBY mode is stable */
    delay_ms(1);
  
	/** set preamble register to rx preamble length */
	sx1276_set_preamble_reg(sx1276_rx_preamble_len_g);

	// Full buffer used for Rx
	sx1276_write( REG_LR_FIFORXBASEADDR, 0 );
	sx1276_write( REG_LR_FIFOADDRPTR, 0 );

	sx1276_write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR|RFLR_IRQFLAGS_RXDONE|RFLR_IRQFLAGS_RXTIMEOUT );

	/** Enable RX_DONE, RX_TIMEOUT, RFLR_IRQFLAGS_PAYLOADCRCERROR, RFLR_IRQFLAGS_VALIDHEADER interrupt mode */
	sx1276_write( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                       //RFLR_IRQFLAGS_RXDONE |
									   //RFLR_IRQFLAGS_PAYLOADCRCERROR |
									   //RFLR_IRQFLAGS_VALIDHEADER |
									   RFLR_IRQFLAGS_TXDONE |
									   RFLR_IRQFLAGS_CADDONE |
									   RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
									   RFLR_IRQFLAGS_CADDETECTED );

    /** Configure DIO0/DIO1 for TIMEOUT and RX_DONE, don't use PAYLOADCRCERROR and VALIDHEADER interrupt */
	/** DIO0 RX Done */
	sx1276_rmw( REG_LR_DIOMAPPING1, RFLR_DIOMAPPING1_DIO0_MASK, RFLR_DIOMAPPING1_DIO0_00);

	/** DIO1 RX TIMEOUT */
	sx1276_rmw( REG_LR_DIOMAPPING1, RFLR_DIOMAPPING1_DIO1_MASK, RFLR_DIOMAPPING1_DIO1_00);

    if(timeout == 0){
        sx1276_set_mode(RFLR_OPMODE_RXCONTINUOUS);
    }else{
        if(sx1276_rf_g.rx_timeout != timeout){
            /** timeout is updated, refresh symbol timeout value */
            uint16_t symbols = sx1276_timeout_to_symbol(timeout, sx1276_rf_g.sf, sx1276_rf_g.bw);
            sx1276_set_symbol_timeout(symbols);
        }
        sx1276_set_mode(RFLR_OPMODE_RXSINGLE);
    }

    /** new timeout value update symbol timeout register */
    sx1276_rf_g.rx_timeout = timeout;

    /** restore previous IRQ status */
    irq_restore(irq_sta);
}

void sx1276_cad(void)
{
    int i;

	sx1276_set_mode(RFLR_OPMODE_STANDBY);

    /** set preamble register to rx preamble length */
	sx1276_set_preamble_reg(sx1276_rx_preamble_len_g);

    /** Enable CAD_DONE interrupt mode */
	sx1276_write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                       RFLR_IRQFLAGS_RXDONE |
									   RFLR_IRQFLAGS_PAYLOADCRCERROR |
									   RFLR_IRQFLAGS_VALIDHEADER |
									   RFLR_IRQFLAGS_TXDONE |
									   //RFLR_IRQFLAGS_CADDONE |
                                       //RFLR_IRQFLAGS_CADDETECTED |
									   RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );

    /** DIO0 CAD Done */
	sx1276_rmw( REG_LR_DIOMAPPING1, RFLR_DIOMAPPING1_DIO0_MASK, RFLR_DIOMAPPING1_DIO0_10);

    sx1276_set_mode(RFLR_OPMODE_CAD);

    for(i=0; i<SX1276_CAD_READ_RSSI_TIMES; i++){
        sx1276_rx_pkt_g.cad_rssi = sx1276_read_rssi();
    }
}

void sx1276_set_mode(uint8_t mode)
{
	uint8_t cur_mode;
    int8_t timeout;

	cur_mode = sx1276_read(REG_LR_OPMODE) & (~RFLR_OPMODE_MASK);

	if(mode != cur_mode){

		/** Reset SX1276 to standby mode */
		if(cur_mode != RFLR_OPMODE_SLEEP && cur_mode != RFLR_OPMODE_STANDBY){
			/** SX1276 is working, exit to standby mode first */
			sx1276_rmw(REG_LR_OPMODE, RFLR_OPMODE_MASK, RFLR_OPMODE_STANDBY);

            /** wait until sx1276 is in standby mode */
            timeout = 50;
			while(cur_mode != RFLR_OPMODE_STANDBY){
				cur_mode = sx1276_read(REG_LR_OPMODE) & (~RFLR_OPMODE_MASK);
                timeout--;
                if(timeout<=0){
                    break;
                }
			}
		}

		/** Set RF switch to right state */
		if(mode == RFLR_OPMODE_SLEEP){
			/** set rf switch mode idle to save power  */
			hal_sx1276_set_rf_switch_mode(SX1276_RF_SWITCH_IDLE);
		}else if(mode == RFLR_OPMODE_TX){
			/** Switch to TX mode */
			hal_sx1276_set_rf_switch_mode(SX1276_RF_SWITCH_TX);
		}else{
			/** Switch to RX mode */
			hal_sx1276_set_rf_switch_mode(SX1276_RF_SWITCH_RX);
		}

		sx1276_rmw(REG_LR_OPMODE, RFLR_OPMODE_MASK, mode); // Set new mode
	}

    /** If new mode is SLEEP/STANDBY, wait until mode is stable. */
    if( (mode == RFLR_OPMODE_SLEEP) || (mode == RFLR_OPMODE_STANDBY) ){
        timeout = 50;
        while(mode != cur_mode){
            cur_mode = sx1276_read(REG_LR_OPMODE) & (~RFLR_OPMODE_MASK);
            timeout--;
            if(timeout<=0){
                break;
            }
        }
	}
}

uint8_t sx1276_get_mode(void)
{
    return (sx1276_read(REG_LR_OPMODE) & (~RFLR_OPMODE_MASK));
}

void sx1276_set_modem( sx1276_modem_t modem )
{
	if(modem == LORA){
		sx1276_set_mode(RFLR_OPMODE_SLEEP);
		/** set lora mode */
		sx1276_rmw(REG_LR_OPMODE, RFLR_OPMODE_LONGRANGEMODE_MASK, RFLR_OPMODE_LONGRANGEMODE_ON);
	}else{
		sx1276_set_mode(RFLR_OPMODE_SLEEP);
		/** set fsk mode */
		sx1276_rmw(REG_LR_OPMODE, RFLR_OPMODE_LONGRANGEMODE_MASK, RFLR_OPMODE_LONGRANGEMODE_OFF);
	}
}

void sx1276_set_channel( uint32_t freq )
{
    // save frequency
    sx1276_rf_g.frf = freq;

    freq = ( uint32_t )( ( double )freq / ( double )SX1276_FREQ_STEP );
	sx1276_write( REG_LR_FRFMSB, ( uint8_t )( ( freq >> 16 ) & 0xFF ) );
	sx1276_write( REG_LR_FRFMID, ( uint8_t )( ( freq >> 8 ) & 0xFF ) );
	sx1276_write( REG_LR_FRFLSB, ( uint8_t )( freq & 0xFF ) );
}

uint32_t sx1276_get_channel( void )
{
	uint32_t freq;

	freq = sx1276_read( REG_LR_FRFMSB );
	freq <<= 8;
	freq |= sx1276_read( REG_LR_FRFMID );
	freq <<= 8;
	freq |= sx1276_read( REG_LR_FRFLSB );
	freq = ( uint32_t )( ( double )freq * ( double )SX1276_FREQ_STEP );

	return (uint32_t)freq;
}

void sx1276_set_sf(sx1276_sf_t sf)
{
	if(sf > SX1276_SF12 || sf < SX1276_SF6){
		return;
	}
	sx1276_rmw(REG_LR_MODEMCONFIG2, RFLR_MODEMCONFIG2_SF_MASK, (uint8_t)sf<<4);

    sx1276_rf_g.sf = sf;

	if(sf == SX1276_SF6){
		/** Optimization for SF6 */
		sx1276_rmw(0x31, 0x07, 0x05);
		sx1276_write(0x37, 0x0C);
	}else{
		/** reset value to default */
		sx1276_rmw(0x31, 0x07, 0x03);
		sx1276_write(0x37, 0x0A);
	}
}

void sx1276_set_preamble(sx1276_preamble_t preamble_type, uint16_t len)
{
	if(preamble_type == SX1276_TX_PREAMBLE){
		sx1276_tx_preamble_len_g = len;
	}else{
		sx1276_rx_preamble_len_g = len;
	}
}

void sx1276_set_preamble_reg(uint16_t len)
{
    if(len < 6){
        len = 6;
    }
    len -= 4;
	sx1276_write(REG_LR_PREAMBLEMSB, (uint8_t)(len>>8));
	sx1276_write(REG_LR_PREAMBLELSB, (uint8_t)len);
}

void sx1276_set_symbol_timeout(uint16_t val)
{
    if(val > 0x3FF){
        val = 0x3FF;
    }

	sx1276_rmw(REG_LR_MODEMCONFIG2, RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK, (uint8_t)(val>>8));
	sx1276_write(REG_LR_SYMBTIMEOUTLSB, (uint8_t)val);
}

void sx1276_set_tx_power(int8_t pow)
{
	uint8_t reg = 0;
	uint8_t MaxPower;
	uint8_t OutputPower;

	/** Output power: -1 ~ 14 (RFO), 2 ~ 17 (PA_BOOST)*/
	MaxPower = 7;

	if(hal_sx1276_get_tx_port(sx1276_rf_g.frf) == SX1276_TX_PORT_RFO)
	{
		/** TX pin RFO */
		reg |= RFLR_PACONFIG_PASELECT_RFO;
		/** Disable PA_BOOST for safe */
		sx1276_rmw(REG_LR_PADAC, RFLR_PADAC_20DBM_MASK, RFLR_PADAC_20DBM_OFF);
		if(pow < -1)
		{
			OutputPower = 0;	// Minimum output
		}
		else if(pow > 14)
		{
			OutputPower = 15;	// Maximum output
		}
		else
		{
			OutputPower = pow;
		}
	}else
	{
		/** TX pin PA_BOOST */
		reg |= RFLR_PACONFIG_PASELECT_PABOOST;
		if(pow > 17)
		{
			/** Enable 20dBm output */
			sx1276_rmw(REG_LR_PADAC, RFLR_PADAC_20DBM_MASK, RFLR_PADAC_20DBM_ON);
            if(pow>20)
			{
                pow = 20;
            }
			OutputPower = pow - 5;	// Maximum output(20dBm)
		}
		else if(pow < 2)
		{
			/** Disable 20dBm output */
			sx1276_rmw(REG_LR_PADAC, RFLR_PADAC_20DBM_MASK, RFLR_PADAC_20DBM_OFF);
			OutputPower = 2;
		}
		else
		{
			sx1276_rmw(REG_LR_PADAC, RFLR_PADAC_20DBM_MASK, RFLR_PADAC_20DBM_OFF);
			OutputPower = pow - 2;
		}
	}

	reg |= (MaxPower << 4) | OutputPower;
	sx1276_write(REG_LR_PACONFIG, reg);
}

void sx1276_set_coding_rate(sx1276_coding_rate_t cr)
{
	if(cr == 0 || cr > SX1276_CR4){
		while(1);
	}
	sx1276_rmw(REG_LR_MODEMCONFIG1, RFLR_MODEMCONFIG1_CODINGRATE_MASK, (uint8_t)cr<<1);
}

void sx1276_set_bandwidth(sx1276_bw_t bw)
{
	if(bw > SX1276_BW_500K){
		return;
	}

    sx1276_rf_g.bw = bw;

    sx1276_rmw(REG_LR_MODEMCONFIG1, RFLR_MODEMCONFIG1_BW_MASK, (uint8_t)bw<<4);
}

void sx1276_set_header_mode(sx1276_header_mode_t hdr_mode)
{
	if(hdr_mode == SX1276_HEADER_ENABLE){
		/** Packet with header */
		sx1276_rmw(REG_LR_MODEMCONFIG1, RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK,\
			RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF);
	}else{
		/** Packet without header */
		sx1276_rmw(REG_LR_MODEMCONFIG1, RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK,\
			RFLR_MODEMCONFIG1_IMPLICITHEADER_ON);
	}
}

void sx1276_set_crc_mode(sx1276_crc_mode_t crc_mode)
{
	if(crc_mode == SX1276_CRC_ON){
		sx1276_rmw(REG_LR_MODEMCONFIG2, RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK, \
			RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON);
	}else{
		sx1276_rmw(REG_LR_MODEMCONFIG2, RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK, \
			RFLR_MODEMCONFIG2_RXPAYLOADCRC_OFF);
	}
}

static void sx1276_set_low_datarate_optimization(sx1276_sf_t sf, sx1276_bw_t bw)
{
	uint8_t is_low_datarate = 0;
	switch(sf){
		case SX1276_SF6:
			break;
		case SX1276_SF7:
			if(bw < SX1276_BW_10K4){
				is_low_datarate = 1;
			}
			break;
		case SX1276_SF8:
			if(bw < SX1276_BW_20K8){
				is_low_datarate = 1;
			}
			break;
		case SX1276_SF9:
			if(bw < SX1276_BW_41K7){
				is_low_datarate = 1;
			}
			break;
		case SX1276_SF10:
			if(bw < SX1276_BW_125K){
				is_low_datarate = 1;
			}
			break;
		case SX1276_SF11:
			if(bw < SX1276_BW_250K){
				is_low_datarate = 1;
			}
			break;
		case SX1276_SF12:
			is_low_datarate = 1;
			break;
	}

#ifdef SX1276_LOWDATARATE_OPTIMIZATION_OFF
    /** Force enable lowdatarate optimization */
    is_low_datarate = 0;
#endif

#ifdef SX1276_LOWDATARATE_OPTIMIZATION_ON
    /** Force enable lowdatarate optimization */
    is_low_datarate = 1;
#endif

	if(is_low_datarate){
		sx1276_rmw( REG_LR_MODEMCONFIG3, RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK, \
				   RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON);
	}else{
		sx1276_rmw( REG_LR_MODEMCONFIG3, RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK, \
				   RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_OFF);
	}
}

void sx1276_set_iq(sx1276_iq_t iq)
{
    switch(iq){
    case SX1276_IQ_RX_IVT:
        sx1276_rmw(REG_LR_INVERTIQ, RFLR_INVERTIQ_RX_MASK, RFLR_INVERTIQ_RX_ON);
        break;
	case SX1276_IQ_RX_NML:
        sx1276_rmw(REG_LR_INVERTIQ, RFLR_INVERTIQ_RX_MASK, RFLR_INVERTIQ_RX_OFF);
        break;
	case SX1276_IQ_TX_IVT:
        sx1276_rmw(REG_LR_INVERTIQ, RFLR_INVERTIQ_TX_MASK, RFLR_INVERTIQ_TX_ON);
        break;
	case SX1276_IQ_TX_NML:
        sx1276_rmw(REG_LR_INVERTIQ, RFLR_INVERTIQ_TX_MASK, RFLR_INVERTIQ_TX_OFF);
        break;
	case SX1276_IQ_RXTX_IVT:
	case SX1276_IQ_TXRX_IVT:
        sx1276_rmw(REG_LR_INVERTIQ, RFLR_INVERTIQ_RX_MASK&RFLR_INVERTIQ_TX_MASK, RFLR_INVERTIQ_RX_ON|RFLR_INVERTIQ_TX_ON);
        break;
	case SX1276_IQ_RXTX_NML:
	case SX1276_IQ_TXRX_NML:
        sx1276_rmw(REG_LR_INVERTIQ, RFLR_INVERTIQ_RX_MASK&RFLR_INVERTIQ_TX_MASK, RFLR_INVERTIQ_RX_OFF|RFLR_INVERTIQ_TX_OFF);
        break;
    }
}

void sx1276_set_cad_thresh(sx1276_cad_thresh_t cad_thresh, uint8_t val)
{
    switch(cad_thresh){
    case SX1276_CAD_THRESH_PNR:
        sx1276_write(REG_LR_CAD_PNR, val);
        break;
    case SX1276_CAD_THRESH_PEAK_MIN:
        sx1276_write(REG_LR_CAD_PEAK_MIN, val);
        break;
    }
}

/** timeout in ms, return symbol numbers */
uint16_t sx1276_timeout_to_symbol(uint32_t timeout, sx1276_sf_t sf, sx1276_bw_t bw)
{
    uint32_t symbol_unit, to_us, symbol_num;

    symbol_unit = sx1276_symbol_length_tab[(bw-SX1276_BW_7K8)][(sf-SX1276_SF6)];
    to_us = timeout * 1000;

    symbol_num = (to_us-1)/symbol_unit + 1;

    if(symbol_num>0x3FF){
        symbol_num = 0x3FF;
    }

    return symbol_num;
}

int16_t sx1276_read_rssi(void)
{
    int16_t offset;

    if(sx1276_rf_g.frf<SX1276_LF_FREQ_MAX){
        offset = SX1276_RSSI_OFFSET_LF;
    }else{
        offset = SX1276_RSSI_OFFSET_HF;
    }

    return (offset + sx1276_read( REG_LR_RSSIVALUE ));
}

/** Formula: SNR = raw_snr/4;
If SNR >= 0 then RSSI = OFFSET+17/16*PacketRssi
If SNR < 0 then RSSI = OFFSET+PacketRssi+PacketSnr/4 */
void sx1276_read_pkt_snr_rssi(int8_t *snr, int16_t *rssi)
{
    uint8_t raw_rssi;
    int16_t offset;

    /** read packet SNR */
    *snr = ((int8_t)sx1276_read( REG_LR_PKTSNRVALUE ))/4;

    /** read packet rssi */
    raw_rssi = sx1276_read( REG_LR_PKTRSSIVALUE );

    if(sx1276_rf_g.frf<SX1276_LF_FREQ_MAX){
        /** LF */
        offset = SX1276_RSSI_OFFSET_LF;
    }else{
        /** HF */
        offset = SX1276_RSSI_OFFSET_HF;
    }

    *rssi = offset + (int16_t)raw_rssi;

    if(*snr<0){
        *rssi += *snr;
    }else{
        /** In sx1276 datasheet, RSSI = -164+16/15 * PacketRssi
            This fast calculation implement make RSSI = -164+17/16 * PacketRssi */
        /** raw_rssi>>4 => raw_rssi/16 */
        *rssi += (int16_t)(raw_rssi>>4);
    }
}

/** Get random number with sx1276 */
uint32_t sx1276_random(uint8_t bits)
{
    uint8_t i, mode, dio;
    uint32_t rnd = 0;

    /** check LoRa Mode, if LoRa tranceiver is busy, then return 0 */
    mode = sx1276_get_mode();
    if((mode == RFLR_OPMODE_TX) || (mode == RFLR_OPMODE_RXSINGLE) || \
        (mode == RFLR_OPMODE_CAD) ){
        return 0;
    }

    dio = sx1276_read(REG_LR_IRQFLAGSMASK);

    // Disable all LoRa modem interrupts
    sx1276_write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                  RFLR_IRQFLAGS_RXDONE |
                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                  RFLR_IRQFLAGS_VALIDHEADER |
                  RFLR_IRQFLAGS_TXDONE |
                  RFLR_IRQFLAGS_CADDONE |
                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                  RFLR_IRQFLAGS_CADDETECTED );

    // Set radio in continuous reception
    sx1276_set_mode(RFLR_OPMODE_RXCONTINUOUS);

    for( i = 0; i < bits; i++ )
    {
        delay_ms(1);
        // Unfiltered RSSI value reading. Only takes the LSB value
        rnd <<= 1;
        rnd |= ( ( uint32_t )sx1276_read( REG_LR_RSSIWIDEBAND ) & 0x01 );
    }

    /** restore mode */
    sx1276_write( REG_LR_IRQFLAGSMASK, dio);
    sx1276_set_mode(mode);

    return rnd;
}

/** low level functions */
uint8_t sx1276_read(uint8_t addr)
{
	uint8_t ret;

    /** disable IRQ, and get current IRQ status */
    irq_state_t irq_sta = irq_disable();

    HAL_SX1276_NSS_L();

	hal_spi_wr( (~0x80) & addr );
	ret = hal_spi_wr( 0x00 );

	HAL_SX1276_NSS_H();

    /** restore previous IRQ status */
    irq_restore(irq_sta);

	return ret;
}

void sx1276_write(uint8_t addr, uint8_t data)
{
    /** disable IRQ, and get current IRQ status */
    irq_state_t irq_sta = irq_disable();

	HAL_SX1276_NSS_L();

	hal_spi_wr( 0x80 | addr );
	hal_spi_wr( data );

	HAL_SX1276_NSS_H();

    /** restore previous IRQ status */
    irq_restore(irq_sta);
}

/** read-modify write register, 'mask' is used to mask corresponding bits */
void sx1276_rmw(uint8_t addr, uint8_t mask, uint8_t val)
{
	uint8_t ret;

    /** disable IRQ, and get current IRQ status */
    irq_state_t irq_sta = irq_disable();

	/** read register */
	HAL_SX1276_NSS_L();

	hal_spi_wr( (~0x80) & addr );
	ret = hal_spi_wr( 0x00 );

	HAL_SX1276_NSS_H();

	/** modify the data */
	ret = (ret & mask) | (val & ~mask);

	/** write data back */
	HAL_SX1276_NSS_L();

	hal_spi_wr( 0x80 | addr );
	hal_spi_wr( ret );

	HAL_SX1276_NSS_H();

    /** restore previous IRQ status */
    irq_restore(irq_sta);
}

void sx1276_read_burst(uint8_t addr, uint8_t *buf, uint16_t len)
{
	int i;

    /** disable IRQ, and get current IRQ status */
    irq_state_t irq_sta = irq_disable();

	HAL_SX1276_NSS_L();

	hal_spi_wr( (~0x80) & addr );
	for(i = 0; i<len; i++){
		buf[i] = hal_spi_wr( 0x00 );
	}

	HAL_SX1276_NSS_H();

    /** restore previous IRQ status */
    irq_restore(irq_sta);
}

void sx1276_write_burst(uint8_t addr, uint8_t *buf, uint16_t len)
{
	int i;

    /** disable IRQ, and get current IRQ status */
    irq_state_t irq_sta = irq_disable();

	HAL_SX1276_NSS_L();

	hal_spi_wr( 0x80 | addr );
	for(i = 0; i<len; i++){
		hal_spi_wr( *buf++ );
	}

	HAL_SX1276_NSS_H();

    /** restore previous IRQ status */
    irq_restore(irq_sta);
}

void sx1276_read_fifo(uint8_t *buf, uint16_t len)
{
	sx1276_read_burst(REG_LR_FIFO, buf, len);
}

void sx1276_write_fifo(uint8_t *buf, uint16_t len)
{
	sx1276_write_burst(REG_LR_FIFO, buf, len);
}

void sx1276_irq_dio0(void)
{
	uint8_t dio0, irq_flag;
	
	dio0 = sx1276_read(REG_LR_DIOMAPPING1) & ~RFLR_DIOMAPPING1_DIO0_MASK;
	irq_flag = sx1276_read(REG_LR_IRQFLAGS);

	switch(dio0){
	  case RFLR_DIOMAPPING1_DIO0_RX_DONE:
		if(irq_flag & RFLR_IRQFLAGS_RXDONE){
//			sx1276_write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE ); // Clear Irq

            sx1276_read_pkt_snr_rssi( &(sx1276_rx_pkt_g.snr), &(sx1276_rx_pkt_g.rssi) );

//            sx1276_rx_pkt_g.hcnt = ((uint16_t)sx1276_read(REG_LR_RXHEADERCNTVALUEMSB)<<8) | sx1276_read(REG_LR_RXHEADERCNTVALUELSB);
//            sx1276_rx_pkt_g.pcnt = ((uint16_t)sx1276_read(REG_LR_RXPACKETCNTVALUEMSB)<<8) | sx1276_read(REG_LR_RXPACKETCNTVALUELSB);

            /** Check valid header */
			if( irq_flag & RFLR_IRQFLAGS_VALIDHEADER ){
				// Clear Irq
//				sx1276_write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_VALIDHEADER );
			}else{
                sx1276_write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_VALIDHEADER | \
                                                RFLR_IRQFLAGS_PAYLOADCRCERROR | \
                                                RFLR_IRQFLAGS_RXDONE );
                if(sx1276_callback_g != NULL){
                    sx1276_rx_pkt_g.error = SX1276_ERROR_HEADER_IRQ;
					sx1276_callback_g(SX1276_RX_ERROR, &sx1276_rx_pkt_g);
				}
                break;  // exit RX_DONE case;
            }

            /** Check header info */
            if( (sx1276_read(REG_LR_MODEMCONFIG2) & RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON) && \
                    !(sx1276_read(REG_LR_HOPCHANNEL) & RFLR_HOPCHANNEL_CRCONPAYLOAD_ON) ){
                // Check
				if(sx1276_callback_g != NULL){
                    sx1276_rx_pkt_g.error = SX1276_ERROR_HEADER_INFO;
					sx1276_callback_g(SX1276_RX_ERROR, &sx1276_rx_pkt_g);
				}
				break;	// exit RX_DONE case;
            }

			/** Check payload crc */
			if( irq_flag & RFLR_IRQFLAGS_PAYLOADCRCERROR ){
				// Clear Irq
//				sx1276_write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );
                sx1276_write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_VALIDHEADER | \
                                                RFLR_IRQFLAGS_PAYLOADCRCERROR | \
                                                RFLR_IRQFLAGS_RXDONE );
				if(sx1276_callback_g != NULL){
                    sx1276_rx_pkt_g.error = SX1276_ERROR_CRC;
					sx1276_callback_g(SX1276_RX_ERROR, &sx1276_rx_pkt_g);
				}
				break;	// exit RX_DONE case;
			}

            /* Read Coding Rate */
            sx1276_rx_pkt_g.cr = sx1276_read(REG_LR_MODEMSTAT)>>5;

			/** read packet SNR and RSSI */
            sx1276_read_pkt_snr_rssi( &(sx1276_rx_pkt_g.snr), &(sx1276_rx_pkt_g.rssi) );

			/** read payload from fifo */
			sx1276_rx_pkt_g.len = sx1276_read( REG_LR_RXNBBYTES );							// Get packet length
			sx1276_write( REG_LR_FIFOADDRPTR, sx1276_read( REG_LR_FIFORXCURRENTADDR ) );	// Set start address for reading
            sx1276_read_fifo(sx1276_rx_pkt_g.buf, sx1276_rx_pkt_g.len );					// Read all data out

            /** Clear all irq */
            sx1276_write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_VALIDHEADER | \
                                                RFLR_IRQFLAGS_PAYLOADCRCERROR | \
                                                RFLR_IRQFLAGS_RXDONE );
			/** call callback function */
			if(sx1276_callback_g != NULL){
				sx1276_callback_g(SX1276_RX_DONE, &sx1276_rx_pkt_g);
			}
		}
		break;
	  case RFLR_DIOMAPPING1_DIO0_TX_DONE:
		/** Make sure interrupt generated */
		if(irq_flag & RFLR_IRQFLAGS_TXDONE){
			sx1276_write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE ); // Clear Irq
			if(sx1276_callback_g != NULL){
				sx1276_callback_g(SX1276_TX_DONE, NULL);
			}
		}
		break;
	  case RFLR_DIOMAPPING1_DIO0_CAD_DONE:
        /** Check to make sure CAD_DONE interrupt generated */
		if(irq_flag & RFLR_IRQFLAGS_CADDONE){
            /** Check CAD detected flag */
            if(irq_flag & RFLR_IRQFLAGS_CADDETECTED){
                /** Clear CAD flags together */
                sx1276_write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE );
                if(sx1276_callback_g != NULL){
                    sx1276_callback_g(SX1276_CAD_DETECTED, &sx1276_rx_pkt_g);
                }
            }else{
                /** Clear CAD_DONE flag */
                sx1276_write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE ); // Clear Irq
                if(sx1276_callback_g != NULL){
                    sx1276_callback_g(SX1276_CAD_DONE, &sx1276_rx_pkt_g);
                }
            }
		}
		break;
	  default:
		break;
	}
}

void sx1276_irq_dio1(void)
{
	uint8_t dio1, irq_flag;
	

	dio1 = sx1276_read(REG_LR_DIOMAPPING1) & ~RFLR_DIOMAPPING1_DIO1_MASK;
	irq_flag = sx1276_read(REG_LR_IRQFLAGS);

	switch(dio1){
    case RFLR_DIOMAPPING1_DIO1_RX_TIMEOUT:
		/** Make sure interrupt generated */
		if(irq_flag & RFLR_IRQFLAGS_RXTIMEOUT){
			sx1276_write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXTIMEOUT ); // Clear Irq
			if(sx1276_callback_g != NULL){
				sx1276_callback_g(SX1276_RX_TIMEOUT, NULL);
			}
		}
		break;
    case RFLR_DIOMAPPING1_DIO1_FHSS_CHANGE_CHANNEL:
        /** Make sure interrupt generated */
		if(irq_flag & RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL){
			sx1276_write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL ); // Clear Irq
		}
		break;
    case RFLR_DIOMAPPING1_DIO0_CAD_DETECTED:
		break;
    default:
		break;
	}
}

void sx1276_irq_dio2(void)
{

}

void sx1276_irq_dio3(void)
{

}

void sx1276_irq_dio4(void)
{

}

void sx1276_irq_dio5(void)
{

}
