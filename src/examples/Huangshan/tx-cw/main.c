/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: TX Continuous Wave

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/
#include "hal-sys-clk.h"
#include "sys-tick.h"
#include "sx1276.h"
#include "led.h"

void main()
{
	uint32_t timestamp;
	sx1276_config_t sx1276_config;

	DISABLE_IRQ();

	hal_clk_init();
	sys_tick_init();

	ENABLE_IRQ();

    led_init();
	sx1276_init(LORA, NULL);

	sx1276_config.frequency = 433400000;
	sx1276_config.spread_factor = SX1276_SF7;
	sx1276_config.bandwidth = SX1276_BW_125K;
	sx1276_config.coding_rate = SX1276_CR1;
	sx1276_config.crc_mode = SX1276_CRC_ON;
	sx1276_config.header_mode = SX1276_HEADER_ENABLE;
	sx1276_config.payload_len = 0;		// Set in HEADER disable mode
	sx1276_config.tx_power = 20;
	sx1276_config.tx_preamble_len = 12;
	sx1276_config.rx_preamble_len = 12;
	sx1276_set_config(&sx1276_config);

    /** Enter HF/LF test mode */
    if(sx1276_config.frequency < SX1276_LF_FREQ_MAX){
        sx1276_write( 0x01, 0x88 );
    }else{
        sx1276_write( 0x01, 0x80 );
    }
    sx1276_write( 0x3D, 0xA1 );
    sx1276_write( 0x36, 0x01 );
    sx1276_write( 0x1e, 0x08 );

    /** Enable TX to enter continuous wave transmitting mode */
    sx1276_send(NULL, 0, 0);

    /** Get system tick */
    timestamp = millis();

	while(1){
		/** Blink LED every 1s*/
        if( millis() - timestamp > 1000){
			timestamp = millis();
            led_blink(LED0, 100);
		}

        /** LED event polling */
        led_evt();
	}
}