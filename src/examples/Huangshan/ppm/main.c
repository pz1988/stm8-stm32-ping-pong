/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: RX Continuous

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#include <stdio.h>

#include "hardware.h"
#include "hal-sys-clk.h"
#include "sys-tick.h"
#include "uart.h"
#include "sx1276.h"
#include "led.h"
#include "lcd.h"
#include "key.h"

#define LED_GREEN               LED1
#define LED_RED                 LED0
#define LED_BLUE                LED2

sx1276_rx_pkt_t *rx_pkt;
uint8_t rx_done_flag = 0;
uint8_t tx_done_flag = 0;
uint8_t rx_error_flag = 0;
uint8_t rx_timeout_flag = 0;

typedef struct{
    uint32_t index_old;
    uint32_t index;
    uint32_t lost;
    uint32_t count;
}rx_cnt_t;

rx_cnt_t upl;
rx_cnt_t downl;
rx_cnt_t *rx_cnt_ptr;

void sx1276_event(sx1276_event_t state, void *ptr)
{
	switch(state){
    case SX1276_TX_DONE:
        tx_done_flag = 1;
	  	break;
    case SX1276_RX_DONE:
		rx_pkt = ptr;
		rx_done_flag = 1;
		break;
    case SX1276_RX_TIMEOUT:
        rx_timeout_flag = 1;
	  	break;
    case SX1276_RX_ERROR:
        rx_pkt = ptr;
        rx_error_flag = 1;
	  	break;
	  	break;
	}
}

void update_ui(void)
{
    uint8_t buf[20];

//    sprintf((char*)buf, "Mode%2d RSSI:%4d", mode, rx_pkt->rssi);
//    lcd_putbuf(0, 0, buf, 16);
//    sprintf((char*)buf, "LEN:%3d  SNR:%3d", rx_pkt->len, rx_pkt->snr);
//    lcd_putbuf(1, 0, buf, 16);
//
    sprintf((char*)buf, "URX:  %9ld  ", upl.count);
    lcd_putbuf(0, 0, buf, 16);
    sprintf((char*)buf, "ULOST:%9ld  ", upl.lost);
    lcd_putbuf(1, 0, buf, 16);

    sprintf((char*)buf, "DRX:  %9ld  ", downl.count);
    lcd_putbuf(2, 0, buf, 16);
    sprintf((char*)buf, "DLOST:%9ld  ", downl.lost);
    lcd_putbuf(3, 0, buf, 16);
}

void main()
{
  	uart_config_t uart_conf;
	sx1276_config_t sx1276_config;

	DISABLE_IRQ();

	led_init();
	hal_clk_init();
	sys_tick_init();
    key_init();

	uart_conf.baud = 9600;
	uart_conf.word_length = 8;
	uart_conf.parity = NONE;
	uart_conf.stop_bits = 1;
	uart_init(&uart_conf);

	ENABLE_IRQ();

    lcd_init();

    /** SX1276 software reset */
	sx1276_init(LORA, sx1276_event);

    /** Initial SX1276 */
	sx1276_config.frequency = 866000000;
	sx1276_config.spread_factor = SX1276_SF9;
	sx1276_config.bandwidth = SX1276_BW_125K;
	sx1276_config.coding_rate = SX1276_CR1;
	sx1276_config.crc_mode = SX1276_CRC_ON;
	sx1276_config.header_mode = SX1276_HEADER_ENABLE;
	sx1276_config.payload_len = 0;		// Set
	sx1276_config.tx_power = 20;
	sx1276_config.tx_preamble_len = 15;
	sx1276_config.rx_preamble_len = 15;
	sx1276_set_config(&sx1276_config);

	sx1276_set_symbol_timeout(0x3FF);
	sx1276_receive(0);

    upl.index_old = 0;
    upl.index = 0;
    upl.lost = 0;
    upl.count = 0;

    downl.index_old = 0;
    downl.index = 0;
    downl.lost = 0;
    downl.count = 0;

	while(1){
		if(rx_done_flag){
			rx_done_flag = 0;

            led_blink(LED_GREEN, 100);

            if(rx_pkt->buf[4] == 0x55 && rx_pkt->buf[5] == 0xAA){
                rx_cnt_ptr = &upl;
            }else{
                rx_cnt_ptr = &downl;
            }

            rx_cnt_ptr->index = rx_pkt->buf[0];
            rx_cnt_ptr->index <<= 8;
            rx_cnt_ptr->index += rx_pkt->buf[1];
            rx_cnt_ptr->index <<= 8;
            rx_cnt_ptr->index += rx_pkt->buf[2];
            rx_cnt_ptr->index <<= 8;
            rx_cnt_ptr->index += rx_pkt->buf[3];
            if(rx_cnt_ptr->index  == (rx_cnt_ptr->index_old + 1) ){
                rx_cnt_ptr->count++;
            }else if( rx_cnt_ptr->index <= rx_cnt_ptr->index_old ){
                rx_cnt_ptr->lost = 0;
                rx_cnt_ptr->count = 1;
                update_ui();
            }else{
                rx_cnt_ptr->count++;
                rx_cnt_ptr->lost += (rx_cnt_ptr->index - rx_cnt_ptr->index_old - 1);
                if(rx_cnt_ptr->index_old == 0){
                    /** Restart RX counting */
                    rx_cnt_ptr->lost = 0;
                    rx_cnt_ptr->count = 1;
                }else{
                    /** Lost packets */
                }
                update_ui();
            }
            rx_cnt_ptr->index_old = rx_cnt_ptr->index;

            update_ui();
		}

        if(rx_error_flag){
            rx_error_flag = 0;
            led_blink(LED_BLUE, 100);
        }

        if(rx_timeout_flag){
            rx_timeout_flag = 0;
            led_blink(LED_BLUE, 100);
        }

        /** Get key and make response */
        switch(key_get()){
        case KEY2:

            break;
        case KEY3:
            update_ui();
            break;
        }

        led_evt();
	}
}
