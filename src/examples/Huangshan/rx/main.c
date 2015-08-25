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

uint32_t rx_count_bak = 0;
uint32_t rx_count = 0;
uint8_t rx_check_sum;
uint32_t rx_pkt_lost_cnt = 0;
uint32_t rx_pkt_cnt = 0;

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

uint8_t check_sum(uint8_t *buf, int len)
{
    uint8_t sum = 0;
    int i;

    DISABLE_IRQ();

    for(i=0; i<len; i++){
        sum += buf[i];
    }

    ENABLE_IRQ();

    return sum;
}

/* 0:Normal 1:RX->Standby->RX 2:RX->Standby->Set Frequency->RX */
#define MODE_MAX                (4)
uint8_t mode = 3;

void update_ui(void)
{
    uint8_t buf[20];

    sprintf((char*)buf, "Mode%2d RSSI:%4d", mode, rx_pkt->rssi);
    lcd_putbuf(0, 0, buf, 16);
    sprintf((char*)buf, "LEN:%3d  SNR:%3d", rx_pkt->len, rx_pkt->snr);
    lcd_putbuf(1, 0, buf, 16);
    sprintf((char*)buf, "RX:  %9ld  ", rx_pkt_cnt);
    lcd_putbuf(2, 0, buf, 16);
    sprintf((char*)buf, "LOST:%9ld  ", rx_pkt_lost_cnt);
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

	while(1){
		if(rx_done_flag){
			rx_done_flag = 0;
            switch(mode){
            case 0:
                break;
            case 1:
                sx1276_set_mode(RFLR_OPMODE_STANDBY);
                sx1276_receive(0);
                break;
            case 2:
                sx1276_set_mode(RFLR_OPMODE_STANDBY);
                delay_ms(5);
                sx1276_receive(0);
                break;
            case 3:
                sx1276_receive(0);
                break;
            case 4:
                sx1276_set_mode(RFLR_OPMODE_SLEEP);
                sx1276_receive(0);
                break;
            }

            led_blink(LED_GREEN, 100);

            rx_count = rx_pkt->buf[0];
            rx_count <<= 8;
            rx_count += rx_pkt->buf[1];
            rx_count <<= 8;
            rx_count += rx_pkt->buf[2];
            rx_count <<= 8;
            rx_count += rx_pkt->buf[3];
            if(rx_count == (rx_count_bak + 1) ){
                rx_pkt_cnt++;
            }else if( rx_count <= rx_count_bak ){
                rx_pkt_lost_cnt = 0;
                rx_pkt_cnt = 1;
                update_ui();
            }else{
                rx_pkt_cnt++;
                rx_pkt_lost_cnt += (rx_count - rx_count_bak - 1);
                if(rx_count_bak == 0){
                    /** Restart RX counting */
                    rx_pkt_lost_cnt = 0;
                    rx_pkt_cnt = 1;
                }else{
                    /** Lost packets */
                }
                update_ui();
            }
            rx_count_bak = rx_count;
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
            mode++;
            if(mode > MODE_MAX){
                mode = 0;
            }
            rx_count_bak = 0;
            rx_count = 0;
            rx_pkt_lost_cnt = 0;
            rx_pkt_cnt = 0;
            update_ui();
            break;
        case KEY3:
            update_ui();
            break;
        }

        led_evt();
	}
}
