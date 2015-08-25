/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: RX Single

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

#define TIMEOUT                 (300)
#define RX_PERIOD               (50)

#define LED_GREEN               LED1
#define LED_RED                 LED0
#define LED_BLUE                LED2

typedef enum {
    SYS_STA_IDLE,
    SYS_STA_RX,
}sys_sta_t;

sx1276_rx_pkt_t *rx_pkt;
sx1276_event_t lora_evt;
sys_sta_t sys_sta = SYS_STA_IDLE;
uint8_t lora_busy = 0;

void dump_reg(void)
{
	int i;
    uint8_t tmp_buf[0x80];
    tmp_buf[0] = 0;
	sx1276_read_burst(1, tmp_buf+1, 0x7F);
	for(i=0; i<8; i++){
		uart_putbuf_hex(tmp_buf+16*i, 16);
		uart_putstring("\r\n");
	}
}

void dump_buf(sx1276_rx_pkt_t *pkt)
{
	int i, len;

    len = pkt->len;

    printf("LEN:%d, RSSI:%d SNR:%d\r\n", pkt->len, pkt->rssi, pkt->snr);

    for(i=0; i<len && len>16; i++){
		uart_putbuf_hex(pkt->buf+16*i, 16);
		uart_putstring("\r\n");
        len-=16;
	}

    if(len){
        uart_putbuf_hex(pkt->buf+16*i, len);
        uart_putstring("\r\n");
    }
}

void sx1276_event(sx1276_event_t evt, void *ptr)
{
    if(lora_evt != SX1276_IDLE){
        return;
    }

    lora_evt = evt;
    if(ptr != NULL ){
        rx_pkt = ptr;
    }
}

void main()
{
  	uart_config_t uart_conf;
	sx1276_config_t sx1276_config;
    uint32_t timestamp;
    uint32_t rx_ts;

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

    /** SX1276 software reset */
	sx1276_init(LORA, sx1276_event);

    /** Initial SX1276 */
	sx1276_config.frequency = 483000000;
	sx1276_config.spread_factor = SX1276_SF12;
	sx1276_config.bandwidth = SX1276_BW_125K;
	sx1276_config.coding_rate = SX1276_CR1;
	sx1276_config.crc_mode = SX1276_CRC_ON;
	sx1276_config.header_mode = SX1276_HEADER_ENABLE;
	sx1276_config.payload_len = 0;		// Set
	sx1276_config.tx_power = 20;
	sx1276_config.tx_preamble_len = 15;
	sx1276_config.rx_preamble_len = 12;
	sx1276_set_config(&sx1276_config);

    timestamp = millis();

    rx_ts = millis();
    sys_sta = SYS_STA_RX;
    lora_evt = SX1276_IDLE;

    printf("SX1276 RxSingle test example\r\n");

	while(1){
        if( uart_readable() ){
            switch(uart_getchar()){
            case 'g':
                dump_reg();
                break;
            case 'w':

                break;
            case 's':

                break;
            default:
                break;
            }
		}

        if(lora_evt != SX1276_IDLE){
            /** Handle SX1276 Event */
            switch(lora_evt){
            case SX1276_RX_DONE:
                led_blink(LED_GREEN, 100);
                printf("RX DONE %ldms\r\n", millis()-timestamp);
                dump_buf(rx_pkt);
                sys_sta = SYS_STA_RX;
                rx_ts = millis();
                break;
            case SX1276_RX_ERROR:
                led_blink(LED_BLUE, 100);
                printf("RX ERROR %ldms\r\n", millis()-timestamp);
                sys_sta = SYS_STA_RX;
                rx_ts = millis();
                break;
            case SX1276_RX_TIMEOUT:
            {
                uint32_t t;
                led_blink(LED_BLUE, 100);
                t = millis() - timestamp;
                if( t>33 ){
                    printf("RX TIMEOUT %ldms\r\n", t);
                }
                //printf("RX TIMEOUT %ldms\r\n", millis()-timestamp);
                sys_sta = SYS_STA_RX;
                rx_ts = millis();
            }
                break;
            case SX1276_TX_DONE:
            case SX1276_CAD_DONE:
            case SX1276_CAD_DETECTED:
                printf("Unexpected event\r\n");
                break;
            case SX1276_IDLE:
                break;
            default:
                printf("Unknown event\r\n");
                break;
            }
            lora_evt = SX1276_IDLE;
        }

        /** Handle test routine status, RX Single in period */
        switch(sys_sta){
        case SYS_STA_IDLE:
            break;
        case SYS_STA_RX:
            if(millis() - rx_ts > RX_PERIOD){
                rx_ts = millis();
                sys_sta = SYS_STA_IDLE;
                sx1276_receive(TIMEOUT);
                timestamp = millis();
            }
            break;
        }

        /** Get key and make response */
        switch(key_get()){
        case KEY2:

            break;
        case KEY3:

            break;
        }

        led_evt();
	}
}
