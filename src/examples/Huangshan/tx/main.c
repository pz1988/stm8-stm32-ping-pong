/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: TX LoRa packet in fixed period

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#include "hardware.h"
#include "hal-sys-clk.h"
#include "sys-tick.h"
#include "uart.h"
#include "sx1276.h"
#include "led.h"

#include <stdio.h>

#define LED_GREEN               LED1
#define LED_RED                 LED0
#define LED_BLUE                LED2

#define TX_WAIT_TIME            (100) //ms
#define TX_LEN                  (255)

typedef enum{
    SYS_STA_IDLE,
    SYS_STA_TX,
}sys_sta_t;

sys_sta_t sys_sta = SYS_STA_IDLE;

int i;
uint8_t tmp_buf[270];

sx1276_rx_pkt_t *rx_pkt;
uint8_t rx_done_flag = 0;
uint8_t tx_done_flag = 0;
uint32_t tx_count = 0;
uint32_t tx_timestamp;

int tx_len;

void dump_reg(void)
{
	int i;
	__disable_interrupt();
	sx1276_read_burst(1, tmp_buf+1, 0x7F);
	__enable_interrupt();
	for(i=0; i<8; i++){
		uart_putbuf_hex(tmp_buf+16*i, 16);
		uart_putstring("\r\n");
	}
}

void dump_fifo(void)
{
	int i;

	uart_putstring("SX1276 LoRa FIFO:\r\n");

	sx1276_write( REG_LR_FIFOADDRPTR, 0 );

	sx1276_read_fifo(tmp_buf+0, 256);
	for(i=0; i<16; i++){
		uart_putbuf_hex(tmp_buf+16*i, 16);
		uart_putstring("\r\n");
	}
}

void dump_buf(uint8_t *buf, int len)
{
	int i;

	uart_putstring("TX:\r\n");

    for(i=0; i<len && len>16; i++){
		uart_putbuf_hex(buf+16*i, 16);
		uart_putstring("\r\n");
        len-=16;
	}

    if(len){
        uart_putbuf_hex(buf+16*i, len);
        uart_putstring("\r\n");
    }
}

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
	  	break;
	}
}

uint8_t check_sum(uint8_t *buf, int len)
{
    uint8_t sum = 0;
    int i;
    for(i=0; i<len; i++){
        sum += buf[i];
    }
    return sum;
}

void update_tx_len(void)
{
    static uint8_t count = 0;
    count++;
    if(count == 10){
        count = 0;
        tx_len--;
        if(tx_len == 10){
            tx_len = TX_LEN;
        }
    }
    tx_len = TX_LEN;
}

void tx_pkt(uint32_t count)
{
    int i;
    /** FIFO Test */
    for(i=4; i<tx_len-1; i++){
        tmp_buf[i] = i;
    }
    tmp_buf[0] = 0;
    tmp_buf[0] = (tx_count>>24)&0xFF;
    tmp_buf[1] = (tx_count>>16)&0xFF;
    tmp_buf[2] = (tx_count>>8)&0xFF;
    tmp_buf[3] = (tx_count>>0)&0xFF;
    tmp_buf[tx_len-1] = check_sum(tmp_buf, tx_len-1);
    sx1276_set_mode(RFLR_OPMODE_SLEEP);
    sx1276_send(tmp_buf, tx_len, 0);
    update_tx_len();
}

void main()
{
  	uart_config_t uart_conf;
	sx1276_config_t sx1276_config;

	__disable_interrupt();

    led_init();

	hal_clk_init();
	sys_tick_init();

	uart_conf.baud = 9600;
	uart_conf.word_length = 8;
	uart_conf.parity = NONE;
	uart_conf.stop_bits = 1;
	uart_init(&uart_conf);

	__enable_interrupt();

	sx1276_init(LORA, sx1276_event);

    uart_putstring("REG: (Software Reset)\r\n");
	dump_reg();

	sx1276_config.frequency = 866000000;
	sx1276_config.spread_factor = SX1276_SF9;
	sx1276_config.bandwidth = SX1276_BW_125K;
	sx1276_config.coding_rate = SX1276_CR1;
	sx1276_config.crc_mode = SX1276_CRC_ON;
	sx1276_config.header_mode = SX1276_HEADER_ENABLE;
	sx1276_config.payload_len = 0;
	sx1276_config.tx_power = 20;
	sx1276_config.tx_preamble_len = 15;
	sx1276_config.rx_preamble_len = 15;
	sx1276_set_config(&sx1276_config);

    //sx1276_set_iq(SX1276_IQ_TX_IVT);

    uart_putstring("REG: (Initialized)\r\n");
	dump_reg();

    tx_len = TX_LEN;
    tx_count = 0;
    tx_pkt(tx_count);
    sys_sta = SYS_STA_TX;

    led_blink(LED_RED, 100);

    /** Check content of tmp buffer */
    //dump_buf(tmp_buf, tx_len);

    tx_timestamp = millis();

	while(1){
		if( uart_readable() ){
			//uart_putchar(uart_getchar());
            switch(uart_getchar()){
            case 'g':
                dump_reg();
                break;
            case 't':
                uart_putstring("tx\r\n");
                sx1276_set_mode(RFLR_OPMODE_SLEEP);
                sx1276_send(tmp_buf, tx_len, 0);
                break;
            case 's':
                printf("Sleep Mode\r\n");
                sx1276_set_mode(RFLR_OPMODE_SLEEP);
                break;
            }
		}
        switch(sys_sta){
        case SYS_STA_IDLE:
            if( millis() - tx_timestamp > TX_WAIT_TIME){
                tx_timestamp = millis();
                tx_count++;
                tx_pkt(tx_count);
                led_blink(LED_RED, 100);
                sys_sta = SYS_STA_TX;
            }
            break;
        case SYS_STA_TX:
            if(tx_done_flag){
                tx_done_flag = 0;
                tx_timestamp = millis();

                sys_sta = SYS_STA_IDLE;
            }
            break;
        default:
            break;
        }

		if(rx_done_flag){
			rx_done_flag = 0;
		}

        led_evt();
	}
}