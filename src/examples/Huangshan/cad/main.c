/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: Example which shows how to use CAD

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

#define CAD_INTERVAL            (1)

#define LED_GREEN               LED1
#define LED_RED                 LED0
#define LED_BLUE                LED2

int i;
uint8_t tmp_buf[270];

sx1276_rx_pkt_t *rx_pkt;
uint8_t rx_done_flag = 0;
uint8_t rx_error_flag = 0;
uint8_t rx_timeout_flag = 0;

uint8_t tx_done_flag = 0;

uint8_t cad_done_flag = 0;
uint8_t cad_detected_flag = 0;

int16_t cad_rssi;

uint32_t rx_count_bak = 0;
uint32_t rx_count = 0;
uint8_t rx_check_sum;
uint32_t rx_pkt_lost_cnt = 0;
uint32_t rx_pkt_cnt = 0;

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

void dump_buf(uint8_t *buf, int len, int16_t rssi, int8_t snr)
{
	int i;

    printf("RX: LEN:%d RSSI:%d SNR:%d\r\n", rx_pkt->len, rssi, snr);

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
	  	hal_led_toggle(LED0);
        tx_done_flag = 1;
	  	break;
	  case SX1276_RX_DONE:
		hal_led_toggle(LED1);
		rx_pkt = ptr;
		rx_done_flag = 1;
		break;
	  case SX1276_RX_TIMEOUT:
	  	hal_led_toggle(LED2);
        rx_timeout_flag = 1;
	  	break;
      case SX1276_RX_ERROR:
	  	rx_error_flag = 1;
	  	break;
      case SX1276_CAD_DONE:
	  	cad_done_flag = 1;
        rx_pkt = ptr;
	  	break;
      case SX1276_CAD_DETECTED:
	  	cad_detected_flag++;
        rx_pkt = ptr;
	  	break;
	}
}

typedef enum{
    SYS_STA_IDLE,
    SYS_STA_CAD,
    SYS_STA_CAD2,
    SYS_STA_RX,
}sys_sta_t;

sys_sta_t sys_sta = SYS_STA_CAD;
uint32_t cad_timestamp;
uint32_t rx_timestamp;
uint32_t cnt;

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

uint32_t cad_detected_count = 0;
uint32_t cad1_count = 0;
uint32_t cad2_count = 0;
uint32_t cad_rx_count = 0;

uint8_t cad_pnr = 0x1c;
uint8_t cad_peak_min = 0x0a;

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

	sx1276_config.frequency = 510000000;
	sx1276_config.spread_factor = SX1276_SF12;
	sx1276_config.bandwidth = SX1276_BW_125K;
	sx1276_config.coding_rate = SX1276_CR1;
	sx1276_config.crc_mode = SX1276_CRC_ON;
	sx1276_config.header_mode = SX1276_HEADER_ENABLE;
	sx1276_config.payload_len = 0;		// Set
	sx1276_config.tx_power = 20;
	sx1276_config.tx_preamble_len = 8;
	sx1276_config.rx_preamble_len = 8;
	sx1276_set_config(&sx1276_config);

    //invert both RX and TX
//    sx1276_set_mode(RFLR_OPMODE_STANDBY);
//    sx1276_set_iq(SX1276_IQ_RX_IVT);
//    sx1276_write(0x33, 0x67);
//    sx1276_rmw(REG_LR_INVERTIQ, RFLR_INVERTIQ_CRX_MASK, RFLR_INVERTIQ_CRX_ON);

    uart_putstring("REG: (Initialized)\r\n");
	dump_reg();

	rx_timestamp = millis();
    cad_timestamp = millis();

    sx1276_set_symbol_timeout(64);
    sx1276_cad();

    uart_putstring("REG: (CAD)\r\n");
	dump_reg();
    rx_timestamp = millis();
    cad_timestamp = millis();

    printf("Normal RX mode\r\n");
    //printf("Inverted RX mode\r\n");

    sx1276_set_symbol_timeout(5);

	while(1){
		if( uart_readable() ){
			//uart_putchar(uart_getchar());
            switch(uart_getchar()){
            case 'g':
                dump_reg();
                break;
            case 'a':
                printf("CAD1 COUNT: %ld\r\n", cad1_count);
                printf("CAD2 COUNT: %ld\r\n", cad2_count);
                printf("CAD DETECTED: %ld\r\n", cad_detected_count);
                printf("CAD RX COUNT: %ld\r\n", cad_rx_count);
                break;
            case 'c':
                printf("Restart Counting\r\n");
                cad1_count = 0;
                cad2_count = 0;
                cad_detected_count = 0;
                cad_rx_count = 0;
                sys_sta = SYS_STA_IDLE;
                break;
            case 'r':

                break;
            case 't':

                break;
            case 'w':
                /* cad_pnr adjust*/
                cad_pnr++;
                sx1276_set_cad_thresh(SX1276_CAD_THRESH_PNR, cad_pnr);
                printf("CAD_PNR++ %02X %d\r\n", cad_pnr, cad_pnr);
                break;
            case 's':
                /* cad_pnr adjust*/
                cad_pnr--;
                sx1276_set_cad_thresh(SX1276_CAD_THRESH_PNR, cad_pnr);
                printf("CAD_PNR++ %02X %d\r\n", cad_pnr, cad_pnr);
                break;
            case 'i':
                /* cad_pnr adjust*/
                cad_peak_min++;
                sx1276_set_cad_thresh(SX1276_CAD_THRESH_PEAK_MIN, cad_peak_min);
                printf("CAD_PEAK_MIN++ %02X %d\r\n", cad_peak_min, cad_peak_min);
                break;
            case 'k':
                /* cad_peak_min adjust*/
                cad_peak_min--;
                sx1276_set_cad_thresh(SX1276_CAD_THRESH_PEAK_MIN, cad_peak_min);
                printf("CAD_PEAK-- %02X %d\r\n", cad_peak_min, cad_peak_min);
                break;
            case 'n':
                printf("Normal IQ\r\n");
                sx1276_set_mode(RFLR_OPMODE_SLEEP);
                sys_sta = SYS_STA_IDLE;
                sx1276_set_iq(SX1276_IQ_RXTX_NML);
                break;
            case 'v':
                printf("Inverted IQ\r\n");
                sx1276_set_mode(RFLR_OPMODE_SLEEP);
                sys_sta = SYS_STA_IDLE;
                sx1276_set_iq(SX1276_IQ_RX_IVT);
                break;
            case 'z':
                sx1276_set_mode(RFLR_OPMODE_SLEEP);
                break;
            }
		}

        switch(sys_sta){
        case SYS_STA_IDLE:
            if( millis() - cad_timestamp > CAD_INTERVAL){
                sx1276_cad();
                cad1_count++;
                cad_timestamp = millis();
                sys_sta = SYS_STA_CAD;
            }
            break;
        case SYS_STA_CAD:
            if(cad_done_flag){
                cad_done_flag = 0;
                sys_sta = SYS_STA_IDLE;
                //printf("CAD Done RSSI:%d\r\n", cad_rssi);
            }
            if(cad_detected_flag){
                sx1276_cad();
                cad_detected_flag = 0;
                cad2_count++;
                sys_sta = SYS_STA_CAD2;
                //printf("CAD1 RSSI%d\r\n", cad_rssi);
            }
            break;
        case SYS_STA_CAD2:
            if(cad_done_flag){
                cad_done_flag = 0;
                sys_sta = SYS_STA_IDLE;
                //printf("CAD Done RSSI:%d\r\n", cad_rssi);
            }
            if(cad_detected_flag){
                //sx1276_set_symbol_timeout(5);
                //sx1276_receive(0);
                cad_detected_flag = 0;
                cad_detected_count++;
                sys_sta = SYS_STA_RX;
                rx_timestamp = millis();
                //printf("CRSSI %d\r\n", rx_pkt->cad_rssi);
            }
            break;
        case SYS_STA_RX:

            cad_timestamp = millis();
            sx1276_cad();
            cad1_count++;
            sys_sta = SYS_STA_CAD;

            break;

            if( millis() - rx_timestamp > 3000){
                rx_timestamp = millis();

                //cad_timestamp = millis();
                //sx1276_cad();
                //sys_sta = SYS_STA_CAD;
            }
            if(rx_error_flag){
                rx_error_flag = 0;
                printf("RX ERROR!!!\r\n");
                led_blink(LED_BLUE, 100);

                cad_timestamp = millis();
                sx1276_cad();
                cad1_count++;
                sys_sta = SYS_STA_CAD;
            }
            if(rx_timeout_flag){

                cad_timestamp = millis();
                sx1276_cad();
                cad1_count++;
                sys_sta = SYS_STA_CAD;

                rx_timeout_flag = 0;
                led_blink(LED_BLUE, 100);
                uart_putstring("RX Timeout!!!\r\n");
                sys_sta = SYS_STA_IDLE;
            }

            if(rx_done_flag){
                rx_done_flag = 0;
                led_blink(LED_GREEN, 100);
                cad_rx_count++;
#if 0
                rx_check_sum = check_sum(rx_pkt->buf, rx_pkt->len-1);
                if(rx_check_sum == rx_pkt->buf[rx_pkt->len-1]){
                    rx_count = rx_pkt->buf[0];
                    rx_count <<= 8;
                    rx_count += rx_pkt->buf[1];
                    rx_count <<= 8;
                    rx_count += rx_pkt->buf[2];
                    rx_count <<= 8;
                    rx_count += rx_pkt->buf[3];
                    if(rx_count == (rx_count_bak + 1) ){
                        rx_pkt_cnt++;
                        uart_putstring("OK ");
                        printf("LEN:%d RSSI:%d SNR:%d\r\n", rx_pkt->len, rx_pkt->rssi, rx_pkt->snr);
                    }else if( rx_count <= rx_count_bak ){
                        uart_putstring("Restart counting.\r\n");
                        rx_pkt_lost_cnt = 0;
                        rx_pkt_cnt = 0;
                    }else{
                        rx_pkt_lost_cnt += (rx_count - rx_count_bak - 1);
                        if(rx_count_bak == 0){
                            uart_putstring("Restart counting.\r\n");
                            rx_pkt_lost_cnt = 0;
                            rx_pkt_cnt = 0;
                        }else{
                            for(cnt = rx_count_bak+1; cnt < rx_count; cnt++){
                                printf("PACKET LOST 0x%lX!!!\r\n", cnt);
                            }
                            printf("OK LEN:%d RSSI:%d SNR:%d\r\n", rx_pkt->len, rx_pkt->rssi, rx_pkt->snr);
                        }
                    }
                    rx_count_bak = rx_count;
                }else{
                    uart_putstring("USER CHECKSUM ERROR!!!\r\n");
                    dump_buf(rx_pkt->buf, rx_pkt->len, rx_pkt->rssi, rx_pkt->snr);
                }
#else
                dump_buf(rx_pkt->buf, rx_pkt->len, rx_pkt->rssi, rx_pkt->snr);
#endif
                sys_sta = SYS_STA_IDLE;
            }
            break;
        }

        led_evt();
	}
}