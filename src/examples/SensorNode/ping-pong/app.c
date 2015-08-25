/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: Ping-Pong application

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/
#include <stdio.h>
#include <string.h>
#include "hal-sys-clk.h"
#include "sys-tick.h"
#include "uart.h"
#include "sx1276.h"
#include "led.h"
#include "lcd.h"
#include "app.h"

/** Maximum packet length */
//#define APP_TX_PKT_LEN_MAX                      (255)
//#define APP_TX_PKT_LEN_MAX                      (63)

//#define APP_USING_SF12

#ifdef APP_USING_SF12
#define APP_TX_PKT_LEN_MAX                      (6)  //4<x<=12, for ping and pong use 4 tytes, at least 4 that is not including rx_count
#else
#define APP_TX_PKT_LEN_MAX                      (255)
#endif

/** Delay time before transmit packet, give receiver time to initial */
//#define APP_TX_DELAY                            (1500)
#define APP_TX_DELAY                            (10)

/** Connecting/IDLE mode timeout value to wait ping packet */
//#define APP_IDLE_WAIT_PING_TIMEOUT              (15000)/*(5000) for debugger*/

/** Slave wait ping packet timeout, if not receive in this timeout,
    Slave enter IDLE mode*/
#define APP_SLAVE_WAIT_PING_TIMEOUT             (30000)

/** Master wait pong packet timeout, if not receive in this timeout,
    master will send next packet, this means one pong packet is lost */
#define APP_MASTER_WAIT_PONG_TIMEOUT            (7000)

/** Maximum time to wait TX_DONE interrupt,
    avoid application wait TX_DONE too long time */
//#define APP_IDLE_TX_TIMEOUT                     (4000)

/** LED color defination */
#define LED_GREEN                               LED1
#define LED_RED                                 LED0
#define LED_BLUE                                LED2

/** RF configuration defination */
#define APP_TEST_FREQ                           (470000000)

/** Comment to disable UART output */
//#define ENABLE_UART

/** SX1276 global RF configuration */
sx1276_config_t sx1276_config;

/** Initial application status (connecting mode) */
app_sta_t app_sta = APP_STA_WAIT_PINGPONG;

/** application timestamp */
uint32_t app_ts;

/** TX packet counter and flags */
uint32_t tx_count = 0;
uint8_t tx_buf[256];
uint8_t tx_len;
uint8_t tx_done_flag = 0;

/** RX flags */
sx1276_rx_pkt_t *rx_pkt;
uint8_t rx_done_flag = 0;
uint8_t rx_error_flag = 0;
uint8_t rx_timeout_flag = 0;

/* RX packet counter */
uint32_t rx_count_bak = 0;
uint32_t rx_count = 0;
uint8_t rx_check_sum;
uint32_t rx_pkt_lost_cnt = 0;
uint32_t rx_pkt_cnt = 0;

uint8_t master_timeout_cnt = 0;

#define APP_MASTER_SELECT 1
#define APP_SLAVE_SELECT  0

uint8_t master_slave_select = APP_SLAVE_SELECT;
uint8_t master_rx_once_flag = 1;

void app_sx1276_event(sx1276_event_t state, void *ptr)
{
	switch(state){
	  case SX1276_TX_DONE:
        tx_done_flag = 1;
	  	break;
	  case SX1276_RX_DONE:
        rx_done_flag = 1;
        rx_pkt = ptr;
		break;
	  case SX1276_RX_TIMEOUT:
        rx_timeout_flag = 1;
	  	break;
      case SX1276_RX_ERROR:
        rx_error_flag = 1;
        rx_pkt = ptr;
	  	break;
	}
}

uint8_t app_check_sum(uint8_t *buf, int len)
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

/* type: 0-ping, 1-pong */
void app_tx_pkt(app_pkt_t app_pkt, uint32_t count)
{
    int i;
    /** FIFO Test */
    for(i=4; i<tx_len-1; i++){
        tx_buf[i] = i;
    }
    tx_buf[0]='p';
    tx_buf[1]='i';
    if(app_pkt == APP_PKT_PONG){
        tx_buf[1]='o';
    }
    tx_buf[2]='n';
    tx_buf[3]='g';

    tx_buf[4] = (count>>24)&0xFF;
    tx_buf[5] = (count>>16)&0xFF;
    tx_buf[6] = (count>>8)&0xFF;
    tx_buf[7] = (count>>0)&0xFF;

    tx_buf[tx_len-1] = app_check_sum(tx_buf, tx_len-1);
    sx1276_send(tx_buf, tx_len, 0);
}

void app_init(void)
{
	sx1276_init(LORA, app_sx1276_event);

	#ifdef APP_USING_SF12
	sx1276_config.spread_factor = SX1276_SF12;
	sx1276_config.tx_preamble_len = 8;
	sx1276_config.rx_preamble_len = 8;
    #else
	sx1276_config.spread_factor = SX1276_SF9;
	sx1276_config.tx_preamble_len = 15;
	sx1276_config.rx_preamble_len = 15;
	#endif
	
	sx1276_config.frequency = APP_TEST_FREQ;
	sx1276_config.bandwidth = SX1276_BW_125K;
	sx1276_config.coding_rate = SX1276_CR1;
	sx1276_config.crc_mode = SX1276_CRC_ON;
	sx1276_config.header_mode = SX1276_HEADER_ENABLE;
	sx1276_config.payload_len = 0;		// Set
	sx1276_config.tx_power = 20;
	sx1276_set_config(&sx1276_config);

    if( master_slave_select == APP_SLAVE_SELECT )
    {
         sx1276_set_mode(RFLR_OPMODE_STANDBY);
         sx1276_receive(0);
        
    }
	
    app_ts = millis();
    app_sta = APP_STA_WAIT_PINGPONG;
	
    rx_done_flag = 0;
    rx_error_flag = 0;
    rx_timeout_flag = 0;
    tx_done_flag = 0;

	tx_len = APP_TX_PKT_LEN_MAX;
}

void app_evt(void)
{
    switch(app_sta){
    case APP_STA_WAIT_PINGPONG:
		if( master_slave_select == APP_SLAVE_SELECT )
		{
			if(rx_done_flag){
            rx_done_flag = 0;
            led_blink(LED_GREEN, 100);

            /* Check received payload, discard if it is none-protocol packet*/
            if( app_check_sum(rx_pkt->buf, rx_pkt->len-1) != rx_pkt->buf[rx_pkt->len-1] ){

                break;
            }

            if( 0 == memcmp(rx_pkt->buf, "ping", 4) )
			{
                rx_count = rx_pkt->buf[4];
                rx_count <<= 8;
                rx_count += rx_pkt->buf[5];
                rx_count <<= 8;
                rx_count += rx_pkt->buf[6];
                rx_count <<= 8;
                rx_count += rx_pkt->buf[7];

                rx_pkt_lost_cnt = 0;
                rx_pkt_cnt = 1;
                rx_count_bak = rx_count;


                app_ts = millis();
				app_sta = APP_STA_SLAVE_SEND_PONG_DELAY;

            }			
		}

        if(rx_error_flag)
		{
            rx_error_flag = 0;
            led_blink(LED_BLUE, 100);
        }

        if(rx_timeout_flag)
		{
            rx_timeout_flag = 0;
            led_blink(LED_BLUE, 100);
        }
			
	}
	else if( master_slave_select == APP_MASTER_SELECT )
	{

        master_rx_once_flag = 1;	
		app_ts = millis();
		app_sta = APP_STA_MASTER_SEND_PING_DELAY;
	}
			
    break;
   
    /* slave mode */
    case APP_STA_SLAVE_SEND_PONG_DELAY:

		if( (millis() - app_ts) > APP_TX_DELAY){
            app_ts = millis();
            app_sta = APP_STA_SLAVE_SEND_PONG;

            tx_count++;
            app_tx_pkt(APP_PKT_PONG, tx_count);
            led_blink(LED_RED, 100);
        }
        break;
		
    case APP_STA_SLAVE_SEND_PONG:
		
        /*if( (millis() - app_ts) > APP_IDLE_TX_TIMEOUT){

            app_ts = millis();
            app_sta = APP_STA_WAIT_PINGPONG;
        }*/

        if(tx_done_flag){
            tx_done_flag = 0;
            app_ts = millis();
            app_sta = APP_STA_SLAVE_WAIT_PING;

			sx1276_set_mode(RFLR_OPMODE_STANDBY);
            sx1276_receive(0);
        }
        break;
    case APP_STA_SLAVE_WAIT_PING:
		
        if( (millis() - app_ts) > APP_SLAVE_WAIT_PING_TIMEOUT){

            app_ts = millis();
            app_sta = APP_STA_WAIT_PINGPONG;

			 sx1276_set_mode(RFLR_OPMODE_STANDBY);
             sx1276_receive(0);
            break;
        }

        if(rx_done_flag){
            rx_done_flag = 0;
            led_blink(LED_GREEN, 100);

            /* Check received payload, discard if it is none-protocol packet*/
            if( app_check_sum(rx_pkt->buf, rx_pkt->len-1) != rx_pkt->buf[rx_pkt->len-1] ){


                break;
            }

            if( 0 == memcmp(rx_pkt->buf, "ping", 4) ){
                rx_count = rx_pkt->buf[4];
                rx_count <<= 8;
                rx_count += rx_pkt->buf[5];
                rx_count <<= 8;
                rx_count += rx_pkt->buf[6];
                rx_count <<= 8;
                rx_count += rx_pkt->buf[7];
                if(rx_count == (rx_count_bak + 1) ){
                    rx_pkt_cnt++;

                }else if( rx_count <= rx_count_bak ){

                    rx_pkt_lost_cnt = 0;
                    rx_pkt_cnt = 0;
                }else{
                    rx_pkt_lost_cnt += (rx_count - rx_count_bak - 1);

                    if(rx_count_bak == 0){

                        rx_pkt_lost_cnt = 0;
                        rx_pkt_cnt = 0;
                    }else{
                        rx_pkt_cnt++;

                    }
                }
                rx_count_bak = rx_count;

                sx1276_set_mode(RFLR_OPMODE_STANDBY);
                app_ts = millis();
                app_sta = APP_STA_SLAVE_SEND_PONG_DELAY;

            }
        }

        if(rx_error_flag){
            rx_error_flag = 0;
            led_blink(LED_BLUE, 100);

        }

        if(rx_timeout_flag){
            rx_timeout_flag = 0;
            led_blink(LED_BLUE, 100);

        }
        break;

    /* master mode */
    case APP_STA_MASTER_SEND_PING_DELAY:
		
        if( (millis() - app_ts) > APP_TX_DELAY){

            app_sta = APP_STA_MASTER_SEND_PING;
            tx_count++;
            app_tx_pkt(APP_PKT_PING, tx_count);
            led_blink(LED_RED, 100);

            app_ts = millis();
        }
        break;
    case APP_STA_MASTER_SEND_PING:
		
        if(tx_done_flag){
            tx_done_flag = 0;

            app_ts = millis();
            app_sta = APP_STA_MASTER_WAIT_PONG;

            sx1276_set_mode(RFLR_OPMODE_STANDBY);
            sx1276_receive(0);

            break;
        }
        break;
    case APP_STA_MASTER_WAIT_PONG:

		if(( millis() - app_ts ) > APP_MASTER_WAIT_PONG_TIMEOUT )
		{

			app_ts = millis();
            app_sta = APP_STA_MASTER_SEND_PING_DELAY;
			break;
		}
		
        if(rx_done_flag){
            rx_done_flag = 0;
            led_blink(LED_GREEN, 100);

            /* Check received payload, discard if it is none-protocol packet*/
            if( app_check_sum(rx_pkt->buf, rx_pkt->len-1) != rx_pkt->buf[rx_pkt->len-1] ){

                break;
            }
			if(master_rx_once_flag == 0)
			{
                master_timeout_cnt = 0;
			}
            if( 0 == memcmp(rx_pkt->buf, "pong", 4) ){
                rx_count = rx_pkt->buf[4];
                rx_count <<= 8;
                rx_count += rx_pkt->buf[5];
                rx_count <<= 8;
                rx_count += rx_pkt->buf[6];
                rx_count <<= 8;
                rx_count += rx_pkt->buf[7];
				
				if( master_rx_once_flag == 1 )
				{
				    master_rx_once_flag = 0;
					
					rx_pkt_lost_cnt = 0;
                    rx_pkt_cnt = 1;
					master_timeout_cnt = 0;
				}
				else if( master_rx_once_flag == 0 )
				{
                    if(rx_count == (rx_count_bak + 1) ){
                        rx_pkt_cnt++;
    
                    }else if( rx_count <= rx_count_bak ){
    
                        rx_pkt_lost_cnt = 0;
                        rx_pkt_cnt = 0;
                    }else{
                        rx_pkt_lost_cnt += (rx_count - rx_count_bak - 1);
                        if(rx_count_bak == 0){
    
                            rx_pkt_lost_cnt = 0;
                            rx_pkt_cnt = 0;
                        }else{
                            rx_pkt_cnt++;
    
                        }

						#if 0
                        //app_update_lcd();
						#endif
                    }
					sx1276_set_mode(RFLR_OPMODE_STANDBY);
				}
                rx_count_bak = rx_count;

                app_ts = millis();
                app_sta = APP_STA_MASTER_SEND_PING_DELAY;
            }
            //break;
        }

        if(rx_error_flag){
            rx_error_flag = 0;
            led_blink(LED_BLUE, 100);

        }

        if(rx_timeout_flag){
            rx_timeout_flag = 0;
            led_blink(LED_BLUE, 100);

        }
     break;
}
}
