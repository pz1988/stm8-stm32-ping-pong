/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: UART HAL driver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#ifndef __HAL_UART_H
#define __HAL_UART_H

#include "hardware.h"

typedef enum{
	NONE,
  	EVEN,
	ODD,
}uart_parity_t;

typedef struct{
	uint32_t baud;
	uint8_t word_length;
	uart_parity_t parity;
	uint8_t stop_bits;
}uart_config_t;

//for debugger
/*typedef enum{
	UART_TX_RUNNING,
	UART_RX_RUNNING,
}uart_tx_rx_flag;*/
/*debugger end*/


typedef uint8_t (*uart_tx_call_back_t) (void);
typedef void (*uart_rx_call_back_t) (uint8_t);

void hal_uart_init(uart_config_t *config, uart_tx_call_back_t tx_hanlder,
				   uart_rx_call_back_t rx_handler);
uint8_t hal_uart_tx_is_empty(void);
void hal_uart_write_tx_reg(uint8_t val);


#endif
