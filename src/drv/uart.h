/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: high level uart driver/API

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#ifndef __UART_H
#define __UART_H

#include "hal-uart.h"

#define UART_RX_BUF_LEN 				(220)
#define UART_TX_BUF_LEN 				(220)

void uart_init(uart_config_t *config);
int uart_readable();
//int uart_writable();
void uart_putchar(uint8_t c);
void uart_putstring(char *str);
void uart_putbuf(uint8_t *buf, uint8_t len);
void uart_puthex(uint8_t val);
void uart_putbuf_hex(uint8_t *buf, uint8_t len);
int16_t uart_getchar(void);

#endif

