/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: LCD ST7920 driver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#ifndef __LCD_H
#define __LCD_H

#include "hardware.h"

void lcd_init(void);
void lcd_write_cmd(uint8_t cmd);
void lcd_write_data(uint8_t dt);
void lcd_putstring( uint8_t row, uint8_t col, uint8_t *str );
void lcd_putbuf( uint8_t row, uint8_t col, uint8_t *str, uint8_t len );

#endif
