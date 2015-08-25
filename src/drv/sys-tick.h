/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: system tick to drive delay and supply application software timer.

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#ifndef __SYS_TICK_H
#define __SYS_TICK_H

#include "hardware.h"

#define TICK_MAX_VALUE			(0xFFFF)

typedef enum{
	OFF=0,
	ON=1,
}sys_tick_sta_type;

void sys_tick_init(void);
uint8_t sys_tick_set(uint8_t num, sys_tick_sta_type status);
uint8_t sys_tick_clear(uint8_t num);
uint16_t sys_tick_get(uint8_t num);
uint8_t sys_tick_apply(void);
uint8_t	sys_tick_set_value(uint8_t num, uint16_t value);
void delay_ms(uint32_t ms);     // accuracy: 1ms
void delay_us(uint32_t us);     // accuracy: 100us
uint32_t millis( void );

#endif /** __SYS_TICK_H */

