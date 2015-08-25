/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: LED driver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#ifndef __LED_H
#define __LED_H

#include "hal-led.h"
#ifdef STM32_SENSERNODE_DELAY
#include <stdint.h>
#endif

void led_init(void);
void led_on(led_index_t led_index);
void led_off(led_index_t led_index);
void led_toggle(led_index_t led_index);
void led_blink(led_index_t led_index, uint16_t light_on_time);
void led_evt(void);

#endif
