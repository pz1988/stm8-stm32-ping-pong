/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: LED low-level driver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#ifndef __HAL_LED_H
#define __HAL_LED_H

#include "hardware.h"

#define HAL_LED_MAX_NUMS        (3)

typedef enum{
	LED0 = 0,
	LED1,
	LED2,
	LED3,
	LED4,
	LED5,
}led_index_t;

void hal_led_init(void);
void hal_led_on(led_index_t led_index);
void hal_led_off(led_index_t led_index);
void hal_led_toggle(led_index_t led_index);

#endif
