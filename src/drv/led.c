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

#include "led.h"
#include "sys-tick.h"

uint32_t led_timestamp_p[HAL_LED_MAX_NUMS];
uint8_t led_sta_p[HAL_LED_MAX_NUMS];
uint32_t led_blink_time_p[HAL_LED_MAX_NUMS];

void led_init(void)
{
    int i;

    hal_led_init();
    for(i=0; i<HAL_LED_MAX_NUMS; i++){
        led_sta_p[i] = 0;
    }
}

void led_on(led_index_t led_index)
{
    hal_led_on(led_index);
}

void led_off(led_index_t led_index)
{
    hal_led_off(led_index);
}

void led_toggle(led_index_t led_index)
{
    hal_led_toggle(led_index);
}

void led_blink(led_index_t led_index, uint16_t light_on_time)
{
    if(led_index >= HAL_LED_MAX_NUMS){
        return;
    }
    led_on(led_index);
    led_sta_p[led_index] = 1;
    led_timestamp_p[led_index] = millis();
    led_blink_time_p[led_index] = light_on_time;
}

void led_evt(void)
{
    int i;

    for(i=0; i<HAL_LED_MAX_NUMS; i++){
        if(led_sta_p[i] == 0){
            continue;
        }
        if( millis() - led_timestamp_p[i] > led_blink_time_p[i]){
			led_timestamp_p[i] = millis();
            led_sta_p[i] = 0;
			led_off((led_index_t)i);
		}
    }
}
