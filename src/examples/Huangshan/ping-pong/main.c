/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: PingPong main function

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#include <stdio.h>

#include "hal-sys-clk.h"
#include "hardware.h"
#include "sys-tick.h"
#include "sx1276.h"
#include "uart.h"
#include "led.h"
#include "lcd.h"
#include "key.h"
#include "app.h"

void main()
{
  	uart_config_t uart_conf;

	DISABLE_IRQ();

	led_init();
	hal_clk_init();
	sys_tick_init();

	uart_conf.baud = 9600;
	uart_conf.word_length = 8;
	uart_conf.parity = NONE;
	uart_conf.stop_bits = 1;
	uart_init(&uart_conf);

	ENABLE_IRQ();

    lcd_init();
    key_init();
    app_init();

	while(1){
        /** read key value */
        switch(key_get()){
        case KEY2:
            break;
        case KEY3:
            app_update_lcd();
            break;
        }

        /** polling all events */
        app_evt();
        led_evt();
	}
}
