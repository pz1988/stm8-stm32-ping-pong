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
#include "app.h"

//for debugger
uint8_t read_reg = 0x08;
uint8_t pre_read_reg = 0x00;
Gpio_t ext_red;
Gpio_t ext_green;
//end debugger

int main()
{
  	uart_config_t uart_conf;

	DISABLE_IRQ();

	hal_clk_init();
	sys_tick_init();
	I2cInit( &I2c, I2C_SCL, I2C_SDA );

	uart_conf.baud = 9600;
	uart_conf.word_length = 8;
	uart_conf.parity = NONE;
	uart_conf.stop_bits = 1;
	uart_init(&uart_conf);

	ENABLE_IRQ();

    app_init();
	
	DISABLE_IRQ();
	 led_init();
	 //for debugger
   read_reg = GpioRead( &Led1 );
	 
	 led_on(LED0);
	 read_reg = GpioRead( &Led1 );
//		   //for debugger
//	   GpioInit( &ext_red, TEST_POINT2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//	   GpioInit( &ext_green, TEST_POINT3, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//	 
//	   GpioWrite( &ext_red, 1 );
//	 GpioWrite( &ext_green, 1 ); 
//    //end debugger
//    
	 
	while(1){
        /** polling all events */
        app_evt();
        led_evt();
	}
}
