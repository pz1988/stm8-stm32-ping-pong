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

#include "hal-led.h"
#include "board.h"

/** HuangShan Demo Board Red LED -- LOE12 */
#define HAL_RLED_BIT			//(1<<12)
/** Output push pull */
#define HAL_RLED_OUTPUT()		GpioInit( &Led1, LED_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 )


/** Input floating */
#define HAL_RLED_INPUT()        GpioInit( &Led1, LED_1, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 )


#define HAL_RLED_H()		    GpioWrite( &Led1, 1 )
#define HAL_RLED_L()			GpioWrite( &Led1, 0 )
#define HAL_RLED_V()			GPIO_ToggleBits( Led1.port, Led1.pinIndex );
#define HAL_RLED_ON()			HAL_RLED_L()
#define HAL_RLED_OFF()			HAL_RLED_H()


/** HuangShan Demo Board Green LED --IOE13 */
#define HAL_GLED_BIT			//(1<<13)
/** Output push pull */
#define HAL_GLED_OUTPUT()		GpioInit( &Led2, LED_2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 )


/** Input floating */
#define HAL_GLED_INPUT()		GpioInit( &Led2, LED_2, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 )

#define HAL_GLED_H()			GpioWrite( &Led2, 1 )
#define HAL_GLED_L()			GpioWrite( &Led2, 0 )
#define HAL_GLED_V()			GPIO_ToggleBits( Led2.port, Led2.pinIndex )
#define HAL_GLED_ON()			HAL_GLED_L()
#define HAL_GLED_OFF()			HAL_GLED_H()

/** HuangShan Demo Board Blue LED -- IOE15*/
#define HAL_BLED_BIT			//(1<<15)
/** Output push pull */
#define HAL_BLED_OUTPUT()		GpioInit( &Led4, LED_4, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 )


/** Input floating */
#define HAL_BLED_INPUT()		GpioInit( &Led4, LED_4, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 )


#define HAL_BLED_H()			GpioWrite( &Led4, 1 )
#define HAL_BLED_L()			GpioWrite( &Led4, 0 )
#define HAL_BLED_V()			GPIO_ToggleBits( Led4.port, Led4.pinIndex )
#define HAL_BLED_ON()			HAL_BLED_L()
#define HAL_BLED_OFF()			HAL_BLED_H()

//for debugger
extern Gpio_t ext_red;
extern Gpio_t ext_green;
//end debugger



void hal_led_init(void)
{
	/** LED initial */
	HAL_RLED_OUTPUT();
	HAL_GLED_OUTPUT();
	HAL_BLED_OUTPUT();

	/** ALL LED off */
	HAL_RLED_OFF();
	HAL_GLED_OFF();
	HAL_BLED_OFF();
}

void hal_led_on(led_index_t led_index)
{
	switch(led_index){
	  case LED0:
	  	//for debugger
	  	//GpioWrite( &ext_red, 0 );
		HAL_RLED_ON();
		//end debugger
		break;
	  case LED1:
	  	//for debugger
	  	//GpioWrite( &ext_green, 0 );
		HAL_GLED_ON();
		//end debugger
		break;
	  case LED2:
		HAL_BLED_ON();
		break;
	}
}

void hal_led_off(led_index_t led_index)
{
	switch(led_index){
	  case LED0:
	  	//for debugger
	  	//GpioWrite( &ext_red, 1 );
		HAL_RLED_OFF();
		//end debugger
		break;
	  case LED1:
	  	//for debugger
	  	//GpioWrite( &ext_green, 1 );
		HAL_GLED_ON();
		//end debugger
		break;
	  case LED2:
		HAL_BLED_OFF();
		break;
	}
}

void hal_led_toggle(led_index_t led_index)
{
	switch(led_index){
	  case LED0:
		HAL_RLED_V();
		break;
	  case LED1:
		HAL_GLED_V();
		break;
	  case LED2:
		HAL_BLED_V();
		break;
	}
}
