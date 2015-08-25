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

/** HuangShan Demo Board Red LED -- PB0 */
#define HAL_RLED_BIT			(1<<0)
/** Output push pull */
#define HAL_RLED_OUTPUT()		GPIOB->CR2 &= ~HAL_RLED_BIT; \
								GPIOB->DDR |= HAL_RLED_BIT; \
								GPIOB->CR1 |= HAL_RLED_BIT; \
								GPIOB->CR2 |= HAL_RLED_BIT;
/** Input floating */
#define HAL_RLED_INPUT()		GPIOD->CR2 &= ~HAL_RLED_BIT; \
								GPIOD->DDR &= ~HAL_RLED_BIT; \
								GPIOD->CR1 &= ~HAL_RLED_BIT;
#define HAL_RLED_H()			GPIOB->ODR |= HAL_RLED_BIT;
#define HAL_RLED_L()			GPIOB->ODR &= ~HAL_RLED_BIT;
#define HAL_RLED_V()			GPIOB->ODR ^= HAL_RLED_BIT;
#define HAL_RLED_ON()			HAL_RLED_L()
#define HAL_RLED_OFF()			HAL_RLED_H()

/** HuangShan Demo Board Green LED -- PB1 */
#define HAL_GLED_BIT			(1<<1)
/** Output push pull */
#define HAL_GLED_OUTPUT()		GPIOB->CR2 &= ~HAL_GLED_BIT; \
								GPIOB->DDR |= HAL_GLED_BIT; \
								GPIOB->CR1 |= HAL_GLED_BIT; \
								GPIOB->CR2 |= HAL_GLED_BIT;
/** Input floating */
#define HAL_GLED_INPUT()		GPIOD->CR2 &= ~HAL_GLED_BIT; \
								GPIOD->DDR &= ~HAL_GLED_BIT; \
								GPIOD->CR1 &= ~HAL_GLED_BIT;
#define HAL_GLED_H()			GPIOB->ODR |= HAL_GLED_BIT;
#define HAL_GLED_L()			GPIOB->ODR &= ~HAL_GLED_BIT;
#define HAL_GLED_V()			GPIOB->ODR ^= HAL_GLED_BIT;
#define HAL_GLED_ON()			HAL_GLED_L()
#define HAL_GLED_OFF()			HAL_GLED_H()

/** HuangShan Demo Board Blue LED -- PB2*/
#define HAL_BLED_BIT			(1<<2)
/** Output push pull */
#define HAL_BLED_OUTPUT()		GPIOB->CR2 &= ~HAL_BLED_BIT; \
								GPIOB->DDR |= HAL_BLED_BIT; \
								GPIOB->CR1 |= HAL_BLED_BIT; \
								GPIOB->CR2 |= HAL_BLED_BIT;
/** Input floating */
#define HAL_BLED_INPUT()		GPIOD->CR2 &= ~HAL_BLED_BIT; \
								GPIOD->DDR &= ~HAL_BLED_BIT; \
								GPIOD->CR1 &= ~HAL_BLED_BIT;
#define HAL_BLED_H()			GPIOB->ODR |= HAL_BLED_BIT;
#define HAL_BLED_L()			GPIOB->ODR &= ~HAL_BLED_BIT;
#define HAL_BLED_V()			GPIOB->ODR ^= HAL_BLED_BIT;
#define HAL_BLED_ON()			HAL_BLED_L()
#define HAL_BLED_OFF()			HAL_BLED_H()


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
		HAL_RLED_ON();
		break;
	  case LED1:
		HAL_GLED_ON();
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
		HAL_RLED_OFF();
		break;
	  case LED1:
		HAL_GLED_OFF();
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
