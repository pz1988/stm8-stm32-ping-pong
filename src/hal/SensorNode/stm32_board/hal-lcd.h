/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: LCD128x64 [ST7920] driver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#ifndef __HAL_LCD_H
#define __HAL_LCD_H

#include "hardware.h"

/** HuangShan Demo Board LCD CS -- PD1 */
#define HAL_LCD_CS_BIT			//(1<<1)
/** Output push pull */
#define HAL_LCD_CS_OUTPUT()		/*GPIOD->CR2 &= ~HAL_LCD_CS_BIT; \
								GPIOD->DDR |= HAL_LCD_CS_BIT; \
								GPIOD->CR1 |= HAL_LCD_CS_BIT; \
								GPIOD->CR2 |= HAL_LCD_CS_BIT;*/
/** Input floating */
#define HAL_LCD_CS_INPUT()		/*GPIOD->CR2 &= ~HAL_LCD_CS_BIT; \
								GPIOD->DDR &= ~HAL_LCD_CS_BIT; \
								GPIOD->CR1 &= ~HAL_LCD_CS_BIT;*/
#define HAL_LCD_CS_H()			//GPIOD->ODR |= HAL_LCD_CS_BIT;
#define HAL_LCD_CS_L()			//GPIOD->ODR &= ~HAL_LCD_CS_BIT;
#define HAL_LCD_CS_V()			//GPIOD->ODR ^= HAL_LCD_CS_BIT;

/** HuangShan Demo Board LCD BL -- PA5 */
#define HAL_LCD_BL_BIT          //(1<<5)
/** Output push pull */
#define HAL_LCD_BL_OUTPUT()     /*GPIOA->CR2 &= ~HAL_LCD_BL_BIT; \
                                				GPIOA->DDR |= HAL_LCD_BL_BIT; \
                                				GPIOA->CR1 |= HAL_LCD_BL_BIT; \
                                				GPIOA->CR2 |= HAL_LCD_BL_BIT;*/
/** Input floating */
#define HAL_LCD_BL_INPUT()      /*GPIOA->CR2 &= ~HAL_LCD_BL_BIT; \
                                				GPIOA->DDR &= ~HAL_LCD_BL_BIT; \
                                				GPIOA->CR1 &= ~HAL_LCD_BL_BIT;*/
#define HAL_LCD_BL_H()          //GPIOA->ODR |= HAL_LCD_BL_BIT;
#define HAL_LCD_BL_L()          //GPIOA->ODR &= ~HAL_LCD_BL_BIT;
#define HAL_LCD_BL_V()          //GPIOA->ODR ^= HAL_LCD_BL_BIT;
#define HAL_LCD_BL_ON()         HAL_LCD_BL_H()
#define HAL_LCD_BL_OFF()        HAL_LCD_BL_L()

/** HuangShan Demo Board LCD PWR -- PC5 */
#define HAL_LCD_PWR_BIT          //(1<<5)
/** Output push pull */
#define HAL_LCD_PWR_OUTPUT()    /*GPIOC->CR2 &= ~HAL_LCD_PWR_BIT; \
                                				GPIOC->DDR |= HAL_LCD_PWR_BIT; \
                                				GPIOC->CR1 |= HAL_LCD_PWR_BIT; \
                                				GPIOC->CR2 |= HAL_LCD_PWR_BIT;*/
/** Input floating */
#define HAL_LCD_PWR_INPUT()     /*GPIOC->CR2 &= ~HAL_LCD_PWR_BIT; \
                                				GPIOC->DDR &= ~HAL_LCD_PWR_BIT; \
                               				 GPIOC->CR1 &= ~HAL_LCD_PWR_BIT;*/
#define HAL_LCD_PWR_H()         //GPIOC->ODR |= HAL_LCD_PWR_BIT;
#define HAL_LCD_PWR_L()         //GPIOC->ODR &= ~HAL_LCD_PWR_BIT;
#define HAL_LCD_PWR_V()         //GPIOC->ODR ^= HAL_LCD_PWR_BIT;
#define HAL_LCD_PWR_ON()        HAL_LCD_PWR_H()
#define HAL_LCD_PWR_OFF()       HAL_LCD_PWR_L()


void hal_lcd_init(void);
void hal_lcd_write(uint8_t dt);

#endif
