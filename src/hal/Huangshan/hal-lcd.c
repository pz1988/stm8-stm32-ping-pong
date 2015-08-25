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

#include "hal-lcd.h"
#include "sys-tick.h"

/** HuangShan Demo Board LCD CS -- PD1 */
#define HAL_LCD_CS_BIT			(1<<1)
/** Output push pull */
#define HAL_LCD_CS_OUTPUT()		GPIOD->CR2 &= ~HAL_LCD_CS_BIT; \
								GPIOD->DDR |= HAL_LCD_CS_BIT; \
								GPIOD->CR1 |= HAL_LCD_CS_BIT; \
								GPIOD->CR2 |= HAL_LCD_CS_BIT;
/** Input floating */
#define HAL_LCD_CS_INPUT()		GPIOD->CR2 &= ~HAL_LCD_CS_BIT; \
								GPIOD->DDR &= ~HAL_LCD_CS_BIT; \
								GPIOD->CR1 &= ~HAL_LCD_CS_BIT;
#define HAL_LCD_CS_H()			GPIOD->ODR |= HAL_LCD_CS_BIT;
#define HAL_LCD_CS_L()			GPIOD->ODR &= ~HAL_LCD_CS_BIT;
#define HAL_LCD_CS_V()			GPIOD->ODR ^= HAL_LCD_CS_BIT;

/** HuangShan Demo Board LCD SID -- PD2 */
#define HAL_LCD_SID_BIT			(1<<2)
/** Output push pull */
#define HAL_LCD_SID_OUTPUT()    GPIOD->CR2 &= ~HAL_LCD_SID_BIT; \
                                GPIOD->DDR |= HAL_LCD_SID_BIT; \
                                GPIOD->CR1 |= HAL_LCD_SID_BIT; \
                                GPIOD->CR2 |= HAL_LCD_SID_BIT;
/** Input floating */
#define HAL_LCD_SID_INPUT()     GPIOD->CR2 &= ~HAL_LCD_SID_BIT; \
                                GPIOD->DDR &= ~HAL_LCD_SID_BIT; \
                                GPIOD->CR1 &= ~HAL_LCD_SID_BIT;
#define HAL_LCD_SID_H()         GPIOD->ODR |= HAL_LCD_SID_BIT;
#define HAL_LCD_SID_L()         GPIOD->ODR &= ~HAL_LCD_SID_BIT;
#define HAL_LCD_SID_V()         GPIOD->ODR ^= HAL_LCD_SID_BIT;

/** HuangShan Demo Board LCD CLK -- PD3 */
#define HAL_LCD_CLK_BIT         (1<<3)
/** Output push pull */
#define HAL_LCD_CLK_OUTPUT()    GPIOD->CR2 &= ~HAL_LCD_CLK_BIT; \
                                GPIOD->DDR |= HAL_LCD_CLK_BIT; \
                                GPIOD->CR1 |= HAL_LCD_CLK_BIT; \
                                GPIOD->CR2 |= HAL_LCD_CLK_BIT;
/** Input floating */
#define HAL_LCD_CLK_INPUT()     GPIOD->CR2 &= ~HAL_LCD_CLK_BIT; \
                                GPIOD->DDR &= ~HAL_LCD_CLK_BIT; \
                                GPIOD->CR1 &= ~HAL_LCD_CLK_BIT;
#define HAL_LCD_CLK_H()         GPIOD->ODR |= HAL_LCD_CLK_BIT;
#define HAL_LCD_CLK_L()         GPIOD->ODR &= ~HAL_LCD_CLK_BIT;
#define HAL_LCD_CLK_V()         GPIOD->ODR ^= HAL_LCD_CLK_BIT;

/** HuangShan Demo Board LCD RST -- PA3 */
#define HAL_LCD_RST_BIT         (1<<3)
/** Output push pull */
#define HAL_LCD_RST_OUTPUT()    GPIOA->CR2 &= ~HAL_LCD_RST_BIT; \
                                GPIOA->DDR |= HAL_LCD_RST_BIT; \
                                GPIOA->CR1 |= HAL_LCD_RST_BIT; \
                                GPIOA->CR2 |= HAL_LCD_RST_BIT;
/** Input floating */
#define HAL_LCD_RST_INPUT()     GPIOA->CR2 &= ~HAL_LCD_RST_BIT; \
                                GPIOA->DDR &= ~HAL_LCD_RST_BIT; \
                                GPIOA->CR1 &= ~HAL_LCD_RST_BIT;
#define HAL_LCD_RST_H()         GPIOA->ODR |= HAL_LCD_RST_BIT;
#define HAL_LCD_RST_L()         GPIOA->ODR &= ~HAL_LCD_RST_BIT;
#define HAL_LCD_RST_V()         GPIOA->ODR ^= HAL_LCD_RST_BIT;

#if 1
//#define HAL_LCD_DELAY()         {nop();nop();nop();nop();nop();nop();		\
//                                 nop();nop();nop();nop();nop();nop();}
#define HAL_LCD_DELAY()         {nop();nop();nop();nop();nop();nop();}
#else
#define HAL_LCD_DELAY()
#endif

void hal_lcd_init(void)
{
    HAL_LCD_CS_OUTPUT();
    HAL_LCD_SID_OUTPUT();
    HAL_LCD_CLK_OUTPUT();
    HAL_LCD_RST_OUTPUT();
    HAL_LCD_BL_OUTPUT();
    HAL_LCD_PWR_OUTPUT();

    /** Turn on back light */
    HAL_LCD_BL_ON();
    HAL_LCD_PWR_ON();
    delay_ms(100);

    HAL_LCD_CS_H();
    HAL_LCD_SID_H();
    HAL_LCD_CLK_H();

    HAL_LCD_CS_L();
    HAL_LCD_SID_L();
    HAL_LCD_CLK_L();

    /** reset LCD controler */
    HAL_LCD_RST_L();
    delay_ms(1);
    HAL_LCD_RST_H();
    delay_ms(5);
}

void hal_lcd_write(uint8_t dt)
{
    int i;
    for(i=0; i<8; i++){
        if(dt&0x80){
            HAL_LCD_SID_H();
        }else{
            HAL_LCD_SID_L();
        }
        HAL_LCD_CLK_H();
        HAL_LCD_DELAY();
        dt <<= 1;
        HAL_LCD_CLK_L();
        //HAL_LCD_DELAY();
    }
}


