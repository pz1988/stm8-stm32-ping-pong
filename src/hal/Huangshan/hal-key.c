/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: key HAL driver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#include "hal-key.h"

/** KEY1 is unavailable for Huangshan demo */
#if 0
/** KE1 -- PA5 */
#define HAL_KEY1_BIT                    (1<<5)

/** Input floating */
#define HAL_KEY1_INPUT()                GPIOA->CR2 &= ~HAL_KEY1_BIT; \
                                        GPIOA->DDR &= ~HAL_KEY1_BIT; \
                                        GPIOA->CR1 |= HAL_KEY1_BIT;
#define HAL_KEY1                        (GPIOA->IDR & HAL_KEY1_BIT)

#else

/** KE1 -- PA5 */
#define HAL_KEY1_BIT                    (1<<5)
/** Input floating */
#define HAL_KEY1_INPUT()
#define HAL_KEY1                        (1)

#endif

/** KE2 -- PA6 */
#define HAL_KEY2_BIT                    (1<<6)
/** Input floating */
#define HAL_KEY2_INPUT()                GPIOA->CR2 &= ~HAL_KEY2_BIT; \
                                        GPIOA->DDR &= ~HAL_KEY2_BIT; \
                                        GPIOA->CR1 |= HAL_KEY2_BIT;
#define HAL_KEY2                        (GPIOA->IDR & HAL_KEY2_BIT)

/** KE3 -- PB3 */
#define HAL_KEY3_BIT                    (1<<3)
/** Input floating */
#define HAL_KEY3_INPUT()                GPIOB->CR2 &= ~HAL_KEY3_BIT; \
                                        GPIOB->DDR &= ~HAL_KEY3_BIT; \
                                        GPIOB->CR1 |= HAL_KEY3_BIT;
#define HAL_KEY3                        (GPIOB->IDR & HAL_KEY3_BIT)

/* Return button numbers MAX */
uint8_t hal_key_init(void)
{
    HAL_KEY1_INPUT();
    HAL_KEY2_INPUT();
    HAL_KEY3_INPUT();

    return HAL_KEY_MAX_NUM;
}

/* Does support press 2 keys at the same time */
uint8_t hal_key_value(void)
{
    uint8_t key_val, key_cnt;

    key_val = 0;
    key_cnt = 0;

    if( HAL_KEY1 == 0){
        key_val |= 1;
        key_cnt++;
    }

    if( HAL_KEY2 == 0){
        key_val |= 2;
        key_cnt++;
    }

    if( HAL_KEY3 == 0){
        key_val |= 4;
        key_cnt++;
    }

    if(key_cnt != 1){
        key_val = 0;
    }

    return key_val;
}

