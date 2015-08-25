/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: STM8 hardware

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#ifndef __HARDWARE_H
#define __HARDWARE_H

//#define STM32L1XX_MD		// Choose for STM32L152K6(Flash: 32KB)
#include "stm32l1xx.h"
#include "stdbool.h"
#include "hal-irq.h"
#include "board.h"

#define F_CPU			(16000000ul)

#ifndef NULL
#define NULL (0)
#endif

/** global interrupt control definition */
#define DISABLE_IRQ()				__disable_irq()
#define ENABLE_IRQ()			    __enable_irq()

#endif
