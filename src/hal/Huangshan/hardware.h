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

#define STM8L15X_MD		// Choose for STM8L152K6(Flash: 32KB)
#include "stm8l15x.h"
#include "stdbool.h"
#include "hal-irq.h"

#define F_CPU			(16000000ul)

#ifndef NULL
#define NULL (0)
#endif

/** global interrupt control definition */
#define DISABLE_IRQ()				__disable_interrupt()
#define ENABLE_IRQ()			    __enable_interrupt()

#endif
