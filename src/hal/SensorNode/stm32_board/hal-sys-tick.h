/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: system tick HAL driver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/
#include <stdint.h>

/*!
 * \brief Timer time variable definition
 */
#ifndef TimerTime_t
typedef uint64_t TimerTime_t;
#endif

#ifndef __HAL_SYS_TICK_H
#define __HAL_SYS_TICK_H

#define HAL_SYS_TICK_PERIOD             (1000)   // unit us

typedef void (*hal_sys_tick_handle_t) (void);

void hal_sys_tick_init(hal_sys_tick_handle_t handler);

#endif



