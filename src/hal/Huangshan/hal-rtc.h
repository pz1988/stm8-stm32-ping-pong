/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: rtc low level

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#ifndef __HAL_RTC_H
#define __HAL_RTC_H

#include "hardware.h"

typedef void (*rtc_call_back_t) (void);

void hal_rtc_init(rtc_call_back_t rtc_cb);

void hal_rtc_set_timeout(uint32_t val);

uint32_t hal_rtc_get_elapsed(void);

#endif
