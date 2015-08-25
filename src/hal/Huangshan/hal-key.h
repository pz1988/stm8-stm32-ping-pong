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
#ifndef __HAL_KEY_H
#define __HAL_KEY_H

#include "hardware.h"

#define HAL_KEY_MAX_NUM             (3)

uint8_t hal_key_init(void);
uint8_t hal_key_value(void);

#endif
