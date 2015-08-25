/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: Key/button driver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#ifndef __KEY_H
#define __KEY_H

#include "hardware.h"

typedef enum{
    KEY1 = 0x01,
    KEY2 = 0x02,
    KEY3 = 0x04,
    KEY4 = 0x08,
    KEY5 = 0x10,
    KEY6 = 0x20,
    KEY7 = 0x40,
    KEY8 = 0x80,
}key_val_t;

void key_init(void);
uint8_t key_get(void);

#endif
