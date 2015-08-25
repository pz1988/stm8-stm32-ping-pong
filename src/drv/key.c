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

#include "key.h"
#include "sys-tick.h"
#include "hal-key.h"

#define KEY_SCAN_PERIOD         (5)

uint32_t key_ts;
static uint8_t key_sta;


void key_init(void)
{
    hal_key_init();
    key_ts = millis();
    key_sta = 0;
}

uint8_t key_get(void)
{
    static uint8_t key_val_bak;
    uint8_t key_val;

    /* Refresh the key until timeout */
    if( (millis() - key_ts) <  KEY_SCAN_PERIOD ){
        return 0;
    }
    key_ts = millis();

    key_val = 0;

    switch(key_sta){
    case 0:
        key_val_bak = hal_key_value();
        if(key_val_bak != 0){
            key_sta = 1;
        }
        break;
    case 1:
        key_val = hal_key_value();
        if( key_val != key_val_bak ){
            key_val = 0;
            key_sta = 0;
        }else{
            key_sta = 2;
        }
        break;
    case 2:
        if( hal_key_value() == 0 ){
            key_sta = 0;
        }else{

        }
        break;
    }

    return key_val;
}





