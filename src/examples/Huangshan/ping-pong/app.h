/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: PingPong application

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/
#ifndef __APP_H
#define __APP_H

#include "hardware.h"

typedef enum{
    APP_STA_IDLE,
    APP_STA_WAIT_PINGPONG,
    APP_STA_SEND_PING,
    /* Slave */
    APP_STA_SLAVE_SEND_PONG_DELAY,
    APP_STA_SLAVE_SEND_PONG,
    APP_STA_SLAVE_WAIT_PING,
    /* Master */
    APP_STA_MASTER_SEND_PING_DELAY,
    APP_STA_MASTER_SEND_PING,
    APP_STA_MASTER_WAIT_PONG,
}app_sta_t;

typedef enum{
    APP_PKT_PING,
    APP_PKT_PONG,
}app_pkt_t;

void app_init(void);
void app_evt(void);
void app_update_lcd(void);

#endif
