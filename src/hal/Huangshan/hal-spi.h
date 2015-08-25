/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: spi low level driver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#ifndef __HAL_SPI_H
#define __HAL_SPI_H

typedef enum{
	SPI_SCK_POLARITY_LOW,
	SPI_SCK_POLARITY_HIGH,
}spi_sck_polarity_t;

typedef enum{
	SPI_SAMPLE_ON_RISING_EDGE,
	SPI_SAMPLE_ON_FALLING_EDGE,
}spi_sample_edge_t;

void hal_spi_init( uint32_t freq, spi_sck_polarity_t sck_pol, spi_sample_edge_t smpl_edge);
uint8_t hal_spi_wr( uint8_t byte );

#endif

