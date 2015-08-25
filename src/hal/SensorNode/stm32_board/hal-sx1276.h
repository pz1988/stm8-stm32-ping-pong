/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: SX1276 Low-Level driver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#ifndef __HAL_SX1276_H
#define __HAL_SX1276_H

#include "hardware.h"
#include "sys-tick.h"

/*!
 * Radio hardware and global parameters
 */
typedef struct SX1276_s
{
    Gpio_t        Reset;
    Gpio_t        DIO0;
    Gpio_t        DIO1;
    Gpio_t        DIO2;
    Gpio_t        DIO3;
    Gpio_t        DIO4;
    Gpio_t        DIO5;
    Spi_t         Spi;
    uint8_t       RxTx;
    //RadioSettings_t Settings;
}sx1276_t;

extern sx1276_t sx1276;

#define SX1276_FXOSC				(32000000ul)
#define SX1276_LF_FREQ_MAX			(525000000ul)

#define SX1276_RSSI_OFFSET_HF       (-157)
#define SX1276_RSSI_OFFSET_LF       (-164)

/** SX1276 SPI NSS Pin -- PA4 */
#define HAL_SX1276_NSS_BIT			    //(1<<4)
/** Output push pull */
#define HAL_SX1276_NSS_OUTPUT()			GpioInit( &sx1276.Spi.Nss, RADIO_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 )


/** Input floating */
#define HAL_SX1276_NSS_INPUT()			GpioInit( &sx1276.Spi.Nss, RADIO_NSS, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 )


#define HAL_SX1276_NSS_H()				GpioWrite( &sx1276.Spi.Nss, 1 )
#define HAL_SX1276_NSS_L()				GpioWrite( &sx1276.Spi.Nss, 0 )

/** SX1276 SPI Reset Pin -- PB10 */
#define HAL_SX1276_RST_BIT				//(1<<10)
/** Output push pull */
#define HAL_SX1276_RST_OUTPUT()			GpioInit( &sx1276.Reset, RADIO_RESET, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 )


/** Input floating */
#define HAL_SX1276_RST_INPUT()			GpioInit( &sx1276.Reset, RADIO_RESET, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 )


#define HAL_SX1276_RST_H()				GpioWrite( &sx1276.Reset, 1 )
#define HAL_SX1276_RST_L()				GpioWrite( &sx1276.Reset, 0 )

typedef void ( *hal_sx1276_irq_callback_t ) (void);

typedef enum{
	SX1276_IRQ_DIO0 = 0,
	SX1276_IRQ_DIO1,
	SX1276_IRQ_DIO2,
	SX1276_IRQ_DIO3,
	SX1276_IRQ_DIO4,
	SX1276_IRQ_DIO5,
}sx1276_irq_t;

typedef enum{
	SX1276_RF_SWITCH_IDLE,		//LOW POWER MODE
	SX1276_RF_SWITCH_RX,
	SX1276_RF_SWITCH_TX,
}sx1276_rf_switch_mode_t;

typedef enum{
	/* SX1276 use RFO_HF/RFO_LF as output, and SX1276 has ability to
		switch between RFO_HF/RFO_LF automatically,
		depends on the frequency used. */
	SX1276_TX_PORT_RFO,
	SX1276_TX_PORT_PA_BOOST,
}sx1276_tx_port_t;

void hal_sx1276_init(void);
void hal_sx1276_set_irq(sx1276_irq_t irq, bool sta, hal_sx1276_irq_callback_t cb);
void hal_sx1276_set_rf_switch_mode(sx1276_rf_switch_mode_t rf_switch);
sx1276_tx_port_t hal_sx1276_get_tx_port(uint32_t freq);

/** DIOx callback definition */
void sx1276_irq_dio0(void);
void sx1276_irq_dio1(void);
void sx1276_irq_dio2(void);
void sx1276_irq_dio3(void);
void sx1276_irq_dio4(void);
void sx1276_irq_dio5(void);


#endif




