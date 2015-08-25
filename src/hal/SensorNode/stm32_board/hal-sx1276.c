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

#include "hardware.h"
#include "hal-sx1276.h"
#include "hal-spi.h"
#include "sx1276-fsk-reg.h"
#include "sx1276-lora-reg.h"



sx1276_t sx1276;

Spi_t Spi1;

/*!
 * Antenna switch GPIO pins objects
 */
Gpio_t AntSwitchLf;
Gpio_t AntSwitchHf;


/** SX1276 Switch Power -- ? */
#define HAL_SX1276_SW_PWR_BIT			//(1<<7)
/** Output push pull */
#define HAL_SX1276_SW_PWR_OUTPUT()		/*GPIOD->CR2 &= ~HAL_SX1276_SW_PWR_BIT; \
										GPIOD->DDR |= HAL_SX1276_SW_PWR_BIT; \
										GPIOD->CR1 |= HAL_SX1276_SW_PWR_BIT; \
										GPIOD->CR2 |= HAL_SX1276_SW_PWR_BIT;*/
/** Input floating */
#define HAL_SX1276_SW_PWR_INPUT()		/*GPIOD->CR2 &= ~HAL_SX1276_SW_PWR_BIT; \
										GPIOD->DDR &= ~HAL_SX1276_SW_PWR_BIT; \
										GPIOD->CR1 &= ~HAL_SX1276_SW_PWR_BIT;*/
#define HAL_SX1276_SW_PWR_H()			//GPIOD->ODR |= HAL_SX1276_SW_PWR_BIT;
#define HAL_SX1276_SW_PWR_L()			//GPIOD->ODR &= ~HAL_SX1276_SW_PWR_BIT;
#define HAL_SX1276_SW_PWR_ON()			HAL_SX1276_SW_PWR_H()
#define HAL_SX1276_SW_PWR_OFF()			HAL_SX1276_SW_PWR_L()

/** SX1276 HF Switch control IO -- PA0 */
#define HAL_SX1276_SW_HF_BIT			//(1<<0)
/** Output push pull */
#define HAL_SX1276_SW_HF_OUTPUT()		GpioInit( &AntSwitchHf, RADIO_ANT_SWITCH_HF, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 )


/** Input floating */
#define HAL_SX1276_SW_HF_INPUT()		GpioInit( &AntSwitchHf, RADIO_ANT_SWITCH_HF, PIN_INPUT, PIN_OPEN_DRAIN, PIN_NO_PULL, 0 )


#define HAL_SX1276_SW_HF_H()			GpioWrite( &AntSwitchHf, 1 )
#define HAL_SX1276_SW_HF_L()			GpioWrite( &AntSwitchHf, 0 )
#define HAL_SX1276_SW_HF_TX()			HAL_SX1276_SW_HF_H()
#define HAL_SX1276_SW_HF_RX()			HAL_SX1276_SW_HF_L()

/** SX1276 LF Switch control IO -- PA1  */
#define HAL_SX1276_SW_LF_BIT			//(1<<1)
/** Output push pull */
#define HAL_SX1276_SW_LF_OUTPUT()		GpioInit( &AntSwitchLf, RADIO_ANT_SWITCH_LF, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 )


/** Input floating */
#define HAL_SX1276_SW_LF_INPUT()		GpioInit( &AntSwitchLf, RADIO_ANT_SWITCH_LF, PIN_INPUT, PIN_OPEN_DRAIN, PIN_NO_PULL, 0 )


#define HAL_SX1276_SW_LF_H()			GpioWrite( &AntSwitchLf, 1 )
#define HAL_SX1276_SW_LF_L()			GpioWrite( &AntSwitchLf, 0 )
#define HAL_SX1276_SW_LF_TX()			HAL_SX1276_SW_LF_L()
#define HAL_SX1276_SW_LF_RX()			HAL_SX1276_SW_LF_H()

/** SX1276 Digital IO DIO0 -- PB11 */
#define HAL_SX1276_DIO0_BIT				//(1<<11)
/** Input floating */
#define HAL_SX1276_DIO0_INPUT()			GpioInit( &sx1276.DIO0, RADIO_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 )


#define HAL_SX1276_DIO0					GpioRead( &sx1276.DIO0 )
#define HAL_SX1276_DIO0_INT_ENABLE()	GpioWrite( &sx1276.DIO0, 1 )
#define HAL_SX1276_DIO0_INT_DISABLE()	GpioWrite( &sx1276.DIO0, 0 )


/** SX1276 Digital IO DIO1 -- PC13 */
#define HAL_SX1276_DIO1_BIT				//(1<<13)
#define HAL_SX1276_DIO1_INPUT()			GpioInit( &sx1276.DIO1, RADIO_DIO_1, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 )


#define HAL_SX1276_DIO1					GpioRead( &sx1276.DIO1 )
#define HAL_SX1276_DIO1_INT_ENABLE()	GpioWrite( &sx1276.DIO1, 1 )
#define HAL_SX1276_DIO1_INT_DISABLE()	GpioWrite( &sx1276.DIO1, 0 )

/** SX1276 Digital IO DIO2 -- PB9 */
#define HAL_SX1276_DIO2_BIT				//(1<<9)
#define HAL_SX1276_DIO2_INPUT()			GpioInit( &sx1276.DIO2, RADIO_DIO_2, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 )


#define HAL_SX1276_DIO2					GpioRead( &sx1276.DIO2 )
#define HAL_SX1276_DIO2_INT_ENABLE()	GpioWrite( &sx1276.DIO2, 1 )
#define HAL_SX1276_DIO2_INT_DISABLE()	GpioWrite( &sx1276.DIO2, 0 )

///** PD7 is used for RF Switch Power control */
#if 0
/** SX1276 Digital IO DIO3 -- PD7 */
#define HAL_SX1276_DIO3_BIT
#define HAL_SX1276_DIO3_INPUT()
#define HAL_SX1276_DIO3
#define HAL_SX1276_DIO3_INT_ENABLE()
#define HAL_SX1276_DIO3_INT_DISABLE()
#else
/*IO DIO2 -- PB4*/
#define HAL_SX1276_DIO3_BIT				//(1<<4)
#define HAL_SX1276_DIO3_INPUT()			GpioInit( &sx1276.DIO3, RADIO_DIO_3, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 )


#define HAL_SX1276_DIO3					GpioRead( &sx1276.DIO3 )
#define HAL_SX1276_DIO3_INT_ENABLE()	GpioWrite( &sx1276.DIO3, 1 )
#define HAL_SX1276_DIO3_INT_DISABLE()	GpioWrite( &sx1276.DIO3, 0 )
#endif

/** SX1276 Digital IO DIO4 -- PB3 (STM8L152 PC port doesn't support EXTI function) */
#define HAL_SX1276_DIO4_BIT				//(1<<3)
#define HAL_SX1276_DIO4_INPUT()			GpioInit( &sx1276.DIO4, RADIO_DIO_4, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 )


#define HAL_SX1276_DIO4					GpioRead( &sx1276.DIO4 )
#define HAL_SX1276_DIO4_INT_ENABLE()
#define HAL_SX1276_DIO4_INT_DISABLE()

#if 1
#define HAL_SX1276_DIO5_INPUT()
#define HAL_SX1276_DIO5_INT_ENABLE()
#define HAL_SX1276_DIO5_INT_DISABLE()
#else
/** SX1276 Digital IO DIO5 -- PA15 (STM8L152 PC port doesn't support EXTI function)*/
#define HAL_SX1276_DIO5_BIT				//(1<<15)
#define HAL_SX1276_DIO5_INPUT()			GpioInit( &sx1276.DIO5, RADIO_DIO_5, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 )


#define HAL_SX1276_DIO5					GpioRead( &sx1276.DIO5 )
#define HAL_SX1276_DIO5_INT_ENABLE()
#define HAL_SX1276_DIO5_INT_DISABLE()
#endif


hal_sx1276_irq_callback_t hal_sx1276_irq_callback[] = { sx1276_irq_dio0, sx1276_irq_dio1,
                                                         sx1276_irq_dio2, sx1276_irq_dio3,
                            							 sx1276_irq_dio4, NULL };


void SX1276IoIrqInit( hal_sx1276_irq_callback_t *irqHandlers,uint32_t *Exit_Line_Sum )
{
    GpioSetInterrupt( &sx1276.DIO0, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[0], Exit_Line_Sum );
    GpioSetInterrupt( &sx1276.DIO1, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[1], Exit_Line_Sum );
    GpioSetInterrupt( &sx1276.DIO2, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[2], Exit_Line_Sum );
    GpioSetInterrupt( &sx1276.DIO3, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[3], Exit_Line_Sum );
    GpioSetInterrupt( &sx1276.DIO4, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[4], Exit_Line_Sum );
    GpioSetInterrupt( &sx1276.DIO5, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[5], Exit_Line_Sum );
}


////for debugger
///*!
// * Performs the Rx chain calibration for LF and HF bands
// * \remark Must be called just after the reset so all registers are at their
// *         default values
// */
//static void RxChainCalibration( void )
//{
//    uint8_t regPaConfigInitVal;
//    uint32_t initialFreq;
//	

//    // Save context
//    regPaConfigInitVal = sx1276_read( REG_PACONFIG );
//    initialFreq = ( double )( ( ( uint32_t )sx1276_read( REG_FRFMSB ) << 16 ) |
//                              ( ( uint32_t )sx1276_read( REG_FRFMID ) << 8 ) |
//                              ( ( uint32_t )sx1276_read( REG_FRFLSB ) ) ) * ( double )61.03515625;

//    // Cut the PA just in case, RFO output, power = -1 dBm
//    sx1276_write( REG_PACONFIG, 0x00 );

//    // Launch Rx chain calibration for LF band
//    sx1276_write( REG_IMAGECAL, ( sx1276_read( REG_IMAGECAL ) & 0xBF ) | 0x40 );
//    while( ( sx1276_read( REG_IMAGECAL ) & 0x20 ) == 0x20 )
//    {
//    }

//    // Sets a Frequency in HF band
//    sx1276_set_channel( 868000000 );

//    // Launch Rx chain calibration for HF band 
//    sx1276_write( REG_IMAGECAL, ( sx1276_read( REG_IMAGECAL ) & 0xBF ) | 0x40 );
//    while( ( sx1276_read( REG_IMAGECAL ) & 0x20 ) == 0x20 )
//    {
//    }

//    // Restore context
//    sx1276_write( REG_PACONFIG, regPaConfigInitVal );
//    sx1276_set_channel( initialFreq );
//}
/*debugger end*/

void hal_sx1276_init(void)
{
    uint32_t Exit_Line_Sum = 0;
	
	 /** SPI 10MHz, SCK Low when IDlE, sample on rising edge */
	Spi1 = sx1276.Spi;
	hal_spi_init(10000000, SPI_SCK_POLARITY_LOW, SPI_SAMPLE_ON_RISING_EDGE);
	
	HAL_SX1276_SW_LF_INPUT();
	HAL_SX1276_SW_HF_INPUT();
	HAL_SX1276_SW_PWR_INPUT();
	
	HAL_SX1276_DIO0_INPUT();
	HAL_SX1276_DIO1_INPUT();
	HAL_SX1276_DIO2_INPUT();
	HAL_SX1276_DIO3_INPUT();
	HAL_SX1276_DIO4_INPUT();
	HAL_SX1276_DIO5_INPUT();
  
//	//sys_tick_init();
//	//for debugger
//	//to be confirmed
//	RxChainCalibration();
/*debugger end*/
	
    DISABLE_IRQ();

	SX1276IoIrqInit( hal_sx1276_irq_callback, &Exit_Line_Sum );
	//EXTI->CR2 = 0x55;		// Px4~7 rising edge interrupt

    ENABLE_IRQ();

	/** Clear interrupt flag */
	EXTI_ClearFlag( Exit_Line_Sum );
	/*EXTI->SR1 |= (HAL_SX1276_DIO0_BIT | HAL_SX1276_DIO1_BIT | \
					HAL_SX1276_DIO2_BIT | HAL_SX1276_DIO3_BIT);*/

	HAL_SX1276_NSS_OUTPUT();
	HAL_SX1276_NSS_H();

	//HAL_SX1276_RST_INPUT();
	
}

void hal_sx1276_set_irq(sx1276_irq_t irq, bool sta, hal_sx1276_irq_callback_t cb)
{
	hal_sx1276_irq_callback[irq] = cb;
	switch(irq){
	case SX1276_IRQ_DIO0:
		if(sta){
			HAL_SX1276_DIO0_INT_ENABLE();
		}else{
			HAL_SX1276_DIO0_INT_DISABLE();
		}
		break;
	case SX1276_IRQ_DIO1:
		if(sta){
			HAL_SX1276_DIO1_INT_ENABLE();
		}else{
			HAL_SX1276_DIO1_INT_DISABLE();
		}
		break;
	case SX1276_IRQ_DIO2:
		if(sta){
			HAL_SX1276_DIO2_INT_ENABLE();
		}else{
			HAL_SX1276_DIO2_INT_DISABLE();
		}
		break;
	case SX1276_IRQ_DIO3:
		if(sta){
			HAL_SX1276_DIO3_INT_ENABLE();
		}else{
			HAL_SX1276_DIO3_INT_DISABLE();
		}
		break;
	case SX1276_IRQ_DIO4:
		if(sta){
			HAL_SX1276_DIO4_INT_ENABLE();
		}else{
			HAL_SX1276_DIO4_INT_DISABLE();
		}
		break;
	case SX1276_IRQ_DIO5:
		if(sta){
			HAL_SX1276_DIO5_INT_ENABLE();
		}else{
			HAL_SX1276_DIO5_INT_DISABLE();
		}
		break;
	}
}

void hal_sx1276_set_rf_switch_mode(sx1276_rf_switch_mode_t rf_switch)
{
	static uint8_t init_flag = 0;	// 0: IO uninitialized, 1: initialed.

	switch(rf_switch){
	case SX1276_RF_SWITCH_RX:
		if( init_flag == 0){
			init_flag = 1;
			/** RF Switch IO initial */
			HAL_SX1276_SW_HF_OUTPUT();
			HAL_SX1276_SW_LF_OUTPUT();
			HAL_SX1276_SW_PWR_OUTPUT();
			HAL_SX1276_SW_PWR_ON();
		}
		HAL_SX1276_SW_HF_RX();
		HAL_SX1276_SW_LF_RX();
		break;
	case SX1276_RF_SWITCH_TX:
		if( init_flag == 0){
			init_flag = 1;
			/** RF Switch IO initial */
			HAL_SX1276_SW_HF_OUTPUT();
			HAL_SX1276_SW_LF_OUTPUT();
			HAL_SX1276_SW_PWR_OUTPUT();
			HAL_SX1276_SW_PWR_ON();
		}
		HAL_SX1276_SW_HF_TX();
		HAL_SX1276_SW_LF_TX();
		break;
	case SX1276_RF_SWITCH_IDLE:
		init_flag = 0;
		/** Disable RF Switch IO to save power */
		HAL_SX1276_SW_HF_INPUT();
		HAL_SX1276_SW_LF_INPUT();
		HAL_SX1276_SW_PWR_INPUT();
		break;
	default:
		break;
	}
}

sx1276_tx_port_t hal_sx1276_get_tx_port(uint32_t freq)
{
	if( freq > SX1276_LF_FREQ_MAX ){
		/** RFO is used as HF OUTPUT*/
		return SX1276_TX_PORT_RFO;
	}

	/** PA_BOOST is used as LF OUTPUT */
	return SX1276_TX_PORT_PA_BOOST;
}

//Need to confirmed 
///** Interrupt routine */
//INTERRUPT_HANDLER( DIO0, 12 )
//{
//    uint32_t Exti_Line = 0;

//	Exti_Line = (uint32_t)( 0x01 << ( RADIO_DIO_0 & 0x0F ) );
//	
//	if( EXTI_GetITStatus( Exti_Line )){
//        /** Clear IRQ Flag */
//		EXTI_ClearFlag( Exti_Line );
//		//EXTI->SR1 |= HAL_SX1276_DIO0_BIT;

//		if( hal_sx1276_irq_callback[0] != NULL ){
//			hal_sx1276_irq_callback[0]();
//		}
//	}
//}

//Need to confirmed 
//INTERRUPT_HANDLER( DIO1, 13 )
//{
//    uint32_t Exti_Line = 0;

//	Exti_Line = (uint32_t)( 0x01 << ( RADIO_DIO_1 & 0x0F ) );
//	if( EXTI_GetITStatus( Exti_Line )){
//        /** Clear IRQ Flag */
//		EXTI_ClearFlag( Exti_Line );
//		//EXTI->SR1 |= HAL_SX1276_DIO0_BIT;

//		if( hal_sx1276_irq_callback[1] != NULL ){
//			hal_sx1276_irq_callback[1]();
//		}
//	}
//}

//Need to confirmed 
//INTERRUPT_HANDLER( DIO2, 14 )
//{
//	 uint32_t Exti_Line = 0;

//	Exti_Line = (uint32_t)( 0x01 << ( RADIO_DIO_2& 0x0F ) );
//	if( EXTI_GetITStatus( Exti_Line )){
//        /** Clear IRQ Flag */
//		EXTI_ClearFlag( Exti_Line );
//		//EXTI->SR1 |= HAL_SX1276_DIO0_BIT;

//		if( hal_sx1276_irq_callback[2] != NULL ){
//			hal_sx1276_irq_callback[2]();
//		}
//	}
//}

//Need to confirmed 
//INTERRUPT_HANDLER( DIO3, 15 )
//{
//	uint32_t Exti_Line = 0;

//	Exti_Line = (uint32_t)( 0x01 << ( RADIO_DIO_3& 0x0F ) );
//	if( EXTI_GetITStatus( Exti_Line )){
//        /** Clear IRQ Flag */
//		EXTI_ClearFlag( Exti_Line );
//		//EXTI->SR1 |= HAL_SX1276_DIO0_BIT;

//        if( hal_sx1276_irq_callback[3] != NULL ){
//			hal_sx1276_irq_callback[3]();
//		}
//	}
//}
