/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: IRQ Management, this driver makes disable/enable could be nested.
This implement is inspired by RIOT-OS/cpu/ARCH/irq_arch.c

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#include "hal-irq.h"

irq_state_t irq_disable(void)
{
    irq_state_t sta = __get_interrupt_state();
    __disable_interrupt();
    return sta;
}

irq_state_t irq_enable(void)
{
    irq_state_t sta = __get_interrupt_state();
    __enable_interrupt();
    return sta;
}

void irq_restore(irq_state_t sta)
{
    __set_interrupt_state(sta);
}

int irq_is_in_int(void)
{
    return 0;
}
