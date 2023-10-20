/**
 *  @file main.c
 *  @author Cristian Cristea
 *  @date October 15, 2023
 *
 *  @brief Main source file for the Base station
 **/


#include "utils.h"
#include "uart.h"
#include "spi.h"

#include <avr/io.h>

#include <stdbool.h>



static inline void ScanBus(void);

void main(void)
{
    SetClockFrequency(CLKCTRL_FRQSEL_24M_gc);

    uart1.Initialize(460800);
    spi0.Initialize();
    
    PauseMiliseconds(5000);
    printf("Hello from AVR128DA48!\n\r");
    
    while (true) { }
}