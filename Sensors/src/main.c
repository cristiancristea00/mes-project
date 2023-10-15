/**
 *  @file main.c
 *  @author Cristian Cristea
 *  @date October 15, 2023
 *
 *  @brief Main source file for the Base station
 **/


#include "config.h"
#include "uart.h"

#include <avr/io.h>

#include <stdbool.h>


void main(void)
{
    SetClockFrequency(CLKCTRL_FRQSEL_24M_gc);

    uart1.Initialize(460800);
    
    EnableGlobalInterrupts();
    
    while (true) 
    {
        uart1.Print("Hello from AVR128DA48!\n\r");
        PauseMiliseconds(1000);
    }
}
