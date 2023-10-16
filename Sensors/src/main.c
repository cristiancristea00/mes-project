/**
 *  @file main.c
 *  @author Cristian Cristea
 *  @date October 15, 2023
 *
 *  @brief Main source file for the Sensors station
 **/


#include "config.h"
#include "uart.h"
#include "i2c.h"

#include <avr/io.h>

#include <stdbool.h>



static inline void ScanBus(void);

void main(void)
{
    SetClockFrequency(CLKCTRL_FRQSEL_24M_gc);

    uart1.Initialize(460800);
    i2c0.Initialize(I2C_MODE_STANDARD);
    
    while (true) 
    {
        printf("Hello from AVR128DA48!\n\r");
        ScanBus();
        PauseMiliseconds(5000);
    }
}

static inline void ScanBus(void)
{
    for (uint8_t clientAddress = I2C_ADDRESS_MIN; clientAddress <= I2C_ADDRESS_MAX; ++clientAddress)
    {
        if (i2c0.IsClientAvailable(clientAddress))
        {
            printf("Got ACK from client on address 0x%02X\n\r", clientAddress);
        }
    }
    
    return;
}