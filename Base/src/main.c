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
#include "oled-draw.h"

#include <avr/io.h>

#include <stdbool.h>


static oled_colour_t const black = { 0x00, 0x00, 0x00 };
static oled_colour_t const white = { 0xFF, 0xFF, 0xFF };


void main(void)
{
    SetClockFrequency(CLKCTRL_FRQSEL_24M_gc);

    uart1.Initialize(460800);
    spi0.Initialize();
    
    oled_device_t oled;
    
    OLED_Initialize(&oled, &spi0);
    OLED_SetBackground(&oled, black);
    
    oled_shape_t circle;
    oled_point_t const circleCenter = {47, 47};
    oled_shape_parameters_t const circleParametrs = {
        .circle.center = circleCenter,
        .circle.radius = 25,
        .circle.width  = 5,
    };
    
    OLED_SetShape(&circle, OLED_SHAPE_CIRCLE, &circleParametrs, white);
    
    PauseMiliseconds(5000);
    printf("Hello from AVR128DA48!\n\r");
    
    circle.Draw(&oled, &circle);
    
    while (true) { }
}