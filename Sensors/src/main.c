/**
 *  @file main.c
 *  @author Cristian Cristea
 *  @date October 15, 2023
 *
 *  @brief Main source file for the Sensors station
 **/


#include "utils.h"
#include "uart.h"
#include "i2c.h"
#include "bme280.h"

#include <avr/io.h>

#include <stdio.h>
#include <stdbool.h>



static inline void ScanBus(void);

void main(void)
{
    SetClockFrequency(CLKCTRL_FRQSEL_24M_gc);

    uart1.Initialize(460800);
    i2c0.Initialize(I2C_MODE_FAST);
    
    bme280_device_t weatherClick;
    
    bme280_settings_t settings = {
        .temperatureOversampling = BME280_OVERSAMPLING_16X,
        .pressureOversampling = BME280_OVERSAMPLING_16X,
        .humidityOversampling = BME280_OVERSAMPLING_16X,
        .iirFilterCoefficients = BME280_IIR_FILTER_16,
        .powerMode = BME280_NORMAL_MODE,
        .standbyTime = BME280_STANDBY_TIME_250_MS
    };
    
    PauseMiliseconds(5000);
    printf("Hello from AVR128DA48!\n\r");
    ScanBus();
    
    BME280_Inititialize(&weatherClick, &BME280_I2C_Handler, &i2c0, BME280_I2C_ADDRESS, &settings);
    
    while (true) 
    {
        if (BME280_GetSensorData(&weatherClick) == BME280_OK)
        {
            printf("Temperature: %0.2lf °C\n\r", BME280_GetDisplayTemperature(&weatherClick.data));
            printf("Pressure: %0.2lf hPa\n\r", BME280_GetDisplayPressure(&weatherClick.data));
            printf("Relative humidity: %0.2lf %c\n\r", BME280_GetDisplayHumidity(&weatherClick.data), '%');
        }
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