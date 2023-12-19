/**
 *  @file app.c
 *  @author Cristian Cristea
 *  @date October 22, 2023
 *
 *  @brief Application source file for the Sensors station
 **/


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "app.h"

#include "config.h"
#include "utils.h"
#include "uart.h"
#include "i2c.h"
#include "bme280.h"
#include "bluetooth.h"

#include <stdio.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                            Private (static) API                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Initializes the system's clock and peripherals (UART and I2C).
 **/
STATIC_INLINE void SystemInitialize(void);

/**
 * @brief Initializes the system's moudules: BME280 and Bluetooth.
 * @param[in, out] bme280 The BME280 module
 * @param[in, out] bluetooth The Bluetooth module to initialize
 **/
STATIC_INLINE void ModulesInitialize(bme280_device_t * const bme280, bluetooth_device_t * const bluetooth);

/**
 * @brief Display the data on the console.
 * @param sensorsData The data to display
 **/
STATIC_INLINE void DisplayData(bme280_data_t const * const sensorsData);


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Private (static) definitions                        //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

STATIC_INLINE void SystemInitialize(void)
{
    SetClockFrequency(CLKCTRL_FRQSEL_24M_gc);

#if defined __AVR128DA48__
    uart1.Initialize(460800);
    uart0.InitializeWithReceive(460800, BLUETOOTH_ReceiveCallback);
#elif defined __AVR64DD32__
    uart0.Initialize(460800);
    uart1.InitializeWithReceive(460800, BLUETOOTH_ReceiveCallback);
#else
    #error "Invalid device!"
#endif
    
    i2c0.Initialize(I2C_MODE_FAST_PLUS);

    return;
}

STATIC_INLINE void ModulesInitialize(bme280_device_t * const bme280, bluetooth_device_t * const bluetooth)
{
    bme280_settings_t settings = {
        .temperatureOversampling = BME280_OVERSAMPLING_16X,
        .pressureOversampling = BME280_OVERSAMPLING_16X,
        .humidityOversampling = BME280_OVERSAMPLING_16X,
        .iirFilterCoefficients = BME280_IIR_FILTER_16,
        .powerMode = BME280_NORMAL_MODE,
        .standbyTime = BME280_STANDBY_TIME_250_MS
    };
    
    BME280_Initialize(bme280, &BME280_I2C_Handler, &i2c0, BME280_I2C_ADDRESS, &settings);

#if defined __AVR128DA48__
    BLUETOOTH_Initialize(bluetooth, &uart0);
#elif defined __AVR64DD32__
    BLUETOOTH_Initialize(bluetooth, &uart1);
#else
    #error "Invalid device!"
#endif

    return;
}

STATIC_INLINE void DisplayData(bme280_data_t const * const sensorsData)
{
    printf("Temperature: %0.2lf °C\n\r", BME280_GetDisplayTemperature(sensorsData));
    printf("Pressure: %0.2lf hPa\n\r", BME280_GetDisplayPressure(sensorsData));
    printf("Relative humidity: %0.2lf %c\n\r", BME280_GetDisplayHumidity(sensorsData), '%');

    return;
}


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Public definitions                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

void Application_Run(void)
{
    SystemInitialize();

    bme280_device_t weatherClick;
    bluetooth_device_t bluetooth;

    ModulesInitialize(&weatherClick, &bluetooth);

    uint8_t serializedSensorsData[BME280_SERIALIZED_SIZE] = { 0 };

    PauseMiliseconds(5000);
    
#if defined __AVR128DA48__
    uart1.Print("Hello from AVR128DA48 - Sensors station!\n\r");
#elif defined __AVR64DD32__
    uart0.Print("Hello from AVR64DD32 - Sensors station!\n\r");
#else
    #error "Invalid device!"
#endif

    EnableGlobalInterrupts();

    while (true)
    {
        if (BME280_GetSensorData(&weatherClick) == BME280_OK)
        {
            BME280_SerializeSensorData(&weatherClick, &serializedSensorsData);
            BLUETOOTH_SendData(&bluetooth, &serializedSensorsData, BME280_SERIALIZED_SIZE);
            DisplayData(&weatherClick.data);
            PauseMiliseconds(20000);
        }
    }
}