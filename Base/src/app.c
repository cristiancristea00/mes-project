/**
 *  @file app.c
 *  @author Cristian Cristea
 *  @date October 22, 2023
 *
 *  @brief Application source file for the Base station
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
#include "oled.h"
#include "oled-draw.h"
#include "bluetooth.h"

#include <stdio.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#define DISPLAY_SPACING    5
#define MAX_ROW_DISPLAY    20


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                            Private (static) API                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Initializes the system's clock and peripherals (UART and SPI).
 **/
STATIC_INLINE void SystemInitialize(void);

/**
 * @brief Initializes the system's moudules: OLED and Bluetooth.
 * @param[in, out] oled The OLED module to initialize
 * @param[in, out] bluetooth The Bluetooth module to initialize
 **/
STATIC_INLINE void ModulesInitialize(oled_device_t * const oled , bluetooth_device_t * const bluetooth);

/**
 * @brief Displays the data on the console and OLED screen.
 * @param[in] sensorsData The data to display
 * @param[in] oled The OLED module to use for the display
 **/
STATIC_INLINE void DisplayData(bme280_data_t const * const sensorsData, oled_device_t const * const oled);

/**
 * @brief Writes on the OLED the temperature, pressure and humidity initial text.
 * @param[in] oled The OLED module to use
 **/
STATIC_INLINE void DisplaySetup(oled_device_t const * const oled);

/**
 * @brief Clears the OLED current line of text.
 * @param[in] oled The OLED module to use
 * @param[in] start The start position of the line to clear
 **/
STATIC_INLINE void ClearLineOfText(oled_device_t const * const oled, oled_point_t const start);


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Private (static) definitions                        //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

static oled_colour_t const black = { 0x00, 0x00, 0x00 };
static oled_colour_t const grey  = { 0xF0, 0xF0, 0xF0 };

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
    
    spi0.Initialize();

    return;
}

STATIC_INLINE void ModulesInitialize(oled_device_t * const oled , bluetooth_device_t * const bluetooth)
{
    OLED_Initialize(oled, &spi0);
    OLED_SetBackground(oled, black);
    DisplaySetup(oled);

#if defined __AVR128DA48__
    BLUETOOTH_Initialize(bluetooth, &uart0);
#elif defined __AVR64DD32__
    BLUETOOTH_Initialize(bluetooth, &uart1);
#else
    #error "Invalid device!"
#endif

    return;
}

STATIC_INLINE void DisplayData(bme280_data_t const * const sensorsData, oled_device_t const * const oled)
{
    char temperatureBuffer[MAX_ROW_DISPLAY];
    double const temperature = BME280_GetDisplayTemperature(sensorsData);
    snprintf(temperatureBuffer, MAX_ROW_DISPLAY, "%0.2lf degrees C", temperature);

    char pressureBuffer[MAX_ROW_DISPLAY];
    double const pressure = BME280_GetDisplayPressure(sensorsData);
    snprintf(pressureBuffer, MAX_ROW_DISPLAY, "%0.2lf hPa", pressure);

    char humidityBuffer[MAX_ROW_DISPLAY];
    double const humidity = BME280_GetDisplayHumidity(sensorsData);
    snprintf(humidityBuffer, MAX_ROW_DISPLAY, "%0.2lf %c", humidity, '%');


    oled_point_t temperatureTextStart = {
        .x = 0,
        .y = 1 * (OLED_DRAW_FONT_HEIGHT + DISPLAY_SPACING)
    };
    oled_point_t pressureTextStart = {
        .x = 0,
        .y = 3 * (OLED_DRAW_FONT_HEIGHT + DISPLAY_SPACING)
    };
    oled_point_t humidityTextStart = {
        .x = 0,
        .y = 5 * (OLED_DRAW_FONT_HEIGHT + DISPLAY_SPACING)
    };

    oled_shape_parameters_t temperaturetTextParameters;
    temperaturetTextParameters.string.scale_x = 1;
    temperaturetTextParameters.string.scale_y = 1;
    temperaturetTextParameters.string.start   = temperatureTextStart;
    temperaturetTextParameters.string.data    = (uint8_t const *) temperatureBuffer;
    oled_shape_t temperatureText;
    OLED_SetShape(&temperatureText, OLED_SHAPE_STRING, &temperaturetTextParameters, grey);

    oled_shape_parameters_t pressureTextParameters;
    pressureTextParameters.string.scale_x = 1;
    pressureTextParameters.string.scale_y = 1;
    pressureTextParameters.string.start   = pressureTextStart;
    pressureTextParameters.string.data    = (uint8_t const *) pressureBuffer;
    oled_shape_t pressureText;
    OLED_SetShape(&pressureText, OLED_SHAPE_STRING, &pressureTextParameters, grey);

    oled_shape_parameters_t humidityTextParameters;
    humidityTextParameters.string.scale_x = 1;
    humidityTextParameters.string.scale_y = 1;
    humidityTextParameters.string.start   = humidityTextStart;
    humidityTextParameters.string.data    = (uint8_t const *) humidityBuffer;
    oled_shape_t humidityText;
    OLED_SetShape(&humidityText, OLED_SHAPE_STRING, &humidityTextParameters, grey);

    ClearLineOfText(oled, temperatureTextStart);
    temperatureText.Draw(oled, &temperatureText);

    ClearLineOfText(oled, pressureTextStart);
    pressureText.Draw(oled, &pressureText);

    ClearLineOfText(oled, humidityTextStart);
    humidityText.Draw(oled, &humidityText);

    printf("Temperature: %0.2lf °C\n\r", temperature);
    printf("Pressure: %0.2lf hPa\n\r", pressure);
    printf("Relative humidity: %0.2lf %c\n\r", humidity, '%');

    return;
}

STATIC_INLINE void DisplaySetup(oled_device_t const * const oled)
{
    oled_shape_parameters_t textParameters;
    oled_shape_t text;

    textParameters.string.scale_x = 1;
    textParameters.string.scale_y = 1;

    oled_point_t textStart;

    textStart.x = 0;
    textStart.y = 0;
    textParameters.string.start = textStart;
    textParameters.string.data = (uint8_t const *) "Temperature:";
    OLED_SetShape(&text, OLED_SHAPE_STRING, &textParameters, grey);
    
    text.Draw(oled, &text);

    textStart.x = 0;
    textStart.y = 2 * (OLED_DRAW_FONT_HEIGHT + DISPLAY_SPACING);
    textParameters.string.start = textStart;
    textParameters.string.data = (uint8_t const *) "Pressure:";
    OLED_SetShape(&text, OLED_SHAPE_STRING, &textParameters, grey);
    
    text.Draw(oled, &text);

    textStart.x = 0;
    textStart.y = 4 * (OLED_DRAW_FONT_HEIGHT + DISPLAY_SPACING);
    textParameters.string.start = textStart;
    textParameters.string.data = (uint8_t const *) "Humidity:";
    OLED_SetShape(&text, OLED_SHAPE_STRING, &textParameters, grey);
    
    text.Draw(oled, &text);

    return;
}

STATIC_INLINE void ClearLineOfText(oled_device_t const * const oled, oled_point_t const start)
{
    oled_point_t const end = { start.x + MAX_ROW_DISPLAY * OLED_DRAW_FONT_WIDTH, start.y + OLED_DRAW_FONT_HEIGHT };
    
    oled_shape_parameters_t clearBoxParameters;

    clearBoxParameters.filled_rectangle.start = start;
    clearBoxParameters.filled_rectangle.end   = end;
    
    oled_shape_t clearBox;

    OLED_SetShape(&clearBox, OLED_SHAPE_FILLED_RECTANGLE, &clearBoxParameters, black);
    
    clearBox.Draw(oled, &clearBox);

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

    oled_device_t oledc;
    bluetooth_device_t bluetooth;

    ModulesInitialize(&oledc, &bluetooth);

    bme280_data_t sensorsData;

    PauseMiliseconds(5000);
    
#if defined __AVR128DA48__
    uart1.Print("Hello from AVR128DA48 - Base station!\n\r");
#elif defined __AVR64DD32__
    uart0.Print("Hello from AVR64DD32 - Base station!\n\r");
#else
    #error "Invalid device!"
#endif

    EnableGlobalInterrupts();

    while (true)
    {
        if (BLUETOOTH_ReceiveData(&bluetooth, &sensorsData, BME280_StructInterpret) == BLUETOOTH_OK)
        {
            DisplayData(&sensorsData, &oledc);
        }
    }
}