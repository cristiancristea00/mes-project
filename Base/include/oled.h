/**
 *  @file oled.h
 *  @author Cristian Cristea
 *  @date October 20, 2023
 *
 *  @brief Header file for the OLED module
 **/


#ifndef OLED_H
#define OLED_H


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "spi.h"

#include <avr/io.h>

#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#define OLED_CHIP_SELECT          ( SPI_CS1 )

#define OLED_DATA_COMMAND_PORT    ( PORTD )
#define OLED_DATA_COMMAND_PIN     ( PIN0 )

#define OLED_RESET_PORT           ( PORTD )
#define OLED_RESET_PIN            ( PIN7 )

#define OLED_ENABLE_PORT          ( PORTD )
#define OLED_ENABLE_PIN           ( PIN6 )
 
#define OLED_READ_WRITE_PORT      ( PORTD )
#define OLED_READ_WRITE_PIN       ( PIN3 )


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

typedef enum OLED_ERROR_CODE
{
    OLED_OK              = 0x00,
    OLED_NULL_POINTER    = 0x01,
} oled_error_code_t;

typedef struct OLED_DEVICE
{
    // SPI device
    spi_t const * spiDevice;
} oled_device_t;

/**
 * @brief 24-bit RGB888 colour
 **/
typedef struct OLED_COLOUR
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} oled_colour_t;

/**
 * @brief 16-bit RGB565 colour
 **/
typedef uint16_t oled_packed_colour_t;


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                 Public API                                 //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Initializes the OLED device.
 * @param[in, out] device OLED device
 * @param[in] spiDevice SPI device
 * @return None
 **/
void OLED_Initialize(oled_device_t * const device, spi_t const * const spiDevice);

/**
 * @brief Enables drawing on the OLED.
 * @param[in] device OLED device
 * @return None
 **/
void OLED_StartWritingDisplay(oled_device_t const * const device);

/**
 * @brief Disables drawing on the OLED.
 * @param[in] device OLED device
 * @return None
 **/
void OLED_StopWritingDisplay(oled_device_t const * const device);

/**
 * @brief Sets the OLED's row start and stop positions.
 * @param[in] device OLED device
 * @param[in] min Row start position
 * @param[in] max Row stop position
 * @return None
 **/
void OLED_SetRowAddressBounds(oled_device_t const * const device, uint8_t const min, uint8_t const max);

/**
 * @brief Sets the OLED's column start and stop positions.
 * @param[in] device OLED device
 * @param[in] min Column start position
 * @param[in] max Column stop position
 * @return None
 **/
void OLED_SetColumnAddressBounds(oled_device_t const * const device, uint8_t const min, uint8_t const max);

/**
 * @brief Converts a 24-bit RGB colour into an RGB565 16-bit colour value.
 * @param[in] rgb RGB 24-bit colour
 * @return oled_packed_colour_t 16-bit colour value
 **/
oled_packed_colour_t OLED_ParseRGBtoPacked(oled_colour_t const rgb);

/**
 * @brief Converts an RGB565 16-bit colour value into a 24-bit RGB colour.
 * @param[in] rawData 16-bit colour value
 * @return oled_colour_t RGB 24-bit colour
 **/
oled_colour_t OLED_ParsePackedToRGB(oled_packed_colour_t const rawData);

/**
 * @brief Draws a pixel on the OLED at the current OLED's position.
 * @param[in] device OLED device
 * @param[in] colour Colour of the pixel
 * @return None
 **/
void OLED_SendColor(oled_device_t const * const device, oled_packed_colour_t const colour);

/**
 * @brief Sets the same colour on all pixels of the OLED.
 * @param[in] device OLED device
 * @param[in] colour Colour of the pixels
 * @return None
 **/
void OLED_SetBackground(oled_device_t const * const device, oled_colour_t const colour);


#endif // OLED_H
