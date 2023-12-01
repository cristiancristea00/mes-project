/**
 *  @file oled-draw.h
 *  @author Cristian Cristea
 *  @date October 20, 2023
 *
 *  @brief Header file for the OLED module
 **/


#ifndef OLED_DRAW_H
#define OLED_DRAW_H

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "oled.h"

#include "utils.h"


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#define OLED_DRAW_FONT_WIDTH       UINT8(5)
#define OLED_DRAW_FONT_HEIGHT      UINT8(8)


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

typedef enum OLED_SHAPE_TYPE
{
    OLED_SHAPE_POINT            = 0x00,
    OLED_SHAPE_LINE             = 0x01,
    OLED_SHAPE_RECTANGLE        = 0x02,
    OLED_SHAPE_FILLED_RECTANGLE = 0x03,
    OLED_SHAPE_DISC             = 0x04,
    OLED_SHAPE_CIRCLE           = 0x05,
    OLED_SHAPE_BITMAP           = 0x06,
    OLED_SHAPE_CHARACTER        = 0x07,
    OLED_SHAPE_STRING           = 0x08,
} oled_shape_type_t;

typedef struct OLED_POINT
{
    uint8_t x;
    uint8_t y;
} oled_point_t;

typedef union OLED_SHAPE_PARAMETERS
{
    struct {
        oled_point_t point;
    } point;

    struct {
        oled_point_t start;
        oled_point_t end;
        uint8_t width;
    } line;

    struct {
        oled_point_t start;
        oled_point_t end;
        uint8_t width;
    } rectangle;

    struct {
        oled_point_t start;
        oled_point_t end;
    } filled_rectangle;

    struct {
        oled_point_t center;
        uint8_t radius;
        uint8_t width;
    } circle;

    struct {
        oled_point_t center;
        uint8_t radius;
    } disc;

    struct {
        oled_point_t start;
        uint8_t size_x;
        uint8_t size_y;
        oled_packed_colour_t const * data;
    } bitmap;

    struct {
        oled_point_t start;
        uint8_t scale_x;
        uint8_t scale_y;
        uint8_t character;
    } character;

    struct {
        oled_point_t start;
        uint8_t scale_x;
        uint8_t scale_y;
        uint8_t const * data;
    } string;
} oled_shape_parameters_t;

typedef struct OLED_SHAPE
{
    oled_packed_colour_t colour;
    oled_shape_type_t type;
    oled_shape_parameters_t parameters;
    void (* Draw) (oled_device_t const * const device, struct OLED_SHAPE const * const shape);
} oled_shape_t;

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                 Public API                                 //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Initializes the OLED shape with its corresponding type, parameters,
 *        colour and drawing function.
 * @param[in, out] shape The shape to initialize
 * @param[in] type Shape type
 * @param[in] parameters Shape's parameters
 * @param[in] colour Shape's colour
 * @return None
 **/
void OLED_SetShape(oled_shape_t * const shape, oled_shape_type_t const type, oled_shape_parameters_t const * const parameters, oled_colour_t const colour);


#endif // OLED_DRAW_H
