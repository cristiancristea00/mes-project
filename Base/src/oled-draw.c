/**
 *  @file oled-draw.c
 *  @author Cristian Cristea
 *  @date October 20, 2023
 *
 *  @brief Source file for the OLED drawing module
 **/


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "oled-draw.h"

#include "config.h"
#include "oled.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#pragma switch speed


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#define OLED_DRAW_MAX_DIMENSION    UINT8(95)


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                            Private (static) API                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Draws a point on the OLED screen.
 * @param[in] device OLED device
 * @param[in] x Column position
 * @param[in] y Row position
 * @param[in] colour Colour of the point
 * @return None
 **/
__attribute__((always_inline)) inline void OLED_DRAW_Point(oled_device_t const * const device, uint8_t const x, uint8_t const y, oled_packed_colour_t const colour);

/**
 * @brief Draws a line on the OLED screen.
 * @param[in] device OLED device
 * @param[in] start Start position
 * @param[in] end End position
 * @param[in] width Width of the line
 * @param[in] colour Colour of the line
 * @return None
 **/
__attribute__((always_inline)) inline void OLED_DRAW_Line(oled_device_t const * const device, oled_point_t const start, oled_point_t const end, uint8_t const width, oled_packed_colour_t const colour);

/**
 * @brief Draws a rectangle on the OLED screen.
 * @param[in] device OLED device
 * @param[in] start Start position (top left corner)
 * @param[in] end End position (bottom right corner)
 * @param[in] width Width of the border
 * @param[in] colour Colour of the border
 * @return None
 **/
__attribute__((always_inline)) inline void OLED_DRAW_Rectangle(oled_device_t const * const device, oled_point_t const start, oled_point_t const end, uint8_t const width, oled_packed_colour_t const colour);

/**
 * @brief Draws a filled rectangle on the OLED screen.
 * @param[in] device OLED device
 * @param[in] startPoint Start position (top left corner)
 * @param[in] endPoint End position (bottom right corner)
 * @param[in] colour Colour of the rectangle
 * @return None
 **/
__attribute__((always_inline)) inline void OLED_DRAW_FilledRectangle(oled_device_t const * const device, oled_point_t const startPoint, oled_point_t const endPoint, oled_packed_colour_t const colour);

/**
 * @brief Draws a circle on the OLED screen.
 * @param[in] device OLED device
 * @param[in] center Center position of the circle
 * @param[in] radius Radius of the circle
 * @param[in] width Width of the border
 * @param[in] colour Colour of the border
 * @return None
 **/
__attribute__((always_inline)) inline void OLED_DRAW_Circle(oled_device_t const * const device, oled_point_t const center, uint8_t const radius, uint8_t const width, oled_packed_colour_t const colour);

/**
 * @brief Draws a disc on the OLED screen.
 * @param[in] device OLED device
 * @param[in] center Center position of the disc
 * @param[in] radius Radius of the disc
 * @param[in] colour Colour of the disc
 * @return None
 **/
__attribute__((always_inline)) inline void OLED_DRAW_Disc(oled_device_t const * const device, oled_point_t const center, uint8_t const radius, oled_packed_colour_t const colour);

/**
 * @brief Draws a 24-bit RGB colour bitmap on the OLED screen.
 * @param[in] device OLED device
 * @param[in] bitmap Bitmap to draw
 * @param[in] xSize Horizontal size of the bitmap
 * @param[in] ySize Vertical size of the bitmap
 * @param[in] start Start position (top left corner)
 * @return None
 **/
__attribute__((always_inline)) inline void OLED_DRAW_Bitmap(oled_device_t const * const device, oled_packed_colour_t const * const bitmap, uint8_t const xSize, uint8_t const ySize, oled_point_t const start);

/**
 * @brief Draws a character on the OLED screen.
 * @param[in] device OLED device
 * @param[in] start Start position (top left corner)
 * @param[in] xScale Horizontal scale of the character
 * @param[in] yScale Vertical scale of the character
 * @param[in] character Character to draw
 * @param[in] colour Colour of the character
 * @return None
 **/
__attribute__((always_inline)) inline void OLED_DRAW_Character(oled_device_t const * const device, oled_point_t const start, uint8_t const xScale, uint8_t const yScale, uint8_t const character, oled_packed_colour_t const colour);

/**
 * @brief Draws a string on the OLED screen.
 * @param[in] device OLED device
 * @param[in] start Start position (top left corner)
 * @param[in] xScale Horizontal scale of the characters
 * @param[in] yScale Vertical scale of the characters
 * @param[in] data String to draw
 * @param[in] colour Colour of the characters
 * @return None
 **/
__attribute__((always_inline)) inline void OLED_DRAW_String(oled_device_t const * const device, oled_point_t const start, uint8_t const xScale, uint8_t const yScale, uint8_t const * const data, oled_packed_colour_t const colour);

/**
 * @brief Unified API to draw a point on the OLED screen.
 * @param[in] device OLED device
 * @param[in] shape Point to draw
 * @return None
 **/
void OLED_DrawPoint(oled_device_t const * const device, oled_shape_t const * const shape);

/**
 * @brief Unified API to draw a line on the OLED screen.
 * @param[in] device OLED device
 * @param[in] shape Line to draw
 * @return None
 **/
void OLED_DrawLine(oled_device_t const * const device, oled_shape_t const * const shape);

/**
 * @brief Unified API to draw a rectangle on the OLED screen.
 * @param[in] device OLED device
 * @param[in] shape Rectangle to draw
 * @return None
 **/
void OLED_DrawRectangle(oled_device_t const * const device, oled_shape_t const * const shape);

/**
 * @brief Unified API to draw a filled rectangle on the OLED screen.
 * @param[in] device OLED device
 * @param[in] shape Filled rectangle to draw
 * @return None
 **/
void OLED_DrawFilledRectangle(oled_device_t const * const device, oled_shape_t const * const shape);

/**
 * @brief Unified API to draw a circle on the OLED screen.
 * @param[in] device OLED device
 * @param[in] shape Circle to draw
 * @return None
 **/
void OLED_DrawCircle(oled_device_t const * const device, oled_shape_t const * const shape);

/**
 * @brief Unified API to draw a disc on the OLED screen.
 * @param[in] device OLED device
 * @param[in] shape Disc to draw
 * @return None
 **/
void OLED_DrawDisc(oled_device_t const * const device, oled_shape_t const * const shape);

/**
 * @brief Unified API to draw a bitmap on the OLED screen.
 * @param[in] device OLED device
 * @param[in] shape Bitmap to draw
 * @return None
 **/
void OLED_DrawBitmap(oled_device_t const * const device, oled_shape_t const * const shape);

/**
 * @brief Unified API to draw a character on the OLED screen.
 * @param[in] device OLED device
 * @param[in] shape Character to draw
 * @return None
 **/
void OLED_DrawCharacter(oled_device_t const * const device, oled_shape_t const * const shape);

/**
 * @brief Unified API to draw a string on the OLED screen.
 * @param[in] device OLED device
 * @param[in] shape String to draw
 * @return None
 **/
void OLED_DrawString(oled_device_t const * const device, oled_shape_t const * const shape);

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Private (static) definitions                        //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

__attribute__((always_inline)) inline void OLED_DRAW_Point(oled_device_t const * const device, uint8_t const x, uint8_t const y, oled_packed_colour_t const colour)
{
    if ((x > OLED_DRAW_MAX_DIMENSION) || (y > OLED_DRAW_MAX_DIMENSION))
    {
        return;
    }

    OLED_SetColumnAddressBounds(device, x, x);
    OLED_SetRowAddressBounds(device, y, y);

    OLED_StartWritingDisplay(device);
    OLED_SendColor(device, colour);
    OLED_StopWritingDisplay(device);

    return;
}

__attribute__((always_inline)) inline void OLED_DRAW_Line(oled_device_t const * const device, oled_point_t const start, oled_point_t const end, uint8_t const width, oled_packed_colour_t const colour)
{
    uint8_t const lineWidth = (width > 1) ? width : 1;

    int8_t const length_x = abs(end.x - start.x);
    int8_t const length_y = abs(end.y - start.y);

    int8_t const delta_x = (start.x < end.x) ? 1 : -1;
    int8_t const delta_y = (start.y < end.y) ? 1 : -1;

    int8_t error = (length_x > length_y) ? length_x : -length_y;
    error >>= 1;

    int8_t x = start.x;
    int8_t y = start.y;

    int8_t currentError = 0;

    while (true)
    {
        if (lineWidth <= 1)
        {
            OLED_DRAW_Point(device, x, y, colour);
        }
        else
        {
            oled_point_t const center = { x, y };
            OLED_DRAW_Disc(device, center, lineWidth >> 1, colour);
        }

        if ((x == end.x) && (y == end.y))
        {
            break;
        }

        currentError = error;

        if (currentError > -length_x)
        {
            error -= length_y;
            x += delta_x;
        }
        if (currentError < length_y)
        {
            error += length_x;
            y += delta_y;
        }
    }

    return;
}

__attribute__((always_inline)) inline void OLED_DRAW_Rectangle(oled_device_t const * const device, oled_point_t const start, oled_point_t const end, uint8_t const width, oled_packed_colour_t const colour)
{

    oled_point_t const topLeft     = { start.x, start.y };
    oled_point_t const topRight    = { end.x, start.y };
    oled_point_t const bottomLeft  = { start.x, end.y };
    oled_point_t const bottomRight = { end.x, end.y };

    OLED_DRAW_Line(device, topLeft, topRight, width, colour);
    OLED_DRAW_Line(device, topRight, bottomRight, width, colour);
    OLED_DRAW_Line(device, bottomRight, bottomLeft, width, colour);
    OLED_DRAW_Line(device, bottomLeft, topLeft, width, colour);

    return;
}

__attribute__((always_inline)) inline void OLED_DRAW_FilledRectangle(oled_device_t const * const device, oled_point_t const start, oled_point_t const end, oled_packed_colour_t const colour)
{
    OLED_SetColumnAddressBounds(device, start.x, end.x);
    OLED_SetRowAddressBounds(device, start.y, end.y);

    OLED_StartWritingDisplay(device);

    for (uint8_t y = start.y; y <= end.y; ++y)
    {
        for (uint8_t x = start.x; x <= end.x; ++x)
        {
            OLED_SendColor(device, colour);
        }
    }

    OLED_StopWritingDisplay(device);

    return;
}

__attribute__((always_inline)) inline void OLED_DRAW_Circle(oled_device_t const * const device, oled_point_t const center, uint8_t const radius, uint8_t const width, oled_packed_colour_t const colour)
{
    int8_t x;
    int8_t y;
    int8_t determinant;

    uint8_t circleWidth = width;
    uint8_t circleRadius = radius + (circleWidth >> 1);

    while (circleWidth != 0)
    {
        --circleWidth;

        x = 0;
        y = circleRadius;
        determinant = 0;

        while (y >= x)
        {
            OLED_DRAW_Point(device, center.x + x, center.y + y, colour);
            OLED_DRAW_Point(device, center.x + x, center.y - y, colour);
            OLED_DRAW_Point(device, center.x - x, center.y + y, colour);
            OLED_DRAW_Point(device, center.x - x, center.y - y, colour);
            OLED_DRAW_Point(device, center.x + y, center.y + x, colour);
            OLED_DRAW_Point(device, center.x + y, center.y - x, colour);
            OLED_DRAW_Point(device, center.x - y, center.y + x, colour);
            OLED_DRAW_Point(device, center.x - y, center.y - x, colour);

            determinant += (2 * x + 1);
            ++x;

            if (determinant >= 0)
            {

                determinant += (-2 * y + 1);
                --y;
            }
        }
        --circleRadius;
    }

    return;
}

__attribute__((always_inline)) inline void OLED_DRAW_Disc(oled_device_t const * const device, oled_point_t const center, uint8_t const radius, oled_packed_colour_t const colour)
{
    int8_t y = ((radius > 1) ? radius : 1) + 1;
    int8_t x = 0;
    int8_t determinant = 0;

    int8_t xCurr = 0;

    while (y >= x)
    {
        determinant += (2 * x + 1);
        ++x;

        if (determinant >= 0)
        {
            for (xCurr = xCurr; xCurr < x; ++xCurr)
            {
                for (int8_t yCurr = xCurr; yCurr < y; ++yCurr)
                {
                    OLED_DRAW_Point(device, center.x + xCurr, center.y + yCurr, colour);
                    OLED_DRAW_Point(device, center.x + xCurr, center.y - yCurr, colour);
                    OLED_DRAW_Point(device, center.x - xCurr, center.y + yCurr, colour);
                    OLED_DRAW_Point(device, center.x - xCurr, center.y - yCurr, colour);
                    OLED_DRAW_Point(device, center.x + yCurr, center.y + xCurr, colour);
                    OLED_DRAW_Point(device, center.x + yCurr, center.y - xCurr, colour);
                    OLED_DRAW_Point(device, center.x - yCurr, center.y + xCurr, colour);
                    OLED_DRAW_Point(device, center.x - yCurr, center.y - xCurr, colour);
                }
            }

            determinant += (-2 * y + 1);
            --y;
        }
    }

    return;
}

__attribute__((always_inline)) inline void OLED_DRAW_Bitmap(oled_device_t const * const device, oled_packed_colour_t const * const bitmap, uint8_t const xSize, uint8_t const ySize, oled_point_t const start)
{
    OLED_SetColumnAddressBounds(device, start.x, start.x + xSize - 1);
    OLED_SetRowAddressBounds(device, start.y, start.y + ySize - 1);

    OLED_StartWritingDisplay(device);

    for (uint8_t y = 0; y < ySize; ++y)
    {
        for (uint8_t x = 0; x < xSize; ++x)
        {
            OLED_SendColor(device, bitmap[y * xSize + x]);
        }
    }

    OLED_StopWritingDisplay(device);

    return;
}

__attribute__((always_inline)) inline void OLED_DRAW_Character(oled_device_t const * const device, oled_point_t const start, uint8_t const xScale, uint8_t const yScale, uint8_t const character, oled_packed_colour_t const colour)
{
    static uint8_t const font[] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFA, 0x00, 0x00, 0x00, 0xE0, 0x00, 0xE0, 0x00,  /*  ' ' AND ! AND "  */
        0x28, 0xFE, 0x28, 0xFE, 0x28,                                                              /*  #                */
        0x24, 0x54, 0xFE, 0x54, 0x48, 0xC4, 0xC8, 0x10, 0x26, 0x46, 0x6C, 0x92, 0x6A, 0x04, 0x0A,  /*  $ AND % AND &    */
        0x00, 0x10, 0xE0, 0xC0, 0x00, 0x00, 0x38, 0x44, 0x82, 0x00, 0x00, 0x82, 0x44, 0x38, 0x00,  /*  ' AND ( AND )    */
        0x54, 0x38, 0xFE, 0x38, 0x54, 0x10, 0x10, 0x7C, 0x10, 0x10, 0x00, 0x00, 0x0E, 0x0C, 0x00,  /*  * AND + AND ,    */
        0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x06, 0x06, 0x00, 0x04, 0x08, 0x10, 0x20, 0x40,  /*  - AND . AND /    */
        0x7C, 0x8A, 0x92, 0xA2, 0x7C, 0x00, 0x42, 0xFE, 0x02, 0x00, 0x4E, 0x92, 0x92, 0x92, 0x62,  /*  0 AND 1 AND 2    */
        0x84, 0x82, 0x92, 0xB2, 0xCC, 0x18, 0x28, 0x48, 0xFE, 0x08, 0xE4, 0xA2, 0xA2, 0xA2, 0x9C,  /*  3 AND 4 AND 5    */
        0x3C, 0x52, 0x92, 0x92, 0x8C, 0x82, 0x84, 0x88, 0x90, 0xE0, 0x6C, 0x92, 0x92, 0x92, 0x6C,  /*  6 AND 7 AND 8    */
        0x62, 0x92, 0x92, 0x94, 0x78, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x02, 0x2C, 0x00, 0x00,  /*  9 AND : AND ;    */
        0x00, 0x10, 0x28, 0x44, 0x82, 0x28, 0x28, 0x28, 0x28, 0x28, 0x00, 0x82, 0x44, 0x28, 0x10,  /*  < AND = AND >    */
        0x40, 0x80, 0x9A, 0x90, 0x60, 0x7C, 0x82, 0xBA, 0x9A, 0x72, 0x3E, 0x48, 0x88, 0x48, 0x3E,  /*  ? AND @ AND A    */
        0xFE, 0x92, 0x92, 0x92, 0x6C, 0x7C, 0x82, 0x82, 0x82, 0x44, 0xFE, 0x82, 0x82, 0x82, 0x7C,  /*  B AND C AND D    */
        0xFE, 0x92, 0x92, 0x92, 0x82, 0xFE, 0x90, 0x90, 0x90, 0x80, 0x7C, 0x82, 0x82, 0x8A, 0xCE,  /*  E AND F AND G    */
        0xFE, 0x10, 0x10, 0x10, 0xFE, 0x00, 0x82, 0xFE, 0x82, 0x00, 0x04, 0x02, 0x82, 0xFC, 0x80,  /*  H AND I AND J    */
        0xFE, 0x10, 0x28, 0x44, 0x82, 0xFE, 0x02, 0x02, 0x02, 0x02, 0xFE, 0x40, 0x38, 0x40, 0xFE,  /*  K AND L AND M    */
        0xFE, 0x20, 0x10, 0x08, 0xFE, 0x7C, 0x82, 0x82, 0x82, 0x7C, 0xFE, 0x90, 0x90, 0x90, 0x60,  /*  N AND O AND P    */
        0x7C, 0x82, 0x8A, 0x84, 0x7A, 0xFE, 0x90, 0x98, 0x94, 0x62, 0x64, 0x92, 0x92, 0x92, 0x4C,  /*  Q AND R AND S    */
        0xC0, 0x80, 0xFE, 0x80, 0xC0, 0xFC, 0x02, 0x02, 0x02, 0xFC, 0xF8, 0x04, 0x02, 0x04, 0xF8,  /*  T AND U AND V    */
        0xFC, 0x02, 0x1C, 0x02, 0xFC, 0xC6, 0x28, 0x10, 0x28, 0xC6, 0xC0, 0x20, 0x1E, 0x20, 0xC0,  /*  W AND X AND Y    */
        0x86, 0x9A, 0x92, 0xB2, 0xC2, 0x00, 0xFE, 0x82, 0x82, 0x82, 0x40, 0x20, 0x10, 0x08, 0x04,  /*  Z AND [ AND \    */
        0x00, 0x82, 0x82, 0x82, 0xFE, 0x20, 0x40, 0x80, 0x40, 0x20, 0x02, 0x02, 0x02, 0x02, 0x02,  /*  ] AND ^ AND _    */
        0x00, 0xC0, 0xE0, 0x10, 0x00, 0x04, 0x2A, 0x2A, 0x1C, 0x02, 0xFE, 0x14, 0x22, 0x22, 0x1C,  /*  ` AND a AND b    */
        0x1C, 0x22, 0x22, 0x22, 0x14, 0x1C, 0x22, 0x22, 0x14, 0xFE, 0x1C, 0x2A, 0x2A, 0x2A, 0x18,  /*  c AND d AND e    */
        0x00, 0x10, 0x7E, 0x90, 0x40, 0x30, 0x4A, 0x4A, 0x52, 0x3C, 0xFE, 0x10, 0x20, 0x20, 0x1E,  /*  f AND g AND h    */
        0x00, 0x22, 0xBE, 0x02, 0x00, 0x04, 0x02, 0x02, 0xBC, 0x00, 0xFE, 0x08, 0x14, 0x22, 0x00,  /*  i AND j AND k    */
        0x00, 0x82, 0xFE, 0x02, 0x00, 0x3E, 0x20, 0x1E, 0x20, 0x1E, 0x3E, 0x10, 0x20, 0x20, 0x1E,  /*  l AND m AND n    */
        0x1C, 0x22, 0x22, 0x22, 0x1C, 0x3E, 0x18, 0x24, 0x24, 0x18, 0x18, 0x24, 0x24, 0x18, 0x3E,  /*  o AND p AND q    */
        0x3E, 0x10, 0x20, 0x20, 0x10, 0x12, 0x2A, 0x2A, 0x2A, 0x24, 0x20, 0x20, 0xFC, 0x22, 0x24,  /*  r AND s AND t    */
        0x3C, 0x02, 0x02, 0x04, 0x3E, 0x38, 0x04, 0x02, 0x04, 0x38, 0x3C, 0x02, 0x0C, 0x02, 0x3C,  /*  u AND v AND w    */
        0x22, 0x14, 0x08, 0x14, 0x22, 0x32, 0x0A, 0x0A, 0x0A, 0x3C, 0x22, 0x26, 0x2A, 0x32, 0x22,  /*  x AND y AND z    */
        0x00, 0x10, 0x6C, 0x82, 0x00, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x82, 0x6C, 0x10, 0x00,  /*  { AND | AND }    */
        0x40, 0x80, 0x40, 0x20, 0x40                                                               /*  ~                */
    };

    oled_point_t drawStart;
    oled_point_t drawEnd;

    uint8_t const * fontData = &font[(character - ' ') * OLED_DRAW_FONT_WIDTH];

    for (uint16_t x = 0; x < OLED_DRAW_FONT_WIDTH * xScale; x += xScale)
    {
        uint8_t currentCharByte = *fontData++;
        for (uint16_t y = OLED_DRAW_FONT_HEIGHT * yScale; y > 0; y -= yScale)
        {
            if (currentCharByte & 0x01)
            {
                drawStart.x = start.x + x;
                drawStart.y = start.y + y;

                drawEnd.x = drawStart.x + xScale - 1;
                drawEnd.y = drawStart.y + yScale - 1;

                OLED_DRAW_FilledRectangle(device, drawStart, drawEnd, colour);
            }
            currentCharByte >>= 1;
        }
    }

    return;
}

__attribute__((always_inline)) inline void OLED_DRAW_String(oled_device_t const * const device, oled_point_t const start, uint8_t const xScale, uint8_t const yScale, uint8_t const * const data, oled_packed_colour_t const colour)
{
    uint8_t character = '\0';
    uint8_t const * currentStringPosition = data;
    oled_point_t currentPosition = start;

    while (true)
    {
        character = *currentStringPosition++;

        if (character == '\0')
        {
            break;
        }

        OLED_DRAW_Character(device, currentPosition, xScale, yScale, character, colour);
        currentPosition.x += OLED_DRAW_FONT_WIDTH * xScale + 1;
    }

    return;
}

void OLED_DrawPoint(oled_device_t const * const device, oled_shape_t const * const shape)
{
    OLED_DRAW_Point(device, shape->parameters.point.point.x, shape->parameters.point.point.y, shape->colour);

    return;
}

void OLED_DrawLine(oled_device_t const * const device, oled_shape_t const * const shape)
{
    OLED_DRAW_Line(device, shape->parameters.line.start, shape->parameters.line.end, shape->parameters.line.width, shape->colour);

    return;
}

void OLED_DrawRectangle(oled_device_t const * const device, oled_shape_t const * const shape)
{
    OLED_DRAW_Rectangle(device, shape->parameters.rectangle.start, shape->parameters.rectangle.end, shape->parameters.rectangle.width, shape->colour);

    return;
}

void OLED_DrawFilledRectangle(oled_device_t const * const device, oled_shape_t const * const shape)
{
    OLED_DRAW_FilledRectangle(device, shape->parameters.filled_rectangle.start, shape->parameters.filled_rectangle.end, shape->colour);

    return;
}

void OLED_DrawCircle(oled_device_t const * const device, oled_shape_t const * const shape)
{
    OLED_DRAW_Circle(device, shape->parameters.circle.center, shape->parameters.circle.radius, shape->parameters.circle.width, shape->colour);

    return;
}

void OLED_DrawDisc(oled_device_t const * const device, oled_shape_t const * const shape)
{
    OLED_DRAW_Disc(device, shape->parameters.disc.center, shape->parameters.disc.radius, shape->colour);

    return;
}

void OLED_DrawBitmap(oled_device_t const * const device, oled_shape_t const * const shape)
{
    OLED_DRAW_Bitmap(device, shape->parameters.bitmap.data, shape->parameters.bitmap.size_x, shape->parameters.bitmap.size_y, shape->parameters.bitmap.start);

    return;
}

void OLED_DrawCharacter(oled_device_t const * const device, oled_shape_t const * const shape)
{
    OLED_DRAW_Character(device, shape->parameters.character.start, shape->parameters.character.scale_x, shape->parameters.character.scale_y, shape->parameters.character.character, shape->colour);

    return;
}

void OLED_DrawString(oled_device_t const * const device, oled_shape_t const * const shape)
{
    OLED_DRAW_String(device, shape->parameters.string.start, shape->parameters.string.scale_x, shape->parameters.string.scale_y, shape->parameters.string.data, shape->colour);

    return;
}


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Public definitions                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

void OLED_SetShape(oled_shape_t * const shape, oled_shape_type_t const type, oled_shape_parameters_t const * const parameters, oled_colour_t const colour)
{
    shape->type = type;
    shape->colour = OLED_ParseRGBtoPacked(colour);

    switch (type)
    {
        case OLED_SHAPE_POINT:
            shape->parameters.point = parameters->point;
            shape->Draw = OLED_DrawPoint;
            break;
        case OLED_SHAPE_LINE:
            shape->parameters.line = parameters->line;
            shape->Draw = OLED_DrawLine;
            break;
        case OLED_SHAPE_RECTANGLE:
            shape->parameters.rectangle = parameters->rectangle;
            shape->Draw = OLED_DrawRectangle;
            break;
        case OLED_SHAPE_FILLED_RECTANGLE:
            shape->parameters.filled_rectangle = parameters->filled_rectangle;
            shape->Draw = OLED_DrawFilledRectangle;
            break;
        case OLED_SHAPE_CIRCLE:
            shape->parameters.circle = parameters->circle;
            shape->Draw = OLED_DrawCircle;
            break;
        case OLED_SHAPE_DISC:
            shape->parameters.disc = parameters->disc;
            shape->Draw = OLED_DrawDisc;
            break;
        case OLED_SHAPE_BITMAP:
            shape->parameters.bitmap = parameters->bitmap;
            shape->Draw = OLED_DrawBitmap;
            break;
        case OLED_SHAPE_CHARACTER:
            shape->parameters.character = parameters->character;
            shape->Draw = OLED_DrawCharacter;
            break;
        case OLED_SHAPE_STRING:
            shape->parameters.string = parameters->string;
            shape->Draw = OLED_DrawString;
            break;
        default:
            LOG_ERROR("Invalid shape in shape creation");
            break;
    }

    return;
}