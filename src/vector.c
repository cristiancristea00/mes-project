/**
 *  @file vector.c
 *  @author Cristian Cristea
 *  @date October 20, 2023
 *
 *  @brief Source file for the Vector module
 **/


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "vector.h"

#include "config.h"
#include "utils.h"

#include <stdbool.h>
#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

// Vector byte sizes

#define VECTOR_BYTE_BYTES           UINT8(1)
#define VECTOR_WORD_BYTES           UINT8(2)
#define VECTOR_DOUBLE_WORD_BYTES    UINT8(4)
#define VECTOR_QUAD_WORD_BYTES      UINT8(8)

// Helper macro for adding bytes to the vector

#define VECTOR_ADD(BYTE_NUMBER, TYPE)          do { buffer[idx + BYTE_NUMBER] = TYPE.bytes.byte ## BYTE_NUMBER; } while (false)
#define VECTOR_ADD_WORD(BYTE_NUMBER)           VECTOR_ADD(BYTE_NUMBER, word)
#define VECTOR_ADD_DOUBLE_WORD(BYTE_NUMBER)    VECTOR_ADD(BYTE_NUMBER, doubleWord)
#define VECTOR_ADD_QUAD_WORD(BYTE_NUMBER)      VECTOR_ADD(BYTE_NUMBER, quadWord)

// Helper macro for getting bytes from the vector

#define VECTOR_ELEM(BYTE_NUMBER)    bytes.byte ## BYTE_NUMBER = buffer[idx + BYTE_NUMBER]


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

typedef union WORD
{
    uint16_t value;

    struct
    {
        uint8_t byte0;
        uint8_t byte1;
    } bytes;
} word_t;

typedef union DOUBLE_WORD
{
    uint32_t value;

    struct
    {
        uint8_t byte0;
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
    } bytes;
} double_word_t;

typedef union QUAD_WORD
{
    uint64_t value;

    struct
    {
        uint8_t byte0;
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
        uint8_t byte4;
        uint8_t byte5;
        uint8_t byte6;
        uint8_t byte7;
    } bytes;
} quad_word_t;


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                            Private (static) API                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Checks if the vector can accommodate a certain amount of bytes. If not,
 *        it clears the vector.
 * @param[in] vector The vector to be checked
 * @param[in] numberBytes The number of bytes to be checked
 * @return None
 **/
static void Vector_CheckAvailableSpace(vector_t * const vector, uint8_t const numberBytes);


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Private (static) definitions                        //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

static void Vector_CheckAvailableSpace(vector_t * const vector, uint8_t const numberBytes)
{
    if ((vector->bufferSize + numberBytes) > VECTOR_MAX_BUFFER_SIZE)
    {
        LOG_WARNING_PRINTF("Failed to add %d bytes to vector. Clearing the vector...", numberBytes);
        Vector_Clear(vector);
    }
}


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Public definitions                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

__attribute__((always_inline)) inline void Vector_Initialize(vector_t * const vector)
{
    Vector_Clear(vector);

    return;
}

__attribute__((always_inline)) inline void Vector_Clear(vector_t * const vector)
{
    vector->bufferSize = 0;

    return;
}

void Vector_AddByte(vector_t * const vector, uint8_t const byteToAdd)
{
    Vector_CheckAvailableSpace(vector, VECTOR_BYTE_BYTES);

    vector->internalBuffer[vector->bufferSize] = byteToAdd;

    vector->bufferSize += VECTOR_BYTE_BYTES;

    return;
}

void Vector_AddWord(vector_t * const vector, uint16_t const wordToAdd)
{
    Vector_CheckAvailableSpace(vector, VECTOR_WORD_BYTES);

    word_t const word = { 
        .value = wordToAdd 
    };

    uint8_t * const buffer = vector->internalBuffer;
    uint8_t const idx = vector->bufferSize;

    VECTOR_ADD_WORD(0);
    VECTOR_ADD_WORD(1);

    vector->bufferSize += VECTOR_WORD_BYTES;

    return;
}

void Vector_AddDoubleWord(vector_t * const vector, uint32_t const doubleWordToAdd)
{
    Vector_CheckAvailableSpace(vector, VECTOR_DOUBLE_WORD_BYTES);

    double_word_t doubleWord = { .value = doubleWordToAdd };

    uint8_t * const buffer = vector->internalBuffer;
    uint8_t const idx = vector->bufferSize;

    VECTOR_ADD_DOUBLE_WORD(0);
    VECTOR_ADD_DOUBLE_WORD(1);
    VECTOR_ADD_DOUBLE_WORD(2);
    VECTOR_ADD_DOUBLE_WORD(3);

    vector->bufferSize += VECTOR_DOUBLE_WORD_BYTES;

    return;
}

void Vector_AddQuadWord(vector_t * const vector, uint64_t const quadWordToAdd)
{
    Vector_CheckAvailableSpace(vector, VECTOR_QUAD_WORD_BYTES);

    quad_word_t quadWord = { .value = quadWordToAdd };

    uint8_t * const buffer = vector->internalBuffer;
    uint8_t const idx = vector->bufferSize;

    VECTOR_ADD_QUAD_WORD(0);
    VECTOR_ADD_QUAD_WORD(1);
    VECTOR_ADD_QUAD_WORD(2);
    VECTOR_ADD_QUAD_WORD(3);
    VECTOR_ADD_QUAD_WORD(4);
    VECTOR_ADD_QUAD_WORD(5);
    VECTOR_ADD_QUAD_WORD(6);
    VECTOR_ADD_QUAD_WORD(7);

    vector->bufferSize += VECTOR_QUAD_WORD_BYTES;

    return;
}

uint8_t Vector_RemoveByte(vector_t * const vector)
{
    vector->bufferSize -= VECTOR_BYTE_BYTES;

    return vector->internalBuffer[vector->bufferSize];
}

uint16_t Vector_RemoveWord(vector_t * const vector)
{
    vector->bufferSize -= VECTOR_WORD_BYTES;

    uint8_t * const buffer = vector->internalBuffer;
    uint8_t const idx = vector->bufferSize;

    word_t const word = {
        .VECTOR_ELEM(0),
        .VECTOR_ELEM(1),
    };

    return word.value;
}

uint32_t Vector_RemoveDoubleWord(vector_t * const vector)
{
    vector->bufferSize -= VECTOR_DOUBLE_WORD_BYTES;

    uint8_t * const buffer = vector->internalBuffer;
    uint8_t const idx = vector->bufferSize;

    double_word_t const doubleWord = {
        .VECTOR_ELEM(0),
        .VECTOR_ELEM(1),
        .VECTOR_ELEM(2),
        .VECTOR_ELEM(3),
    };

    return doubleWord.value;
}

uint64_t Vector_RemoveQuadWord(vector_t * const vector)
{
    vector->bufferSize -= VECTOR_QUAD_WORD_BYTES;

    uint8_t * const buffer = vector->internalBuffer;
    uint8_t const idx = vector->bufferSize;

    quad_word_t const quadWord = {
        .VECTOR_ELEM(0),
        .VECTOR_ELEM(1),
        .VECTOR_ELEM(2),
        .VECTOR_ELEM(3),
        .VECTOR_ELEM(4),
        .VECTOR_ELEM(5),
        .VECTOR_ELEM(6),
        .VECTOR_ELEM(7),
    };

    return quadWord.value;
}

uint8_t Vector_FirstByte(vector_t const * const vector)
{
    return vector->internalBuffer[0];
}

uint8_t Vector_LastByte(vector_t const * const vector)
{
    return vector->internalBuffer[vector->bufferSize - VECTOR_BYTE_BYTES];
}

uint16_t Vector_FirstWord(vector_t const * const vector)
{
    uint8_t const * const buffer = vector->internalBuffer;
    uint8_t const idx = 0;

    word_t const word = {
        .VECTOR_ELEM(0),
        .VECTOR_ELEM(1),
    };

    return word.value;
}

uint16_t Vector_LastWord(vector_t const * const vector)
{
    uint8_t const * const buffer = vector->internalBuffer;
    uint8_t const idx = vector->bufferSize - VECTOR_WORD_BYTES;

    word_t const word = {
        .VECTOR_ELEM(0),
        .VECTOR_ELEM(1),
    };

    return word.value;
}

uint32_t Vector_FirstDoubleWord(vector_t const * const vector)
{
    uint8_t const * const buffer = vector->internalBuffer;
    uint8_t const idx = 0;

    double_word_t const doubleWord = {
        .VECTOR_ELEM(0),
        .VECTOR_ELEM(1),
        .VECTOR_ELEM(2),
        .VECTOR_ELEM(3),
    };

    return doubleWord.value;
}

uint32_t Vector_LastDoubleWord(vector_t const * const vector)
{
    uint8_t const * const buffer = vector->internalBuffer;
    uint8_t const idx = vector->bufferSize - VECTOR_DOUBLE_WORD_BYTES;

    double_word_t const doubleWord = {
        .VECTOR_ELEM(0),
        .VECTOR_ELEM(1),
        .VECTOR_ELEM(2),
        .VECTOR_ELEM(3),
    };

    return doubleWord.value;
}

uint64_t Vector_FirstQuadWord(vector_t const * const vector)
{
    uint8_t const * const buffer = vector->internalBuffer;
    uint8_t const idx = 0;

    quad_word_t const quadWord = {
        .VECTOR_ELEM(0),
        .VECTOR_ELEM(1),
        .VECTOR_ELEM(2),
        .VECTOR_ELEM(3),
        .VECTOR_ELEM(4),
        .VECTOR_ELEM(5),
        .VECTOR_ELEM(6),
        .VECTOR_ELEM(7),
    };

    return quadWord.value;
}

uint64_t Vector_LastQuadWord(vector_t const * const vector)
{
    uint8_t const * const buffer = vector->internalBuffer;
    uint8_t const idx = vector->bufferSize - VECTOR_QUAD_WORD_BYTES;

    quad_word_t const quadWord = {
        .VECTOR_ELEM(0),
        .VECTOR_ELEM(1),
        .VECTOR_ELEM(2),
        .VECTOR_ELEM(3),
        .VECTOR_ELEM(4),
        .VECTOR_ELEM(5),
        .VECTOR_ELEM(6),
        .VECTOR_ELEM(7),
    };

    return quadWord.value;
}