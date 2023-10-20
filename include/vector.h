/**
 *  @file vector.h
 *  @author Cristian Cristea
 *  @date October 20, 2023
 *
 *  @brief Header file for the Vector module
 **/


#ifndef VECTOR_H
#define	VECTOR_H


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "config.h"
#include "utils.h"

#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#define VECTOR_MAX_BUFFER_SIZE    UINT8(64)


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

typedef struct VECTOR
{
    // Internal buffer
    uint8_t internalBuffer[VECTOR_MAX_BUFFER_SIZE];

    // Current buffer size
    uint8_t bufferSize;
} vector_t;


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                 Public API                                 //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Sets the internal buffer to zeros.
 * @param[in, out] vector The vector to initialized
 * @return None
 **/
void Vector_Initialize(vector_t * const vector);

/**
 * @brief Clears the internal buffer.
 * @param vector The vector to be cleared
 * @return None
 **/
void Vector_Clear(vector_t * const vector);

/**
 * @brief Adds a byte to the vector.
 * @param[in, out] vector The vector to be added to
 * @param[in] byteToAdd The byte to be added
 * @return None
 **/
void Vector_AddByte(vector_t * const vector, uint8_t const byteToAdd);

/**
 * @brief Adds a word (2 bytes) to the vector.
 * @param[in, out] vector The vector to be added to
 * @param[in] wordToAdd The word to be added
 * @return None
 **/
void Vector_AddWord(vector_t * const vector, uint16_t const wordToAdd);

/**
 * @brief Adds a double word (4 bytes) to the vector.
 * @param[in, out] vector The vector to be added to
 * @param[in] doubleWordToAdd The double word to be added
 * @return None
 **/
void Vector_AddDoubleWord(vector_t * const vector, uint32_t const doubleWordToAdd);

/**
 * @brief Adds a quad word (8 bytes) to the vector.
 * @param[in, out] vector The vector to be added to
 * @param[in] quadWordToAdd The quad word to be added
 * @return None
 **/
void Vector_AddQuadWord(vector_t * const vector, uint64_t const quadWordToAdd);

/**
 * @brief Removes a byte from the vector and returns it.
 * @param[in, out] vector The vector to be removed from
 * @return uint8_t The byte removed from the vector
 **/
uint8_t Vector_RemoveByte(vector_t * const vector);

/**
 * @brief Removes a word (2 bytes) from the vector and returns it.
 * @param[in, out] vector The vector to be removed from
 * @return uint16_t The word removed from the vector
 **/
uint16_t Vector_RemoveWord(vector_t * const vector);

/**
 * @brief Removes a double word (4 bytes) from the vector and returns it.
 * @param[in, out] vector The vector to be removed from
 * @return uint32_t The double word removed from the vector
 **/
uint32_t Vector_RemoveDoubleWord(vector_t * const vector);

/**
 * @brief Removes a quad word (8 bytes) from the vector and returns it.
 * @param[in, out] vector The vector to be removed from
 * @return uint64_t The quad word removed from the vector
 **/
uint64_t Vector_RemoveQuadWord(vector_t * const vector);

/**
 * @brief Returns the first byte of the vector.
 * @param[in] vector The vector to be read from
 * @return uint8_t The first byte of the vector
 **/
uint8_t Vector_FirstByte(vector_t const * const vector);

/**
 * @brief Returns the last byte of the vector.
 * @param[in] vector The vector to be read from
 * @return uint8_t The last byte of the vector
 **/
uint8_t Vector_LastByte(vector_t const * const vector);

/**
 * @brief Returns the first word (2 bytes) of the vector.
 * @param[in] vector The vector to be read from
 * @return uint16_t The first word of the vector
 **/
uint16_t Vector_FirstWord(vector_t const * const vector);

/**
 * @brief Returns the last word (2 bytes) of the vector.
 * @param[in] vector The vector to be read from
 * @return uint16_t The last word of the vector
 **/
uint16_t Vector_LastWord(vector_t const * const vector);

/**
 * @brief Returns the first double word (4 bytes) of the vector.
 * @param[in] vector The vector to be read from
 * @return uint32_t The first double word of the vector
 **/
uint32_t Vector_FirstDoubleWord(vector_t const * const vector);

/**
 * @brief Returns the last double word (4 bytes) of the vector.
 * @param[in] vector The vector to be read from
 * @return uint32_t The last double word of the vector
 **/
uint32_t Vector_LastDoubleWord(vector_t const * const vector);

/**
 * @brief Returns the first quad word (8 bytes) of the vector.
 * @param[in] vector The vector to be read from
 * @return uint64_t The first quad word of the vector
 **/
uint64_t Vector_FirstQuadWord(vector_t const * const vector);

/**
 * @brief Returns the last quad word (8 bytes) of the vector.
 * @param[in] vector The vector to be read from
 * @return uint64_t The last quad word of the vector
 **/
uint64_t Vector_LastQuadWord(vector_t const * const vector);


#endif // VECTOR_H