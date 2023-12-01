/**
 *  @file utils.h
 *  @author Cristian Cristea
 *  @date October 16, 2023
 *
 *  @brief Header file for the Utils module
 **/
 
 
#ifndef UTILS_H
#define UTILS_H


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "config.h"

#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

// Unsigned integer cast

#define UINT8(X)     ((uint8_t)  (X))
#define UINT16(X)    ((uint16_t) (X))
#define UINT32(X)    ((uint32_t) (X))
#define UINT64(X)    ((uint64_t) (X))

// Signed integer cast

#define INT8(X)      ((int8_t)  (X))
#define INT16(X)     ((int16_t) (X))
#define INT32(X)     ((int32_t) (X))
#define INT64(X)     ((int64_t) (X))

// Interrupts macros

#define EnableGlobalInterrupts()     do { sei(); } while (false)
#define DisableGlobalInterrupts()    do { cli(); } while (false)

// Delay macros

#define PauseMiliseconds(MILIS)      do { _delay_ms((MILIS)); } while (false)
#define PauseMicroseconds(MICROS)    do { _delay_us((MICROS)); } while (false)

// Force inline for functions

#define INLINE           __attribute__((always_inline)) inline
#define STATIC_INLINE    __attribute__((always_inline)) static inline


#endif // UTILS_H
