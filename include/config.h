/**
 *  @file config.h
 *  @author Cristian Cristea
 *  @date October 15, 2023
 *
 *  @brief Header file for the Config module
 **/


#ifndef CONFIG_H
#define CONFIG_H

/**
 * @brief This must be defined at the top to have other includes working.
 **/
#define F_CPU    ( 24000000UL )


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include <avr/interrupt.h>
#include <util/delay.h>

#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#define UART_PRINTF    // Enable/Disable printf functionality on UART
#define LOGGING        // Enable/Disable logging

#if defined ( __DEBUG ) || defined ( LOGGING )

#define LOG_DEBUG_PRINTF(STRING, ...)      do { printf("[DEBUG]: " STRING "\n\r", ##__VA_ARGS__); } while (false)
#define LOG_INFO_PRINTF(STRING, ...)       do { printf("[INFO]: " STRING "\n\r", ##__VA_ARGS__); } while (false)
#define LOG_WARNING_PRINTF(STRING, ...)    do { printf("[WARNING]: " STRING "\n\r", ##__VA_ARGS__); } while (false)
#define LOG_ERROR_PRINTF(STRING, ...)      do { printf("[ERROR]: " STRING "\n\r", ##__VA_ARGS__); } while (false)

#define LOG_DEBUG(STRING)                  do { PrintForLogging("[DEBUG]: " STRING "\n\r"); } while (false)
#define LOG_INFO(STRING)                   do { PrintForLogging("[INFO]: " STRING "\n\r"); } while (false)
#define LOG_WARNING(STRING)                do { PrintForLogging("[WARNING]: " STRING "\n\r"); } while (false)
#define LOG_ERROR(STRING)                  do { PrintForLogging("[ERROR]: " STRING "\n\r"); } while (false)

#else

#define LOG_DEBUG_PRINTF
#define LOG_INFO_PRINTF
#define LOG_WARNING_PRINTF
#define LOG_ERROR_PRINTF

#define LOG_DEBUG
#define LOG_INFO
#define LOG_WARNING
#define LOG_ERROR

#endif // LOGGING


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                 Public API                                 //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#if defined ( __DEBUG ) || defined ( LOGGING )

#include <stdio.h>

/**
 * @brief Wrapper function that sends the message 
 **/
void PrintForLogging(char const * const message);

#endif // LOGGING

/**
 * @brief Sets the CPU's clock frequency and the prescaler factor.
 * @param[in] frequency The desired frequency of the CPU's clock
 * @param[in] prescaler The prescaler factor
 **/
void SetClockFrequencyWithPrescaler(uint8_t const frequency, uint8_t const prescaler);

/**
 * @brief Sets the CPU's clock frequency.
 * @param[in] frequency The desired frequency of the CPU's clock
 **/
void SetClockFrequency(uint8_t const frequency);

/**
 * @brief Empty function that should be used in loops for better readability.
 **/
void TightLoopContents(void);


#endif // CONFIG_H
