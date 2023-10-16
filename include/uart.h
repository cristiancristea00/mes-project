/**
 *  @file uart.h
 *  @author Cristian Cristea
 *  @date October 15, 2023
 *
 *  @brief Header file for the UART module
 **/


#ifndef UART_H
#define	UART_H


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

typedef void (* uart_callback_t) (uint8_t const value);
typedef void (* uart_initialize_t) (uint32_t const baudRate);
typedef void (* uart_initialize_receive_t) (uint32_t const baudRate, uart_callback_t const receiveCallback);
typedef void (* uart_send_byte_t) (uint8_t const dataByte);
typedef void (* uart_send_data_t) (uint8_t const * const buffer, uint8_t const bufferSize);
typedef void (* uart_print_char_t) (char const character);
typedef void (* uart_print_t) (char const * const string);
typedef void (* uart_register_callback_t) (uart_callback_t const callback);


typedef struct UART
{
    uart_initialize_t Initialize;
    uart_initialize_receive_t InitializeWithReceive;
    uart_send_byte_t SendByte;
    uart_send_data_t SendData;
    uart_print_char_t PrintChar;
    uart_print_t Print;
    uart_register_callback_t RegisterCallback;
} uart_t;


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Modules                                   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

extern uart_t const uart0;

extern uart_t const uart1;


#endif // UART_H
