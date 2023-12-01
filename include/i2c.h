/**
 *  @file i2c.h
 *  @author Cristian Cristea
 *  @date October 16, 2023
 *
 *  @brief Header file for the I2C module
 **/


#ifndef I2C_H
#define I2C_H


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "utils.h"

#include <stdint.h>
#include <stdbool.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#define I2C_ADDRESS_MIN    UINT8(0x00)
#define I2C_ADDRESS_MAX    UINT8(0x7F)


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

typedef enum I2C_ERROR_CODE
{
    I2C_OK                  = 0x00,
    I2C_NULL_POINTER        = 0x01,
    I2C_INVALID_ADDRESS     = 0x02,
    I2C_NACK_OF_ADDRESS     = 0x03,
    I2C_COMMUNICATION_ERROR = 0x04,
} i2c_error_code_t;

typedef enum I2C_MODE
{
    I2C_MODE_STANDARD  = 0x65,
    I2C_MODE_FAST      = 0x15,
    I2C_MODE_FAST_PLUS = 0x06,
} i2c_mode_t;

typedef void (* i2c_initialize_t) (i2c_mode_t const mode);
typedef i2c_error_code_t (* i2c_send_data_t) (uint8_t const address, uint8_t const * const dataForSend, uint8_t const initialLength);
typedef i2c_error_code_t (* i2c_receive_data_t) (uint8_t const address, uint8_t * dataForReceive, uint8_t const initialLength);
typedef void (* i2c_end_transaction_t) (void);
typedef bool (* i2c_is_client_available_t) (uint8_t const clientAddress);

/**
 * @brief Object struct for the I2C module
 **/
typedef struct I2C
{
    i2c_initialize_t Initialize;
    i2c_send_data_t SendData;
    i2c_receive_data_t ReceiveData;
    i2c_end_transaction_t EndTransaction;
    i2c_is_client_available_t IsClientAvailable;
} i2c_t;


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Modules                                   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

extern i2c_t const i2c0;


#endif // I2C_H
