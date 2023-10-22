/**
 *  @file bluetooth.h
 *  @author Cristian Cristea
 *  @date October 22, 2023
 *
 *  @brief Header file for the Bluetooth module
 **/


#ifndef BLUETOOTH_H
#define	BLUETOOTH_H


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "vector.h"
#include "uart.h"

#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

typedef enum BLUETOOTH_ERROR_CODE
{
    BLUETOOTH_OK              = 0x00,
    BLUETOOTH_NULL_POINTER    = 0x01,
    BLUETOOTH_SEND_FAILED     = 0x02,
    BLUETOOTH_RECEIVE_FAILED  = 0x03,
    BLUETOOTH_FAILED_CHECKSUM = 0x04,
} bluetooth_error_code_t;

typedef struct BLUETOOTH_DEVICE
{
    // UART device
    uart_t const * uartDevice;
} bluetooth_device_t;

typedef void (* struct_interpret_t) (void * const, vector_t * const);


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                 Public API                                 //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Initializes the Bluetooth device.
 * @param[in, out] device Bluetooth device
 * @param[in] uartDevice UART device
 * @return bluetooth_error_code_t The error code
 * @retval BLUETOOTH_OK If the device was initialized successfully
 * @retval BLUETOOTH_NULL_POINTER If the device or the UART device is NULL
 **/
bluetooth_error_code_t BLUETOOTH_Initialize(bluetooth_device_t * const device, uart_t const * const uartDevice);

/**
 * @brief  Sends data to the Bluetooth device from a buffer. Sends just one
 *         packet at a time and should be called within a loop.
 * @param[in] device Bluetooth device
 * @param[in] buffer Buffer containing the data to send
 * @param[in] bufferSize Size of the buffer
 * @return bluetooth_error_code_t The error code
 * @retval BLUETOOTH_OK If the data was sent successfully
 * @retval BLUETOOTH_NULL_POINTER If the buffer is NULL
 * @retval BLUETOOTH_SEND_FAILED If the data could not be sent
 **/
bluetooth_error_code_t BLUETOOTH_SendData(bluetooth_device_t const * const device, uint8_t const * const buffer, uint8_t const bufferSize);

/**
 * @brief Receives data from the Bluetooth device and stores it in the given data
 *        structure using the given interpret function. Receives just one packet
 *        at a time and should be called within a loop.
 * @param[in] device Bluetooth device
 * @param[out] dataStructure Data structure to store the received data
 * @param[in] structInterpreter Interpret function to use on the data
 * @return bluetooth_error_code_t The error code
 * @retval BLUETOOTH_OK If the data was received successfully
 * @retval BLUETOOTH_NULL_POINTER If the data structure or the interpreter is NULL
 * @retval BLUETOOTH_RECEIVE_FAILED If the data could not be received
 **/
bluetooth_error_code_t BLUETOOTH_ReceiveData(bluetooth_device_t const * const device, void * const dataStructure, struct_interpret_t structInterpreter);

/**
 * @brief Callback function for the UART device that that stores received data
 *        in the vector.
 * @param[in] data Byte received
 * @return None
 **/
void BLUETOOTH_ReceiveCallback(uint8_t const data);

#endif // BLUETOOTH_H