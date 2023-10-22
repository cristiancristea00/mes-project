/**
 *  @file bluetooth.c
 *  @author Cristian Cristea
 *  @date October 22, 2023
 *
 *  @brief Source file for the Bluetooth module
 **/


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "bluetooth.h"

#include "config.h"
#include "utils.h"
#include "vector.h"
#include "uart.h"
#include "crc8.h"

#include <util/delay.h>

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#define BLUETOOTH_ACK_NACK_SIZE     UINT8(3)
#define BLUETOOTH_ACK_NACK_BYTES    UINT8(1)


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

typedef enum BLUETOOTH_STATUS
{
    BLUETOOTH_IDLE        = 0x00,
    BLUETOOTH_IN_PROGRESS = 0x01,
    BLUETOOTH_FINISHED    = 0x02,
} bluetooth_status_t;

typedef enum BLUETOOTH_RESPONSE
{
    BLUETOOTH_EMPTY            = 0x00,
    BLUETOOTH_ACKED            = 0x01,
    BLUETOOTH_NACKED           = 0x02,
    BLUETOOTH_INVALID_CHECKSUM = 0x03,
    BLUETOOTH_NOT_CONFIRMED    = 0x04,
} bluetooth_response_t;


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                            Private (static) API                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Checks if the device and its UART device are valid.
 * @param[in] device Bluetooth device
 * @return bluetooth_error_code_t The error code
 * @retval BLUETOOTH_OK If the device is valid and its UART device is valid
 * @retval BLUETOOTH_NULL_POINTER If the device or the UART device is NULL
 **/
static bluetooth_error_code_t BLUETOOTH_CheckNull(bluetooth_device_t const * const device);

/**
 * @brief Sends a prepared packet using the UART device. A prepared packet is
 *        composed of a byte that specifies the length of the buffer, followed
 *        by the buffer itself and finally a byte that specifies the checksum.
 *
 *        ┌───────────┬───────────────────┬──────────┐
 *        │ NUMBER OF │     BUFFER OF     │          │
 *        │           │ ...           ... │ CHECKSUM │
 *        │   BYTES   │       BYTES       │          │
 *        └───────────┴───────────────────┴──────────┘
 *
 * @param[in] device Bluetooth device
 * @param[in] buffer Buffer to be sent
 * @param[in] bufferSize Size of the buffer
 * @return None
 **/
__attribute__((always_inline)) inline static void BLUETOOTH_SendPreparedData(bluetooth_device_t const * const device, uint8_t const * const buffer, uint8_t const bufferSize);

/**
 * @brief Waits for 500 miliseconds to receive a packet containing an ACK or
 *        a NACK and returns the result.
 * @return bluetooth_response_t The response
 * @retval BLUETOOTH_ACKED If the packet was ACKED
 * @retval BLUETOOTH_NACKED If the packet was NACKED
 * @retval BLUETOOTH_INVALID_CHECKSUM If the packet was invalid
 * @retval BLUETOOTH_NOT_CONFIRMED If the packet was not received
 **/
static bluetooth_response_t BLUETOOTH_WaitForConfirmation(void);

/**
 * @brief Returns ACK or NACK if the checksum is valid based on the received
 *        packet in the vector or an error code if the checksum is invalid.
 * @return bluetooth_response_t The response
 * @retval BLUETOOTH_ACKED If the packet was ACKED
 * @retval BLUETOOTH_NACKED If the packet was NACKED
 * @retval BLUETOOTH_INVALID_CHECKSUM If the packet was invalid
 **/
static bluetooth_response_t BLUETOOTH_GetResponseReceived(void);

/**
 * @brief Checks if the received byte is the last byte of the packet.
 * @return bool Whether the byte is the last byte of the packet
 * @retval true The byte is the last byte of the packet
 * @retval false The byte is not the last byte of the packet
 **/
__attribute__((always_inline)) inline static bool BLUETOOTH_IsLastByte(void);

/**
 * @brief Computes the number of reamining bytes to be received.
 * @return uint8_t Number of remaining bytes
 **/
__attribute__((always_inline)) inline static uint8_t BLUETOOTH_GetNumberOfBytesToReceive(void);

/**
 * @brief Computes the checksum of the received packet.
 * @return bluetooth_error_code_t The error code
 * @retval BLUETOOTH_OK If the checksum is valid
 * @retval BLUETOOTH_FAILED_CHECKSUM If the checksum is invalid
 **/
static bluetooth_error_code_t BLUETOOTH_VerifyReceiveChecksum(void);

/**
 * @brief Computes the checksum of a byte buffer and checks it against the
 *        received checksum.
 * @param[in] data Byte buffer
 * @param[in] dataLength Length of the byte buffer
 * @param[in] checksum Received checksum
 * @return bluetooth_error_code_t The error code
 * @retval BLUETOOTH_OK If the checksum is valid
 * @retval BLUETOOTH_FAILED_CHECKSUM If the checksum is invalid
 **/
static bluetooth_error_code_t BLUETOOTH_VerifyChecksum(uint8_t const * const data, uint8_t const dataLength, uint8_t const checksum);

/**
 * @brief Sends an ACK packet.
 * @param[in] device Bluetooth device
 **/
static void BLUETOOTH_SendAcknowledge(bluetooth_device_t const * const device);

/**
 * @brief Sends a NACK packet.
 * @param[in] device Bluetooth device
 **/
static void BLUETOOTH_SendNotAcknowledge(bluetooth_device_t const * const device);

/**
 * @brief Sends a packet containing an ACK or a NACK.
 * @param[in] device Bluetooth device
 * @param[in] response ACK or NACK
 * @return None
 **/
__attribute__((always_inline)) inline static void BLUETOOTH_SendResponse(bluetooth_device_t const * const device, bluetooth_response_t const response);

/**
 * @brief Computes the checksum of a byte buffer.
 * @param[in] data Byte buffer
 * @param[in] dataLength Length of the byte buffer
 * @return uint8_t The CRC8 checksum
 **/
__attribute__((always_inline)) inline static uint8_t BLUETOOTH_ComputeChecksum(uint8_t const * const data, uint8_t const dataLength);

/**
 * @brief Ends the current transmission by clearing the vector and trasmission
 *        flag.
 * @return None
 **/
__attribute__((always_inline)) inline static void BLUETOOTH_EndTransmission(void);


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Private (static) definitions                        //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Vector used to store the received data.
 **/
static vector_t receiveBuffer;

/**
 * @brief Flag used to indicate the status of the transmission. 
 **/
static volatile bluetooth_status_t transsmitionStatus = BLUETOOTH_IDLE;

static bluetooth_error_code_t BLUETOOTH_CheckNull(bluetooth_device_t const * const device)
{
    if ((device == NULL) || (device->uartDevice == NULL))
    {
        LOG_ERROR("Found NULL pointer in Bluetooth check");
        return BLUETOOTH_NULL_POINTER;
    }
    else
    {
        return BLUETOOTH_OK;
    }
}

__attribute__((always_inline)) inline static void BLUETOOTH_SendPreparedData(bluetooth_device_t const * const device, uint8_t const * const buffer, uint8_t const bufferSize)
{
    uint8_t const newLength = bufferSize + 2;

    // Please forgive me for the sin that I'm committing

    uint8_t * preparedData = (uint8_t *) calloc(newLength, sizeof(uint8_t));

    preparedData[0] = bufferSize;
    memcpy(preparedData + 1, buffer, bufferSize);
    preparedData[newLength - 1] = BLUETOOTH_ComputeChecksum(preparedData, bufferSize);

    device->uartDevice->SendData(preparedData, newLength);

    free(preparedData);
    preparedData = NULL;

    // My soul is finally free

    return;
}

static bluetooth_response_t BLUETOOTH_WaitForConfirmation(void)
{
    PauseMiliseconds(500);

    if (transsmitionStatus == BLUETOOTH_FINISHED)
    {
        return BLUETOOTH_GetResponseReceived();
    }
    else
    {
        return BLUETOOTH_NOT_CONFIRMED;
    }
}

static bluetooth_response_t BLUETOOTH_GetResponseReceived(void)
{
    uint8_t response[BLUETOOTH_ACK_NACK_SIZE] = { 0 };

    response[2] = Vector_RemoveByte(&receiveBuffer);
    response[1] = Vector_RemoveByte(&receiveBuffer);
    response[0] = Vector_RemoveByte(&receiveBuffer);

    if (BLUETOOTH_VerifyChecksum(response, BLUETOOTH_ACK_NACK_BYTES, response[2]) == BLUETOOTH_OK)
    {
        return (bluetooth_response_t) response[1];
    }
    else
    {
        return BLUETOOTH_INVALID_CHECKSUM;
    }
}

__attribute__((always_inline)) inline static bool BLUETOOTH_IsLastByte(void)
{
    return BLUETOOTH_GetNumberOfBytesToReceive() == receiveBuffer.bufferSize;
}

__attribute__((always_inline)) inline static uint8_t BLUETOOTH_GetNumberOfBytesToReceive(void)
{
    return Vector_FirstByte(&receiveBuffer) + 1;
}

static bluetooth_error_code_t BLUETOOTH_VerifyReceiveChecksum(void)
{
    uint8_t const * const data = receiveBuffer.internalBuffer;
    uint8_t const dataLength = Vector_FirstByte(&receiveBuffer);
    uint8_t const checksum = Vector_LastByte(&receiveBuffer);

    return BLUETOOTH_VerifyChecksum(data, dataLength, checksum);
}

static bluetooth_error_code_t BLUETOOTH_VerifyChecksum(uint8_t const * const data, uint8_t const dataLength, uint8_t const checksum)
{
    uint8_t const computedChecksum = BLUETOOTH_ComputeChecksum(data, dataLength);

    if (checksum == computedChecksum)
    {
        return BLUETOOTH_OK;
    }
    else
    {
        return BLUETOOTH_FAILED_CHECKSUM;
    }
}

static void BLUETOOTH_SendAcknowledge(bluetooth_device_t const * const device)
{
    BLUETOOTH_SendResponse(device, BLUETOOTH_ACKED);

    return;
}

static void BLUETOOTH_SendNotAcknowledge(bluetooth_device_t const * const device)
{
    BLUETOOTH_SendResponse(device, BLUETOOTH_NACKED);

    return;
}

__attribute__((always_inline)) inline static void BLUETOOTH_SendResponse(bluetooth_device_t const * const device, bluetooth_response_t const response)
{
    uint8_t packet[BLUETOOTH_ACK_NACK_SIZE] = { 0 };

    packet[0] = BLUETOOTH_ACK_NACK_BYTES;
    packet[1] = (uint8_t) (response);
    packet[2] = BLUETOOTH_ComputeChecksum(packet, BLUETOOTH_ACK_NACK_BYTES);

    device->uartDevice->SendData(packet, BLUETOOTH_ACK_NACK_SIZE);

    return;
}

__attribute__((always_inline)) inline static uint8_t BLUETOOTH_ComputeChecksum(uint8_t const * const data, uint8_t const dataLength)
{
    return CRC8_Compute(data, dataLength + 1);
}

__attribute__((always_inline)) inline static void BLUETOOTH_EndTransmission(void)
{
    transsmitionStatus = BLUETOOTH_IDLE;
    Vector_Clear(&receiveBuffer);

    return;
}


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Public definitions                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

bluetooth_error_code_t BLUETOOTH_Initialize(bluetooth_device_t * const device, uart_t const * const uartDevice)
{
    LOG_INFO("Started Bluetooth initialization");

    Vector_Initialize(&receiveBuffer);

    device->uartDevice = uartDevice;

    bluetooth_error_code_t initResult = BLUETOOTH_CheckNull(device);

    if (initResult == BLUETOOTH_OK)
    {
        LOG_INFO("Finished the Bluetooth initialization");
    }
    else
    {
        LOG_ERROR("Couldn't finish the Bluetooth initialization");
    }

    return initResult;
}

bluetooth_error_code_t BLUETOOTH_SendData(bluetooth_device_t const * const device, uint8_t const * const buffer, uint8_t const bufferSize)
{
    if (buffer == NULL || bufferSize == 0)
    {
        return BLUETOOTH_NULL_POINTER;
    }
    
    bluetooth_error_code_t sendResult = BLUETOOTH_OK;

    LOG_INFO("Started Bluetooth send transmission");

    bluetooth_response_t transsmitionResponse = BLUETOOTH_EMPTY;

    uint8_t tryCount = 10;

    while (tryCount != 0)
    {
        BLUETOOTH_SendPreparedData(device, buffer, bufferSize);

        transsmitionResponse = BLUETOOTH_WaitForConfirmation();

        if (transsmitionResponse == BLUETOOTH_ACKED)
        {
            sendResult = BLUETOOTH_OK;
            break;
        }
        else if (transsmitionResponse == BLUETOOTH_NACKED)
        {
            LOG_WARNING("Bluetooth send transmission partner NACKED. Retrying...");
        }
        else
        {
            LOG_WARNING("Bluetooth send transmission partner is not responding. Retrying...");
        }

        --tryCount;
    }

    if (tryCount == 0)
    {
        sendResult = BLUETOOTH_SEND_FAILED;
        LOG_ERROR("Bluetooth send transmission failed");
    }
    else
    {
        LOG_INFO("Finished Bluetooth send transmission");
    }

    BLUETOOTH_EndTransmission();

    return sendResult;
}

bluetooth_error_code_t BLUETOOTH_ReceiveData(bluetooth_device_t const * const device, void * const dataStructure, struct_interpret_t structInterpreter)
{
    if (structInterpreter == NULL || dataStructure == NULL)
    {
        return BLUETOOTH_NULL_POINTER;
    }
    
    bluetooth_error_code_t receiveResult = BLUETOOTH_OK;

    LOG_INFO("Started Bluetooth receive transmission");

    while (transsmitionStatus != BLUETOOTH_FINISHED)
    {
        TightLoopContents();
    }

    receiveResult = BLUETOOTH_VerifyReceiveChecksum();

    if (receiveResult == BLUETOOTH_OK)
    {
        structInterpreter(dataStructure, &receiveBuffer);

        BLUETOOTH_SendAcknowledge(device);

        LOG_INFO("Finished Bluetooth receive transmission");
    }
    else
    {
        BLUETOOTH_SendNotAcknowledge(device);
        LOG_ERROR("Bluetooth receive transmission failed");
    }

    BLUETOOTH_EndTransmission();

    return receiveResult;
}

void BLUETOOTH_ReceiveCallback(uint8_t const data)
{
    if (transsmitionStatus == BLUETOOTH_IDLE)
    {
        transsmitionStatus = BLUETOOTH_IN_PROGRESS;
        Vector_AddByte(&receiveBuffer, data);
    }
    else if (transsmitionStatus == BLUETOOTH_IN_PROGRESS)
    {
        if (BLUETOOTH_IsLastByte())
        {
            transsmitionStatus = BLUETOOTH_FINISHED;
        }
        Vector_AddByte(&receiveBuffer, data);
    }

    return;
}