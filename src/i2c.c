/**
 *  @file i2c.c
 *  @author Cristian Cristea
 *  @date October 16, 2023
 *
 *  @brief Source file for the I2C module
 **/

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "i2c.h"

#include <avr/io.h>

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#define I2C_DATA_bm    UINT8(0x01)


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

typedef enum I2C_STATE
{
    I2C_INIT   = 0x00,
    I2C_ACKED  = 0x01,
    I2C_NACKED = 0x02,
    I2C_READY  = 0x03,
    I2C_ERROR  = 0x04,
} i2c_state_t;

typedef enum I2C_DATA_DIRECTION
{
    I2C_DATA_SEND    = 0x00,
    I2C_DATA_RECEIVE = 0x01,
} i2c_data_direction_t;


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                            Private (static) API                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Initializes the I2C module on the TWI0 peripheral with the given mode.
 * @param[in] mode The mode of the I2C bus: Standard, Fast or Fast Plus
 * @return None
 **/
__attribute__((always_inline)) inline static void I2C0_Inititialize(i2c_mode_t const mode);

/**
 * @brief Sends a specific number of bytes to the device on the I2C bus.
 * @param[in] address The address of the device
 * @param[in] dataForSend Pointer to the data to be sent
 * @param[in] length The length of the data to be sent
 * @return i2c_error_code_t The error code of the operation
 * @retval I2C_NULL_POINTER If the data pointer is NULL
 * @retval I2C_NACK_OF_ADDRESS If the device did not respond with ACK
 * @retval I2C_COMMUNICATION_ERROR If the data was not sent correctly
 * @retval I2C_OK If the data was sent correctly
 **/
static i2c_error_code_t I2C0_SendData(uint8_t const address, uint8_t const * const dataForSend, uint8_t const initialLength);

/**
 * @brief Waits for the I2C bus to be ready after write operation.
 * @return i2c_state_t The response of the device or lack thereof
 * @retval I2C_ERROR If a bus error or arbitration lost occurred
 * @retval I2C_NACKED If there was no ACK on the bus
 * @retval I2C_ACKED If the device responded with ACK
 **/
static i2c_state_t I2C0_WaitWrite(void);

/**
 * @brief Receives a specific number of bytes from the device from the I2C bus.
 * @param[in] address The address of the device
 * @param[out] dataForReceive Pointer to the data to be received
 * @param[in] length The length of the data to be received
 * @return i2c_error_code_t The error code of the operation
 * @retval I2C_NULL_POINTER If the data pointer is NULL
 * @retval I2C_NACK_OF_ADDRESS If the device did not respond with ACK
 * @retval I2C_COMMUNICATION_ERROR If the data was not received correctly
 * @retval I2C_OK If the data was received correctly
 **/
static i2c_error_code_t I2C0_ReceiveData(uint8_t const address, uint8_t * dataForReceive, uint8_t const initialLength);

/**
 * @brief Waits for the I2C bus to be ready after read operation.
 * @return i2c_state_t The response of the device or lack thereof
 * @retval I2C_ERROR If a bus error or arbitration lost occurred
 * @retval I2C_READY If the device is ready to send data
 **/
static i2c_state_t I2C0_WaitRead(void);

/**
 * @brief Checks if a device is available on the I2C bus.
 * @param[in] address The address of the device
 * @return bool Whether the device is available or not
 * @retval true If the device is available
 * @retval false If the device is not available
 **/
static bool I2C0_ClientAvailable(uint8_t const clientAddress);

/**
 * @brief Sets the I2C bus address of the device based on the given chip address
 *        and the read/write bit.
 * @param[in] deviceAddress The chip address of the device
 * @param[in] dataDirection The direction of the data: send or receive
 * @return uint8_t The address of the device on the I2C bus
 * @retval 0x00 If the data direction is invalid
 * @retval The address of the device on the I2C bus
 **/
static uint8_t I2C0_SetAdressDirectionBit(uint8_t const deviceAddress, i2c_data_direction_t const dataDirection);

/**
 * @brief Ends the I2C communication by sending a stop condition.
 * @return None
 **/
__attribute__((always_inline)) inline static void I2C0_EndTransation(void);


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Private (static) definitions                        //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

__attribute__((always_inline)) inline static void I2C0_Inititialize(i2c_mode_t const mode)
{
    // Select I2C pins to PC2 - SDA and PC3 - SCL
    PORTMUX.TWIROUTEA = PORTMUX_TWI0_ALT2_gc;

    // Enable internal pull-ups
    PORTC.PIN2CTRL = PORT_PULLUPEN_bm;
    PORTC.PIN3CTRL = PORT_PULLUPEN_bm;

    // Host baud rate control
    TWI0.MBAUD = (uint8_t) mode;

    // Enable Fast Mode Plus
    if (mode == I2C_MODE_FAST_PLUS)
    {
        TWI0.CTRLA = TWI_FMPEN_ON_gc;
    }
    
    // Initialise the address register
    TWI0.MADDR = 0x00;

    // Initialise the data register
    TWI0.MDATA = 0x00;
    
    // Enable I2C as host
    TWI0.MCTRLA = TWI_ENABLE_bm;

    // Set the bus state to idle
    TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;

    return;
}

static i2c_error_code_t I2C0_SendData(uint8_t const address, uint8_t const * const dataForSend, uint8_t const initialLength)
{
    if (dataForSend == NULL && initialLength != 0)
    {
        return I2C_NULL_POINTER;
    }

    TWI0.MADDR = I2C0_SetAdressDirectionBit(address, I2C_DATA_SEND);

    if (I2C0_WaitWrite() != I2C_ACKED)
    {
        return I2C_NACK_OF_ADDRESS;
    }

    uint8_t bytesSent = 0;
    uint8_t length = initialLength;
    uint8_t const * dataPointer = dataForSend;

    while (length != 0)
    {
        --length;

        TWI0.MDATA = *dataPointer;

        if (I2C0_WaitWrite() == I2C_ACKED)
        {
            ++bytesSent;
            ++dataPointer;
        }
        else
        {
            break;
        }
    }

    if (bytesSent == initialLength)
    {
        return I2C_OK;
    }
    else
    {
        return I2C_COMMUNICATION_ERROR;
    }
}

static i2c_state_t I2C0_WaitWrite(void)
{
    i2c_state_t state = I2C_INIT;

    do
    {
        if (TWI0.MSTATUS & (TWI_BUSERR_bm | TWI_ARBLOST_bm))
        {
            state = I2C_ERROR;
        }
        else if (TWI0.MSTATUS & (TWI_WIF_bm | TWI_RIF_bm))
        {
            if (!(TWI0.MSTATUS & TWI_RXACK_bm))
            {
                state = I2C_ACKED;
            }
            else
            {
                state = I2C_NACKED;
            }
        }
    }
    while (state == I2C_INIT);

    return state;
}

static i2c_error_code_t I2C0_ReceiveData(uint8_t const address, uint8_t * const dataForReceive, uint8_t const initialLength)
{
    if (dataForReceive == NULL && initialLength != 0)
    {
        return I2C_NULL_POINTER;
    }

    TWI0.MADDR = I2C0_SetAdressDirectionBit(address, I2C_DATA_RECEIVE);

    if (I2C0_WaitWrite() != I2C_ACKED)
    {
        return I2C_NACK_OF_ADDRESS;
    }

    uint8_t bytesReceived = 0;
    uint8_t length = initialLength;
    uint8_t * dataPointer = dataForReceive;

    while (length != 0)
    {
        --length;

        if (I2C0_WaitRead() == I2C_READY)
        {
            *dataPointer = TWI0.MDATA;

            TWI0.MCTRLB = (length == 0) ? (TWI_ACKACT_NACK_gc | TWI_MCMD_STOP_gc) : TWI_MCMD_RECVTRANS_gc;

            ++bytesReceived;
            ++dataPointer;
        }
        else
        {
            break;
        }
    }

    if (bytesReceived == initialLength)
    {
        return I2C_OK;
    }
    else
    {
        return I2C_COMMUNICATION_ERROR;
    }
}

static i2c_state_t I2C0_WaitRead(void)
{
    i2c_state_t state = I2C_INIT;

    do
    {
        if (TWI0.MSTATUS & (TWI_BUSERR_bm | TWI_ARBLOST_bm))
        {
            state = I2C_ERROR;
        }
        else if (TWI0.MSTATUS & (TWI_WIF_bm | TWI_RIF_bm))
        {
            state = I2C_READY;
        }
    }
    while (state == I2C_INIT);

    return state;
}

static uint8_t I2C0_SetAdressDirectionBit(uint8_t const deviceAddress, i2c_data_direction_t const dataDirection)
{
    if (dataDirection == I2C_DATA_SEND)
    {
        return ( (deviceAddress << 1) & ~I2C_DATA_bm );
    }
    else if (dataDirection == I2C_DATA_RECEIVE)
    {
        return ( (deviceAddress << 1) | I2C_DATA_bm );
    }
    else
    {
        return 0x00;
    }
}

static bool I2C0_ClientAvailable(uint8_t const clientAddress)
{
    i2c_error_code_t const returnValue = I2C0_SendData(clientAddress, NULL, 0);
    
    I2C0_EndTransation();
    
    return ( returnValue == I2C_OK );
}

__attribute__((always_inline)) inline static void I2C0_EndTransation(void)
{
    // Sends a STOP condition on the bus and clears the internal state
    TWI0.MCTRLB = TWI_MCMD_STOP_gc;

    return;
}



////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Modules                                   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Module for I2C0
 **/
i2c_t const i2c0 = {
    .Initialize = I2C0_Inititialize,
    .SendData = I2C0_SendData,
    .ReceiveData = I2C0_ReceiveData,
    .EndTransaction = I2C0_EndTransation,
    .IsClientAvailable = I2C0_ClientAvailable
};
