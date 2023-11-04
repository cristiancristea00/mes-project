/**
 *  @file uart.c
 *  @author Cristian Cristea
 *  @date October 15, 2023
 *
 *  @brief Source file for the UART module
 **/


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "uart.h"

#include "config.h"

#include <avr/interrupt.h>
#include <util/atomic.h>
#include <avr/io.h>

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Macro to convert UART baud rate to the value to be used in the BAUD
 *        register of the USART.
 **/
#define UART_BAUD_RATE(X) ((uint16_t) ((4UL * F_CPU) / (X)))


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                            Private (static) API                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 *                                                                             *
 *                                    UART0                                    *
 *                                                                             *
 *******************************************************************************/

/**
 * @brief Initializes the UART0 module by setting the baud rate and enabling the
 *        transmitter and receiver. The UART0 module is configured for 8-bit
 *        with no parity and 1 stop bit. The Receive Complete Interrupt and
 *        Global interrupts are enabled.
 * @param[in] baudRate The baud rate
 * @param[in] receiveCallback The callback function to be called when a byte is
 *                            received
 * @return None
 **/
__attribute__((always_inline)) inline static void UART0_InitializeWithReceive(uint32_t const baudRate, uart_callback_t const receiveCallback);

/**
 * @brief Initializes the UART0 module by setting the baud rate and enabling the
 *        transmitter. The UART0 module is configured for 8-bit with no parity
 *        and 1 stop bit.
 * @param[in] baudRate The baud rate
 * @return None
 **/
__attribute__((always_inline)) inline static void UART0_Initialize(uint32_t const baudRate);

/**
 * @brief Sends a null-terminated string over UART0.
 * @param[in] string The null-terminated string to be sent
 * @return None
 **/
static void UART0_Print(char const * const string);

/**
 * @brief Send a single character over UART0.
 * @param[in] character The character to be sent
 * @return None
 */
__attribute__((always_inline)) inline static void UART0_PrintChar(char const character);

/**
 * @brief Sends a number of bytes over UART0.
 * @param[in] buffer The buffer containing the bytes to be sent
 * @param[in] bufferSize The number of bytes to be sent
 * @return None
 **/
static void UART0_SendData(uint8_t const * const buffer, uint8_t const bufferSize);

/**
 * @brief Sends a byte over UART0.
 * @param[in] dataByte The byte to be sent
 * @return None
 **/
static void UART0_SendByte(uint8_t const dataByte);

/**
 * @brief Checks if the UART0 module is busy sending data.
 * @return bool Whatever the TX line is busy
 * @retval true The UART1 module is busy.
 * @retval false The UART1 module is ready.
 **/
__attribute__((always_inline)) inline static bool UART0_TXBusy(void);

/**
 * @brief Registers a callback function to be called when a byte is received
 *        over UART0.
 * @param[in] callback The callback function to be registered
 * @return None
 **/
__attribute__((always_inline)) inline static void UART0_RegisterCallback(uart_callback_t const callback);

#if ( defined UART_PRINTF ) && ( defined __AVR64DD32__ )

/**
 * @brief Wrapper around the @ref UART0_PrintChar function to make it compatible
 *        with the C stream interface.
 * @param[in] character The character to be sent
 * @param[in] stream The stream used to send the character
 * @return int8_t Always returns 0
 **/
__attribute__((always_inline)) inline static int8_t UART0_SendChar(char const character, FILE * const stream);

#endif // UART_PRINTF && __AVR64DD32__

/*******************************************************************************
 *                                                                             *
 *                                    UART1                                    *
 *                                                                             *
 *******************************************************************************/

/**
 * @brief Initializes the UART1 module by setting the baud rate and enabling the
 *        transmitter and receiver. The UART1 module is configured for 8-bit
 *        with no parity and 1 stop bit. The Receive Complete Interrupt is
 *        enabled.
 * @param[in] baudRate The baud rate
 * @param[in] receiveCallback The callback function to be called when a byte is
 *                        received
 * @return None
 **/
__attribute__((always_inline)) inline static void UART1_InitializeWithReceive(uint32_t const baudRate, uart_callback_t const receiveCallback);

/**
 * @brief Initializes the UART1 module by setting the baud rate and enabling the
 *        transmitter. The UART1 module is configured for 8-bit with no parity
 *        and 1 stop bit.
 * @param[in] baudRate The baud rate
 * @return None
 **/
__attribute__((always_inline)) inline static void UART1_Initialize(uint32_t const baudRate);

/**
 * @brief Sends a null-terminated string over UART1.
 * @param[in] string The null-terminated string to be sent
 * @return None
 **/
static void UART1_Print(char const * const string);

/**
 * @brief Send a single character over UART1.
 * @param[in] character The character to be sent
 * @return None
 */
__attribute__((always_inline)) inline static void UART1_PrintChar(char const character);

/**
 * @brief Sends a number of bytes over UART1.
 * @param[in] buffer The buffer containing the bytes to be sent
 * @param[in] bufferSize The number of bytes to be sent
 * @return None
 **/
static void UART1_SendData(uint8_t const * const buffer, uint8_t const bufferSize);

/**
 * @brief Sends a byte over UART1.
 * @param[in] dataByte The byte to be sent
 * @return None
 **/
static void UART1_SendByte(uint8_t const dataByte);

/**
 * @brief Checks if the UART1 module is busy sending data.
 * @return bool Whatever the TX line is busy
 * @retval true The UART1 module is busy.
 * @retval false The UART1 module is ready.
 **/
__attribute__((always_inline)) inline static bool UART1_TXBusy(void);

/**
 * @brief Registers a callback function to be called when a byte is received
 *        over UART1.
 * @param[in] callback The callback function to be registered
 * @return None
 **/
__attribute__((always_inline)) inline static void UART1_RegisterCallback(uart_callback_t const callback);

#if ( defined UART_PRINTF ) && ( defined __AVR128DA48__ )

/**
 * @brief Wrapper around the @ref UART1_PrintChar function to make it compatible
 *        with the C stream interface.
 * @param[in] character The character to be sent
 * @param[in] stream The stream used to send the character
 * @return int8_t Always returns 0
 **/
__attribute__((always_inline)) inline static int8_t UART1_SendChar(char const character, FILE * const stream);

#endif // UART_PRINTF && __AVR128DA48__


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Private (static) definitions                        //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 *                                                                             *
 *                                    UART0                                    *
 *                                                                             *
 *******************************************************************************/

static uart_callback_t uart0Callback = NULL;

__attribute__((always_inline)) inline static void UART0_InitializeWithReceive(uint32_t const baudRate, uart_callback_t const receiveCallback)
{
    UART0_RegisterCallback(receiveCallback);

    UART0_Initialize(baudRate);

#if defined __AVR128DA48__
    PORTA.DIRCLR = PIN1_bm;
#elif defined __AVR64DD32__
    PORTD.DIRCLR = PIN5_bm;
#else
    #error "Invalid device!"
#endif

    USART0.CTRLA = USART_RXCIE_bm;

    USART0.CTRLB |= USART_RXEN_bm;

    return;
}

#if ( defined UART_PRINTF ) && ( defined __AVR64DD32__ )

__attribute__((always_inline)) inline static int8_t UART0_SendChar(char const character, __attribute__((unused)) FILE * const stream)
{
    UART0_PrintChar(character);

    return 0;
}

static FILE uart0Stream = FDEV_SETUP_STREAM(UART0_SendChar, NULL, _FDEV_SETUP_WRITE);

#endif // UART_PRINTF && __AVR64DD32__

__attribute__((always_inline)) inline static void UART0_Initialize(uint32_t const baudRate)
{
#if ( defined UART_PRINTF ) && ( defined __AVR64DD32__ )

    stdout = &uart0Stream;

#endif // UART_PRINTF && __AVR64DD32__

#if defined __AVR128DA48__
    PORTA.DIRSET = PIN0_bm;
    PORTA.OUTSET = PIN0_bm;
#elif defined __AVR64DD32__
    PORTMUX.USARTROUTEA = PORTMUX_USART0_ALT3_gc;
    PORTD.DIRSET = PIN4_bm;
    PORTD.OUTSET = PIN4_bm;
#else
    #error "Invalid device!"
#endif

    USART0.BAUD = UART_BAUD_RATE(baudRate);

    USART0.CTRLB = USART_TXEN_bm;

    return;
}

static void UART0_Print(char const * const string)
{
    char character = '\0';
    char const * currentStringPosition = string;

    while (true)
    {
        character = *currentStringPosition++;

        if (character == '\0')
        {
            break;
        }

        UART0_PrintChar(character);
    }

    return;
}

__attribute__((always_inline)) inline static void UART0_PrintChar(char const character)
{
    UART0_SendByte((char) character);

    return;
}

static void UART0_SendData(uint8_t const * const buffer, uint8_t const bufferSize)
{
    for (uint8_t bufferPosition = 0; bufferPosition < bufferSize; ++bufferPosition)
    {
        UART0_SendByte(buffer[bufferPosition]);
    }

    return;
}

static void UART0_SendByte(uint8_t const dataByte)
{
    while (UART0_TXBusy())
    {
        TightLoopContents();
    }

    USART0.TXDATAL = dataByte;

    return;
}

__attribute__((always_inline)) inline static inline bool UART0_TXBusy(void)
{
    return !(USART0.STATUS & USART_DREIF_bm);
}

__attribute__((always_inline)) inline static void UART0_RegisterCallback(uart_callback_t const callback)
{
    uart0Callback = callback;

    return;
}


/*******************************************************************************
 *                                                                             *
 *                                    UART1                                    *
 *                                                                             *
 *******************************************************************************/

static uart_callback_t uart1Callback = NULL;

__attribute__((always_inline)) inline static void UART1_InitializeWithReceive(uint32_t const baudRate, uart_callback_t const receiveCallback)
{
    UART1_RegisterCallback(receiveCallback);

    UART1_Initialize(baudRate);

    PORTC.DIRCLR = PIN1_bm;

    USART1.CTRLA = USART_RXCIE_bm;

    USART1.CTRLB |= USART_RXEN_bm;

    return;
}

#if ( defined UART_PRINTF ) && ( defined __AVR128DA48__ )

__attribute__((always_inline)) inline static int8_t UART1_SendChar(char const character, __attribute__((unused)) FILE * const stream)
{
    UART1_PrintChar(character);

    return 0;
}

static FILE uart1Stream = FDEV_SETUP_STREAM(UART1_SendChar, NULL, _FDEV_SETUP_WRITE);

#endif // UART_PRINTF && __AVR128DA48__

__attribute__((always_inline)) inline static void UART1_Initialize(uint32_t const baudRate)
{
#if ( defined UART_PRINTF ) && ( defined __AVR128DA48__ )

    stdout = &uart1Stream;

#endif // UART_PRINTF && __AVR128DA48__

    PORTC.DIRSET = PIN0_bm;
    PORTC.OUTSET = PIN0_bm;

    USART1.BAUD = UART_BAUD_RATE(baudRate);

    USART1.CTRLB = USART_TXEN_bm;

    return;
}

static void UART1_Print(char const * const string)
{
    char character = '\0';
    char const * currentStringPosition = string;

    while (true)
    {
        character = *currentStringPosition++;

        if (character == '\0')
        {
            break;
        }

        UART1_PrintChar(character);
    }

    return;
}

__attribute__((always_inline)) inline static void UART1_PrintChar(char const character)
{
    UART1_SendByte((char) character);

    return;
}

static void UART1_SendData(uint8_t const * const buffer, uint8_t const bufferSize)
{
    for (uint8_t bufferPosition = 0; bufferPosition < bufferSize; ++bufferPosition)
    {
        UART1_SendByte(buffer[bufferPosition]);
    }

    return;
}

static void UART1_SendByte(uint8_t const dataByte)
{
    while (UART1_TXBusy())
    {
        TightLoopContents();
    }

    USART1.TXDATAL = dataByte;

    return;
}

__attribute__((always_inline)) inline static inline bool UART1_TXBusy(void)
{
    return !(USART1.STATUS & USART_DREIF_bm);
}

__attribute__((always_inline)) inline static void UART1_RegisterCallback(uart_callback_t const callback)
{
    uart1Callback = callback;

    return;
}


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                     Interrupt Service Routines (ISRs)                      //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 *                                                                             *
 *                                    UART0                                    *
 *                                                                             *
 *******************************************************************************/

ISR(USART0_RXC_vect)
{
    uint8_t const byte = USART0.RXDATAL;

    if (uart0Callback != NULL)
    {
        uart0Callback(byte);
    }
}

/*******************************************************************************
 *                                                                             *
 *                                    UART1                                    *
 *                                                                             *
 *******************************************************************************/

ISR(USART1_RXC_vect)
{
    uint8_t const byte = USART1.RXDATAL;

    if (uart1Callback != NULL)
    {
        uart1Callback(byte);
    }
}


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Modules                                   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 *                                                                             *
 *                                    UART0                                    *
 *                                                                             *
 *******************************************************************************/

/**
 * @brief  Module for UART0
 **/
uart_t const uart0 = {
    .Initialize = UART0_Initialize,
    .InitializeWithReceive = UART0_InitializeWithReceive,
    .SendByte = UART0_SendByte,
    .SendData = UART0_SendData,
    .PrintChar = UART0_PrintChar,
    .Print = UART0_Print,
    .RegisterCallback = UART0_RegisterCallback
};

/*******************************************************************************
 *                                                                             *
 *                                    UART1                                    *
 *                                                                             *
 *******************************************************************************/

/**
 * @brief  Module for UART1
 **/
uart_t const uart1 = {
    .Initialize = UART1_Initialize,
    .InitializeWithReceive = UART1_InitializeWithReceive,
    .SendByte = UART1_SendByte,
    .SendData = UART1_SendData,
    .PrintChar = UART1_PrintChar,
    .Print = UART1_Print,
    .RegisterCallback = UART1_RegisterCallback
};
