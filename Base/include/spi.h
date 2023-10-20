/**
 *  @file spi.h
 *  @author Cristian Cristea
 *  @date October 20, 2023
 *
 *  @brief Header file for the SPI module
 **/


#ifndef SPI_H
#define	SPI_H


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

typedef enum SPI_ERROR_CODE
{
    SPI_OK                  = 0x00,
    SPI_NULL_POINTER        = 0x01,
} spi_error_code_t;

typedef enum SPI_CHIP_SELECT
{
    SPI_CS1 = 0x00,
    SPI_CS2 = 0x01,
    SPI_CS3 = 0x02,
} spi_chip_select_t;

typedef void (* spi_inititialize_t) (void);
typedef spi_error_code_t (* spi_send_data_t) (uint8_t const * const dataForSend, uint8_t const initialLength);
typedef spi_error_code_t (* spi_receive_data_t) (uint8_t * const dataForReceive, uint8_t const initialLength);
typedef spi_error_code_t (* spi_exchange_data_t) (uint8_t * const dataForExchange, uint8_t const initialLength);
typedef void (* spi_client_t) (spi_chip_select_t const chipSelect);

/**
 * @brief Object struct for the SPI module
 **/
typedef struct SPI
{
    spi_inititialize_t Initialize;
    spi_send_data_t SendData;
    spi_receive_data_t ReceiveData;
    spi_exchange_data_t ExchangeData;
    spi_client_t ClientSelect;
    spi_client_t ClientDeselect;
} spi_t;


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Modules                                   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

extern spi_t const spi0;


#endif // SPI_H
