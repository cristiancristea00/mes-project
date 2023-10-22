/**
 *  @file crc8.h
 *  @author Cristian Cristea
 *  @date October 22, 2023
 *
 *  @brief Header file for the CRC8 module
 **/


#ifndef CRC8_H
#define	CRC8_H


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                 Public API                                 //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Computes the CRC-8 of a given data buffer using the CRC-8-AUTOSAR
 *        version. The CRC-8-AUTOSAR algorithm has the following properties:
 *        - The polynomial is x^8 + x^5 + x^3 + x^2 + x + 1 (i.e. 0x2F)
 *        - The initial value is 0xFF
 *        - The final XOR value is 0xFF
 *
 * @param[in] data The data buffer to compute the CRC-8 of
 * @param[in] dataLength The length of the data buffer
 * @return uint8_t The CRC-8-AUTOSAR value of the data buffer
 **/
uint8_t CRC8_Compute(uint8_t const * const data, uint8_t const dataLength);

#endif // CRC8_H