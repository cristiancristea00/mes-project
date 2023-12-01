/**
 *  @file bme280.h
 *  @author Cristian Cristea
 *  @date October 20, 2023
 *
 *  @brief Header file for the BME280 module
 **/


#ifndef BME280_H
#define BME280_H


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "vector.h"
#include "utils.h"
#include "i2c.h"

#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

// Principal and secondary I2C addresses of the chip

#define BME280_I2C_ADDRESS        UINT8(0x76)
#define BME280_I2C_ADDRESS_SEC    UINT8(0x77)

// Serialized data length

#define BME280_SERIALIZED_SIZE    UINT8(12)


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

typedef enum BME280_ERROR_CODE
{
    BME280_OK                  = 0x00,
    BME280_DEVICE_NOT_FOUND    = 0x01,
    BME280_NULL_POINTER        = 0x02,
    BME280_COMMUNICATION_ERROR = 0x03,
    BME280_INVALID_LENGTH      = 0x04,
    BME280_NVM_COPY_FAILED     = 0x05,
} bme280_error_code_t;

typedef enum BME280_OVERSAMPLING
{
    BME280_NO_OVERSAMPLING  = 0x00,
    BME280_OVERSAMPLING_1X  = 0x01,
    BME280_OVERSAMPLING_2X  = 0x02,
    BME280_OVERSAMPLING_4X  = 0x03,
    BME280_OVERSAMPLING_8X  = 0x04,
    BME280_OVERSAMPLING_16X = 0x05,
} bme280_oversampling_t;

typedef enum BME280_IIR_FILTER
{
    BME280_IIR_FILTER_OFF = 0x00,
    BME280_IIR_FILTER_2   = 0x01,
    BME280_IIR_FILTER_4   = 0x02,
    BME280_IIR_FILTER_8   = 0x03,
    BME280_IIR_FILTER_16  = 0x04,

} bme280_iir_filter_t;

typedef enum BME280_STANDY_TIME
{
    BME280_STANDBY_TIME_0_5_MS  = 0x00,
    BME280_STANDBY_TIME_62_5_MS = 0x01,
    BME280_STANDBY_TIME_125_MS  = 0x02,
    BME280_STANDBY_TIME_250_MS  = 0x03,
    BME280_STANDBY_TIME_500_MS  = 0x04,
    BME280_STANDBY_TIME_1000_MS = 0x05,
    BME280_STANDBY_TIME_10_MS   = 0x06,
    BME280_STANDBY_TIME_20_MS   = 0x07,
} bme280_standby_time_t;

typedef enum BME280_POWER_MODE
{
    BME280_SLEEP_MODE  = 0x00,
    BME280_FORCED_MODE = 0x01,
    BME280_NORMAL_MODE = 0x03,
} bme280_power_mode_t;

typedef bme280_error_code_t (* bme280_i2c_read_t) (i2c_t const * const i2c, uint8_t const address, uint8_t const registerAddress, uint8_t * const dataBuffer, uint8_t const bufferLength);
typedef bme280_error_code_t (* bme280_i2c_write_t) (i2c_t const * const i2c, uint8_t const address, uint8_t const * const registerAddresses, uint8_t const * const dataBuffer, uint8_t const bufferLength);

typedef struct BME280_HANDLER
{
    bme280_i2c_read_t I2C_Read;
    bme280_i2c_write_t I2C_Write;
} bme280_handler_t;

typedef struct BME280_SETTINGS
{
    // Temperature oversampling
    bme280_oversampling_t temperatureOversampling;

    // Pressure oversampling
    bme280_oversampling_t pressureOversampling;

    // Humidity oversampling
    bme280_oversampling_t humidityOversampling;

    // IIR filter coefficient
    bme280_iir_filter_t iirFilterCoefficients;

    // Standby time
    bme280_standby_time_t standbyTime;

    // Power mode
    bme280_power_mode_t powerMode;
} bme280_settings_t;

typedef struct BME280_CALIBRATION_DATA
{
    // Calibration coefficients for the temperature sensor

    uint16_t temperatureCoef1;
    int16_t temperatureCoef2;
    int16_t temperatureCoef3;

    int32_t temperatureTemporary;

    // Calibration coefficients for the pressure sensor

    uint16_t pressureCoef1;
    int16_t pressureCoef2;
    int16_t pressureCoef3;
    int16_t pressureCoef4;
    int16_t pressureCoef5;
    int16_t pressureCoef6;
    int16_t pressureCoef7;
    int16_t pressureCoef8;
    int16_t pressureCoef9;

    // Calibration coefficients for the humidity sensor

    uint8_t humidityCoef1;
    int16_t humidityCoef2;
    uint8_t humidityCoef3;
    int16_t humidityCoef4;
    int16_t humidityCoef5;
    int8_t humidityCoef6;
} bme280_calibration_data_t;

typedef struct BME280_DATA
{
    // Compensated data for the temperature sensor
    int32_t temperature;

    // Compensated data for the pressure sensor
    uint32_t  pressure;

    // Compensated data for the humidity sensor
    uint32_t humidity;
} bme280_data_t;

typedef struct BME280_DEVICE
{
    // I2C address of the device
    uint8_t i2cAddress;

    // I2C device
    i2c_t const * i2cDevice;

    // Handler
    bme280_handler_t const * handler;

    // Sensor settings
    bme280_settings_t settings;

    // Calibration data
    bme280_calibration_data_t calibrationData;

    // Sensor data
    bme280_data_t data;

} bme280_device_t;

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                 Public API                                 //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Initializes the BME280 device.
 * @param[in, out] device BME280 device
 * @param[in] handler Read and write operations handler
 * @param[in] i2cDevice I2C device
 * @param[in] i2cAddress I2C device address
 * @param[in] settings The settings to set
 * @return bme280_error_code_t Error code
 * @retval BME280_OK If the device was initialized successfully
 * @retval BME280_DEVICE_NOT_FOUND If the device was not found on the I2C bus
 * @retval BME280_NULL_POINTER If the device, handler or I2C device is NULL
 * @note The device must be initialized before any other operation.
 **/
bme280_error_code_t BME280_Initialize(bme280_device_t * const device, bme280_handler_t const * const handler, i2c_t const * const i2cDevice, uint8_t const i2cAddress, bme280_settings_t const * const settings);

/**
 * @brief Reads the sensor data from the device and stores it in the sensor data
 *        structure.
 * @param[in, out] device BME280 device
 * @return bme280_error_code_t Error code
 * @retval BME280_OK If the sensor data was read successfully
 * @retval BME280_COMMUNICATION_ERROR If a communication error occurred
 * @retval BME280_NULL_POINTER If a NULL pointer was found
 * 
 **/
bme280_error_code_t BME280_GetSensorData(bme280_device_t * const device);

/**
 * @brief Returns the temperature from the last measurement.
 * @param[in] deviceData BME280 device sensor data
 * @return int32_t The temperature
 **/
int32_t BME280_GetTemperature(bme280_data_t const * const deviceData);

/**
 * @brief Returns the pressure from the last measurement.
 * @param[in] deviceData BME280 device sensor data
 * @return uint32_t The pressure
 **/
uint32_t BME280_GetPressure(bme280_data_t const * const deviceData);

/**
 * @brief Returns the humidity from the last measurement.
 * @param[in] deviceData BME280 device sensor data
 * @return uint32_t The humidity
 **/
uint32_t BME280_GetHuimidity(bme280_data_t const * const deviceData);

/**
 * @brief Returns the temperature from the last measurement in display format
 *        (as a floating point number).
 * @param[in] deviceData BME280 device sensor data
 * @return double The temperature
 **/
double BME280_GetDisplayTemperature(bme280_data_t const * const deviceData);

/**
 * @brief Returns the pressure from the last measurement in display format
 *        (as a floating point number).
 * @param[in] deviceData BME280 device sensor data
 * @return double The pressure
 **/
double BME280_GetDisplayPressure(bme280_data_t const * const deviceData);

/**
 * @brief Returns the humidity from the last measurement in display format
 *        (as a floating point number).
 * @param[in] deviceData BME280 device sensor data
 * @return double The humidity
 **/
double BME280_GetDisplayHumidity(bme280_data_t const * const deviceData);

/**
 * @brief Converts a vector to a @ref bme280_data_t structure.
 * @param[out] data BME280 structure to fill
 * @param[in] vector Vector to convert
 * @return None
 **/
void BME280_StructInterpret(void * const data, vector_t * const vector);

/**
 * @brief Converts a @ref bme280_data_t structure to serialized array of bytes.
 * @param[in] device BME280 device
 * @param[out] buffer Buffer to fill
 * @return None
 **/
void BME280_SerializeSensorData(bme280_device_t const * const device, uint8_t * const buffer);


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Modules                                   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

extern bme280_handler_t const BME280_I2C_Handler;


#endif // BME280_H
