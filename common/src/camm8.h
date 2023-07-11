/**
 * u-blox CAM-M8 Driver (DDC / I2C)
 *
 * @author  David Hughes
 * @date    30Jun23
 */
#pragma once

#include <stdint.h>     // uint8_t
#include <stddef.h>     // size_t

#include "i2c_interface.h"


/*
 * Public constants
 */
#define CAMM8_ADDRESS        (0x42)


/*
 * Public types
 */
typedef struct
{
    i2c_interface_t i2c_rw;
}
camm8_t;


/*
 * Public functions
 */


/**
 * Initialises the module
 *
 * @param pSelf     Instance variable
 * @param I2C_RW    Function to use for communication with the hardware
 */
void camm8_init(camm8_t* const pSelf,
                i2c_interface_t I2C_RW);


/**
 * Reads from the hardware
 *
 * @param pSelf     Instance variable
 * @param pData     Pointer to the buffer to receive the data
 * @param DataLen   Number of bytes of the data buffer and that to receive
 * @return Number of bytes written to pData
 */
size_t camm8_read(camm8_t* const pSelf,
                  void* const pData,
                  size_t const DataLen);


/**
 * Reads from the hardware
 *
 * @param pSelf     Instance variable
 * @param pData     Pointer to the buffer of data to send
 * @param DataLen   Number of bytes of the data buffer and that to send
 */
void camm8_write(camm8_t* const pSelf,
                 void* const pData,
                 size_t const DataLen);
