/**
 * Abstract I2C Interface
 *
 * @author  David Hughes
 * @date    30Jun23
 */
#pragma once

#include <stdbool.h>    // bool
#include <stddef.h>     // size_t


/*
 * Public types
 */


/**
 * I2C communication primitive
 *
 * @param DeviceAddress The device address (right-justified)
 * @param pWData        Pointer to the data to send to the device
 * @param WDataSize     Number of bytes to write
 * @param pRData        Pointer to the buffer to receive data from the device
 * @param RDataSize     Number of bytes to read back
 * @return true if the operation completed ok; false otherwise
 */
typedef bool (*i2c_interface_t)(unsigned const DeviceAddress,
                                void* pWData, size_t const WDataSize,
                                void* pRData, size_t const RDataSize);
