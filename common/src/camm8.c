#include "camm8.h"

#include <stdint.h>     // uint8_t
#include <stddef.h>     // NULL, size_t
#include <string.h>     // memcpy
#include <assert.h>

#include "util.h"       // util_bytes_to_uint16_be, UTIL_MIN


/*
 * Public functions
 */
void camm8_init(camm8_t* const pSelf,
                i2c_interface_t I2C_RW)
{
    assert(pSelf != NULL);

    pSelf->i2c_rw = I2C_RW;
}


size_t camm8_read(camm8_t* const pSelf,
                  void* const pData,
                  size_t const DataLen)
{
    assert(pSelf != NULL);
    assert(pData != NULL);
    assert(DataLen > 0);

    uint8_t p_write_data[1] = { 0xFD }; // in waiting msb register
    uint8_t p_read_data[2] = { 0 };

    pSelf->i2c_rw(CAMM8_ADDRESS,
                  p_write_data, sizeof(p_write_data),
                  p_read_data, sizeof(p_read_data));

    size_t const data_len_waiting = util_bytes_to_uint16_be(p_read_data);

    if (data_len_waiting == 0) return 0;

    size_t read_len = UTIL_MIN(data_len_waiting, DataLen);

    (void) pSelf->i2c_rw(CAMM8_ADDRESS,
                         NULL, 0,  // Using default register 0xFF (Data)
                         pData, read_len);

    return read_len;
}


void camm8_write(camm8_t* const pSelf,
                 void* const pData,
                 size_t const DataLen)
{
    assert(pSelf != NULL);
    assert(pData != NULL);
    assert(DataLen > 0);

    (void) pSelf->i2c_rw(CAMM8_ADDRESS,
                         pData, DataLen,
                         NULL, 0);
}
