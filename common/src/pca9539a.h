/**
 * NXP PCA9539A Driver
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
#define PCA9539A_ADDRESS        (0x74) // to 0x77 (= + 2*A1 + A0)


/*
 * Public types
 */
typedef struct 
{
    i2c_interface_t i2c_rw;
    uint8_t address;
}
pca9539a_t;


typedef enum
{
    pca9539a_reg_input_port_0      = 0, // RO
    pca9539a_reg_input_port_1      = 1, // RO

    pca9539a_reg_output_port_0     = 2, // RW
    pca9539a_reg_output_port_1     = 3, // RW

    pca9539a_reg_pol_inv_port_0    = 4, // RW
    pca9539a_reg_pol_inv_port_1    = 5, // RW

    pca9539a_reg_config_port_0     = 6, // RW
    pca9539a_reg_config_port_1     = 7, // RW
}
pca9539a_reg_t;

typedef uint8_t pca9539a_pins_t;    // bit mask of the pins

extern const pca9539a_pins_t pca9539_reg_output_default;
extern const pca9539a_pins_t pca9539_reg_pol_inv_default;
extern const pca9539a_pins_t pca9539_reg_config_default;


/*
 * Public functions
 */
void pca9539a_init(pca9539a_t* const pSelf,
                   i2c_interface_t I2C_RW,
                   uint8_t const Address);


/**
 * Reads from a register
 *
 * @param Address   Device I2C address
 * @param RegID     Index of the register
 * @return The value of the register
 */
pca9539a_pins_t pca9539a_read(pca9539a_t* const pSelf,
                              pca9539a_reg_t const RegID);


/**
 * Writes to a register
 *
 * @param RegID Index of the register
 * @param Value The value to send
 */
void pca9539a_write(pca9539a_t* const pSelf,
                    pca9539a_reg_t const RegID,
                    pca9539a_pins_t const Value);


/*
 * Enum to String conversions
 */
const char* pca9539a_reg_str(pca9539a_reg_t const Value);
