#include "pca9539a.h"

#include <stdint.h>     // uint8_t
#include <stddef.h>     // NULL


/*
 * Public constants
 */
const pca9539a_pins_t pca9539_reg_output_default    = 0xFF;
const pca9539a_pins_t pca9539_reg_pol_inv_default   = 0x00;
const pca9539a_pins_t pca9539_reg_config_default    = 0xFF;


/*
 * Public functions
 */
void pca9539a_init(pca9539a_t* const pSelf,
                   i2c_interface_t I2C_RW,
                   uint8_t const Address)
{
    pSelf->i2c_rw = I2C_RW;
    pSelf->address = Address;
    
    // "reset"
    {
        uint8_t p_write_data[] =
        {
            pca9539a_reg_output_port_0,
            (uint8_t) pca9539_reg_output_default,   // Port 0
            (uint8_t) pca9539_reg_output_default,   // Port 1
        };

        pSelf->i2c_rw(pSelf->address,
                      p_write_data, sizeof(p_write_data),
                      NULL, 0);
    }                      

    {
        uint8_t p_write_data[] =
        {
            pca9539a_reg_pol_inv_port_0,
            (uint8_t) pca9539_reg_pol_inv_default,  // Port 0
            (uint8_t) pca9539_reg_pol_inv_default,  // Port 1
        };

        pSelf->i2c_rw(pSelf->address,
                      p_write_data, sizeof(p_write_data),
                      NULL, 0);
    }
    
    {
        uint8_t p_write_data[] =
        {
            pca9539a_reg_config_port_0,
            (uint8_t) pca9539_reg_config_default,   // Port 0
            (uint8_t) pca9539_reg_config_default,   // Port 1
        };

        pSelf->i2c_rw(pSelf->address,
                      p_write_data, sizeof(p_write_data),
                      NULL, 0);
    }
}


pca9539a_pins_t pca9539a_read(pca9539a_t* const pSelf,
                              pca9539a_reg_t const RegID)
{
    uint8_t p_write_data[1] = { (uint8_t) RegID };
    uint8_t p_read_data[1] = { 0 };

    pSelf->i2c_rw(pSelf->address,
                  p_write_data, sizeof(p_write_data),
                  p_read_data, sizeof(p_read_data));

    return (pca9539a_pins_t) p_read_data[0];
}


void pca9539a_write(pca9539a_t* const pSelf,
                    pca9539a_reg_t const RegID,
                    pca9539a_pins_t const Value)
{
    uint8_t p_write_data[2] = { (uint8_t) RegID, (uint8_t) Value };

    pSelf->i2c_rw(pSelf->address,
                  p_write_data, sizeof(p_write_data),
                  NULL, 0);
}


const char* pca9539a_reg_str(pca9539a_reg_t const Value)
{
    const char* p_str;

    switch (Value)
    {
    case pca9539a_reg_input_port_0:     p_str = "pca9539a_reg_input_port_0";    break;
    case pca9539a_reg_input_port_1:     p_str = "pca9539a_reg_input_port_1";    break;
    case pca9539a_reg_output_port_0:    p_str = "pca9539a_reg_output_port_0";   break;
    case pca9539a_reg_output_port_1:    p_str = "pca9539a_reg_output_port_1";   break;
    case pca9539a_reg_pol_inv_port_0:   p_str = "pca9539a_reg_pol_inv_port_0";  break;
    case pca9539a_reg_pol_inv_port_1:   p_str = "pca9539a_reg_pol_inv_port_1";  break;
    case pca9539a_reg_config_port_0:    p_str = "pca9539a_reg_config_port_0";   break;
    case pca9539a_reg_config_port_1:    p_str = "pca9539a_reg_config_port_1";   break;
    default:                            p_str = "pca9539a_reg <unknown>";       break;
    }

    return p_str;
}
