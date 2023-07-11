#include "ioexpander.h"

#include <stdbool.h>     // bool

#include "pca9539a.h"


/*
 * Private data
 */
typedef struct
{
    bool modified;
    pca9539a_pins_t value;
}
_ioexpander_reg_t;


static struct
{
    pca9539a_t* p_dev;

    pca9539a_pins_t   p_reg_input[2];
    _ioexpander_reg_t p_reg_output[2];
    _ioexpander_reg_t p_reg_dir[2];
}
_ioexpander;


/*
 * Public functions
 */
void ioexpander_init(pca9539a_t* const pDev)
{
    _ioexpander.p_dev = pDev;

    _ioexpander.p_reg_input[0] =
        pca9539a_read(_ioexpander.p_dev, pca9539a_reg_input_port_0);

    _ioexpander.p_reg_input[1] =
        pca9539a_read(_ioexpander.p_dev, pca9539a_reg_input_port_1);

    _ioexpander.p_reg_output[0].value   =
        pca9539a_read(_ioexpander.p_dev, pca9539a_reg_output_port_0);
    _ioexpander.p_reg_output[0].modified = false;

    _ioexpander.p_reg_output[1].value   =
        pca9539a_read(_ioexpander.p_dev, pca9539a_reg_output_port_1);
    _ioexpander.p_reg_output[1].modified = false;

    _ioexpander.p_reg_dir[0].value      =
        pca9539a_read(_ioexpander.p_dev, pca9539a_reg_config_port_0);
    _ioexpander.p_reg_dir[0].modified   = false;

    _ioexpander.p_reg_dir[1].value      =
        pca9539a_read(_ioexpander.p_dev, pca9539a_reg_config_port_1);
    _ioexpander.p_reg_dir[1].modified   = false;
}


void ioexpander_set_direction_input(ioexpander_pin_t const PinNo)
{
    uint8_t const pin_mask = 1 << (PinNo % 8);
    _ioexpander_reg_t* const p_reg = &_ioexpander.p_reg_dir[PinNo / 8];

    p_reg->value    |= pin_mask;
    p_reg->modified = true;
}


void ioexpander_set_direction_output(ioexpander_pin_t const PinNo)
{
    uint8_t const pin_mask = 1 << (PinNo % 8);
    _ioexpander_reg_t* const p_reg = &_ioexpander.p_reg_dir[PinNo / 8];

    p_reg->value    &= ~pin_mask;
    p_reg->modified = true;
}


bool ioexpander_get_input_level(ioexpander_pin_t const PinNo)
{
    uint8_t const pin_mask = 1 << (PinNo % 8);
    pca9539a_pins_t* const p_reg = &_ioexpander.p_reg_input[PinNo / 8];

    return (*p_reg & pin_mask) != 0;
}


bool ioexpander_get_output_level(ioexpander_pin_t const PinNo)
{
    uint8_t const pin_mask = 1 << (PinNo % 8);
    _ioexpander_reg_t* const p_reg = &_ioexpander.p_reg_output[PinNo / 8];

    return (p_reg->value & pin_mask) != 0;
}


void ioexpander_set_output_level(ioexpander_pin_t const PinNo,
                                 bool const HighLevel)
{
    uint8_t const pin_mask = 1 << (PinNo % 8);
    _ioexpander_reg_t* const p_reg = &_ioexpander.p_reg_output[PinNo / 8];

    if (HighLevel)
    {
        p_reg->value |= pin_mask;
    }
    else
    {
        p_reg->value &= ~pin_mask;
    }

    p_reg->modified = true;
}


void ioexpander_sync_hardware(void)
{
    _ioexpander.p_reg_input[0] = pca9539a_read(_ioexpander.p_dev,
                                               pca9539a_reg_input_port_0);

    _ioexpander.p_reg_input[1] = pca9539a_read(_ioexpander.p_dev,
                                               pca9539a_reg_input_port_1);

    if (_ioexpander.p_reg_output[0].modified)
    {
        pca9539a_write(_ioexpander.p_dev,
                       pca9539a_reg_output_port_0,
                       _ioexpander.p_reg_output[0].value);
        _ioexpander.p_reg_output[0].modified = false;
    }

    if (_ioexpander.p_reg_output[1].modified)
    {
        pca9539a_write(_ioexpander.p_dev,
                       pca9539a_reg_output_port_1,
                       _ioexpander.p_reg_output[1].value);
        _ioexpander.p_reg_output[1].modified = false;
    }

    if (_ioexpander.p_reg_dir[0].modified)
    {
        pca9539a_write(_ioexpander.p_dev,
                       pca9539a_reg_config_port_0,
                       _ioexpander.p_reg_dir[0].value);
        _ioexpander.p_reg_dir[0].modified = false;
    }

    if (_ioexpander.p_reg_dir[1].modified)
    {
        pca9539a_write(_ioexpander.p_dev,
                       pca9539a_reg_config_port_1,
                       _ioexpander.p_reg_dir[1].value);
        _ioexpander.p_reg_dir[1].modified = false;
    }
}
