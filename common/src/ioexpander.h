/**
 * NXP PCA9539A Driver
 * Static class!
 *
 * @author  David Hughes
 * @date    30Jun23
 */
#pragma once

#include <stdbool.h>     // bool

#include "pca9539a.h"


/*
 * Public types
 */
typedef unsigned ioexpander_pin_t;
#define IOEXPANDER_IO0_0    ((ioexpander_pin_t) 0)
#define IOEXPANDER_IO0_1    ((ioexpander_pin_t) 1)
#define IOEXPANDER_IO0_2    ((ioexpander_pin_t) 2)
#define IOEXPANDER_IO0_3    ((ioexpander_pin_t) 3)
#define IOEXPANDER_IO0_4    ((ioexpander_pin_t) 4)
#define IOEXPANDER_IO0_5    ((ioexpander_pin_t) 5)
#define IOEXPANDER_IO0_6    ((ioexpander_pin_t) 6)
#define IOEXPANDER_IO0_7    ((ioexpander_pin_t) 7)
#define IOEXPANDER_IO1_0    ((ioexpander_pin_t) 8)
#define IOEXPANDER_IO1_1    ((ioexpander_pin_t) 9)
#define IOEXPANDER_IO1_2    ((ioexpander_pin_t) 10)
#define IOEXPANDER_IO1_3    ((ioexpander_pin_t) 11)
#define IOEXPANDER_IO1_4    ((ioexpander_pin_t) 12)
#define IOEXPANDER_IO1_5    ((ioexpander_pin_t) 13)
#define IOEXPANDER_IO1_6    ((ioexpander_pin_t) 14)
#define IOEXPANDER_IO1_7    ((ioexpander_pin_t) 15)


/*
 * Public functions
 */


/**
 * Module initialiser
 *
 * @param pDev  PCA9539A device to use for communication
 */
void ioexpander_init(pca9539a_t* const pDev);


/**
 * Sets the pin as an input
 *
 * @param PinNo The number of the pin to use
 */
void ioexpander_set_direction_input(ioexpander_pin_t const PinNo);


/**
 * Sets the pin as an output
 *
 * @param PinNo The number of the pin to use
 */
void ioexpander_set_direction_output(ioexpander_pin_t const PinNo);


/**
 * Queries if the pin is a high-level
 * @note Need to call ioexpander_sync_hardware to get the latest readings
 *
 * @param PinNo The number of the pin to use
 * @return The true if so; false otherwise
 */
bool ioexpander_get_input_level(ioexpander_pin_t const PinNo);


/**
 * Queries if the pin is commanded to a high-level
 * @note Need to call ioexpander_sync_hardware to get the latest readings
 *
 * @param PinNo The number of the pin to use
 * @return The true if so; false otherwise
 */
bool ioexpander_get_output_level(ioexpander_pin_t const PinNo);


/**
 * Sets the value of the pin
 * @note Need to call ioexpander_sync_hardware to set the pin
 *
 * @param PinNo The number of the pin to use
 * @param HighLevel true if the pin is high; false for low
 */
void ioexpander_set_output_level(ioexpander_pin_t const PinNo,
                                 bool const HighLevel);


/**
 * Sets the pending operations to the chip and retrieves the input state
 */
void ioexpander_sync_hardware(void);
