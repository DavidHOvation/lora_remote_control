/**
 * Receiver User Interface - buttons and LEDs
 *
 * @author  David Hughes
 * @date    23Jun23
 */
#pragma once

#include <stdint.h>     // uint32_t
#include <stdbool.h>    // bool
#include <stddef.h>     // size_t


/*
 * Public types
 */
typedef enum
{
    ReceiverUI_colour_black,  // AKA off
    ReceiverUI_colour_green,
    ReceiverUI_colour_red,
    ReceiverUI_colour_yellow,
}
ReceiverUI_colour_t;


/*
 * Public functions
 */


/**
 * module initialiser
 */
void ReceiverUI_init(void);


/**
 * Queries the button state
 *
 * @return The value requested
 */
bool ReceiverUI_button_is_pressed(void);


/**
 * Sets the status LED
 * @note Only red, green, yellow and black are supported
 *
 * @param Colour    The colour to set
 */
void ReceiverUI_led_status_set(ReceiverUI_colour_t const Colour);


/**
 * Utility to convert the colour to a string
 *
 * @param Value     Value to convert
 * @return The value requested
 */
const char* ReceiverUI_colour_str(ReceiverUI_colour_t const Value);

