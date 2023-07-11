/**
 * Keyfob User Interface - buttons and LEDs
 *
 * @author  David Hughes
 * @date    30Jun23
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
    keyfobUI_button_none,
    keyfobUI_button_on,
    keyfobUI_button_off,
    keyfobUI_button_status,
    keyfobUI_button_record,
    keyfobUI_button_stream,
    keyfobUI_button_off_record,
    keyfobUI_button_off_stream,
    keyfobUI_button_record_stream,
}
keyfobUI_button_t;


typedef enum
{
    keyfobUI_colour_black,  // AKA off
    keyfobUI_colour_blue,
    keyfobUI_colour_green,
    keyfobUI_colour_cyan,
    keyfobUI_colour_red,
    keyfobUI_colour_magenta,
    keyfobUI_colour_yellow,
    keyfobUI_colour_white,
}
keyfobUI_colour_t;


typedef struct
{
    keyfobUI_colour_t   colour;
    uint32_t            delay_msec;
}
keyfobUI_led_seq_ele_t;


typedef struct
{
    uint32_t repeat_count;
    uint32_t length;
    keyfobUI_led_seq_ele_t p_seq[];
}
keyfobUI_led_sequence_t;


/*
 * Public functions
 */


/**
 * module initialiser
 */
void keyfobUI_init(void);


/**
 * Queries the button state
 *
 * @return The value requested
 */
keyfobUI_button_t keyfobUI_button_get(void);


/**
 * Queries whether the button detection is on-going
 *
 * @return true if so; false otherwise
 */
bool keyfobUI_button_is_busy(void);


/**
 * Sets the status LED
 * @note Only red, green, yellow and black are supported
 *
 * @param Colour    The colour to set
 */
void keyfobUI_led_status_set(keyfobUI_colour_t const Colour);


/**
 * Sets the sequence for the LEDs. This will repeat until stopped.
 * @note It is expected that the sequence generator isn't busy
 *
 * @param NumRepeats        Number of times the sequence should repeat
 * @param pSequence         Pointer to the array of sequence elements
 * @param SequenceLength    Number of elements in the sequence array
 */
void keyfobUI_led_sequence_start(keyfobUI_led_sequence_t const* const pSequence);


/**
 * Queries whether the led sequence generator is in the middle of a sequence
 *
 * @return true if so; false otherwise
 */
bool keyfobUI_led_sequence_is_busy(void);


/**
 * Interrupt context task. must be called every 10 msec
 */
void keyfobUI_interrupt_task(void);


/**
 * Utility to convert the button state to a string
 *
 * @param Value     Value to convert
 * @return The value requested
 */
const char* keyfobUI_button_str(keyfobUI_button_t const Value);


/**
 * Utility to convert the colour to a string
 *
 * @param Value     Value to convert
 * @return The value requested
 */
const char* keyfobUI_colour_str(keyfobUI_colour_t const Value);

