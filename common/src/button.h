/**
 * Button-handling
 *
 * @author  David Hughes
 * @date    30Jun23
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>


/*
 * Public types
 */
typedef struct
{
    // Interrupt only
    volatile uint32_t   down_count;
    volatile uint32_t   up_count;

    volatile bool       down_short_last;
    volatile bool       down_long_last;

    // Interface variables
    volatile bool       long_press;
    volatile bool       short_press;
    volatile bool       is_busy;
}
button_t;


/*
 * Public functions
 */


/**
 * Initialises the class and sets the critical timings
 *
 * @param ShortPressCount       Min short-press in button_run invocations
 * @param LongPressCount        Min long press in button_run invocations
 * @param ReleaseRecoverCount   Min released time in button_run invocations
 */
void button_init_class(uint32_t const ShortPressCount,
                       uint32_t const LongPressCount,
                       uint32_t const ReleaseRecoverCount);


/**
 * Initialises an instance
 *
 * @param pSelf Pointer to the instance variable
 */
void button_init(button_t* const pSelf);


/**
 * Returns true if the button was pressed between ShortPressCount and
 * LongPressCount after being released ReleaseRecoverCount
 *
 * @param pSelf Pointer to the instance variable
 * @return true if so; false otherwise
 */
bool button_has_short_press(button_t* const pSelf);


/**
 * Returns true if the button was pressed for LongPressCount after being
 * released ReleaseRecoverCount
 *
 * @param pSelf Pointer to the instance variable
 * @return true if so; false otherwise
 */
bool button_has_long_press(button_t* const pSelf);


/**
 * Used to determine whether there is a short or long press pending
 *
 * @param pSelf Pointer to the instance variable
 * @return true if so; false otherwise
 */
bool button_is_busy(button_t* const pSelf);


/**
 * Module executable. Needs to be run at a constant rate
 * @note Intended to be executed from a periodic interrupt routine
 *
 * @param pSelf     Pointer to the instance variable
 * @param IsPressed Current pressed value of the hardware
 */
void button_run(button_t* const pSelf,
                bool const IsPressed);
