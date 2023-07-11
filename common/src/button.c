#include "Button.h"

#include <stdint.h>     // uint32_t
#include <stdbool.h>    // bool

#include "util.h"       // util_clear


/*
 * Private data
 */
static struct
{
    // Interrupt-only variables
    uint32_t short_press_count;
    uint32_t long_press_count;
    uint32_t release_count;
}
_button;


/*
 * Public functions
 */
void button_init_class(uint32_t const ShortPressCount,
                       uint32_t const LongPressCount,
                       uint32_t const ReleaseRecoverCount)
{
    _button.short_press_count   = ShortPressCount;
    _button.long_press_count    = LongPressCount;
    _button.release_count       = ReleaseRecoverCount;
}


void button_init(button_t* const pSelf)
{
    util_clear(pSelf, sizeof(button_t));
}


bool button_has_short_press(button_t* const pSelf)
{
    const bool ret = pSelf->short_press;
    pSelf->short_press  = false;
    return ret;
}


bool button_has_long_press(button_t* const pSelf)
{
    const bool ret = pSelf->long_press;
    pSelf->long_press   = false;
    return ret;
}


bool button_is_busy(button_t* const pSelf)
{
    return pSelf->is_busy;
}


void button_run(button_t* const pSelf,
                bool const IsPressed)
{
    bool down_short   = false;
    bool down_long    = false;

    /*
     * Button has to be stable for at least the times given to create an event
     * The button has a hold-off delay before it will respond to new presses.
     */
    if (IsPressed)
    {
        pSelf->is_busy = true;
        pSelf->up_count = 0;

        if (pSelf->down_count < _button.long_press_count)
        {
            pSelf->down_count++;
        }
        else
        {
            down_long = true;
        }
    }
    else    // Released
    {
        if ((pSelf->down_count >= _button.short_press_count) &&
            (pSelf->down_count <  _button.long_press_count))
        {
            down_short = true;
        }

        pSelf->down_count = 0;

        if (pSelf->up_count < _button.release_count)
        {
            pSelf->up_count++;
        }
        else
        {
            pSelf->is_busy = false;
        }
    }


    if (down_short && !pSelf->down_short_last)
    {
        pSelf->short_press = true;
    }


    if (down_long && !pSelf->down_long_last)
    {
        pSelf->long_press = true;
    }


    pSelf->down_short_last = down_short;
    pSelf->down_long_last  = down_long;
}
