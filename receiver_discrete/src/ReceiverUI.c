#include "ReceiverUI.h"

#include <stdint.h>     // uint32_t
#include <stdbool.h>    // bool
#include <stddef.h>     // size_t


#include "util.h"       // util_clear
#include <hal-pcb1350a.h>


/*
 * Private constants
 */
static unsigned const _ReceiverUI_red_mask        = 1 << 2;
static unsigned const _ReceiverUI_green_mask      = 1 << 1;


/*
 * Private functions
 */
unsigned _ReceiverUI_get_rgb(ReceiverUI_colour_t const Colour);


/*
 * Public functions
 */
void ReceiverUI_init(void)
{
    ReceiverUI_led_status_set(ReceiverUI_colour_black);
}


bool ReceiverUI_button_is_pressed(void)
{
    return hal_button_is_pressed(hal_button_button);
}


void ReceiverUI_led_status_set(ReceiverUI_colour_t const Colour)
{
    unsigned const rg = _ReceiverUI_get_rgb(Colour);
    bool const red_on   = (rg & _ReceiverUI_red_mask) != 0;
    bool const green_on = (rg & _ReceiverUI_green_mask) != 0;

    hal_led_stat_set(red_on, green_on);
}


/*
 * Private functions
 */
unsigned _ReceiverUI_get_rgb(ReceiverUI_colour_t const Colour)
{
    unsigned rg;

    unsigned const red   = _ReceiverUI_red_mask;
    unsigned const green = _ReceiverUI_green_mask;

    switch (Colour)
    {
        default:
        case ReceiverUI_colour_black:   rg = 0;                    break;
        case ReceiverUI_colour_green:   rg = green;                break;
        case ReceiverUI_colour_red:     rg = red;                  break;
        case ReceiverUI_colour_yellow:  rg = red | green;          break;
    }

    return rg;
}


const char* ReceiverUI_colour_str(ReceiverUI_colour_t const Value)
{
    const char* str;

    switch (Value)
    {
    case ReceiverUI_colour_black:   str = "ReceiverUI_colour_black";      break;
    case ReceiverUI_colour_green:   str = "ReceiverUI_colour_green";      break;
    case ReceiverUI_colour_red:     str = "ReceiverUI_colour_red";        break;
    case ReceiverUI_colour_yellow:  str = "ReceiverUI_colour_yellow";     break;
    default:                        str = "ReceiverUI_colour <unknown>";  break;
    }

    return str;
}
