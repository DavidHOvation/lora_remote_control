#include "keyfobUI.h"

#include <stdint.h>     // uint32_t
#include <stdbool.h>    // bool
#include <stddef.h>     // size_t


#include "util.h"       // util_clear
#include "button.h"
#include <hal-pcb1350a.h>


/*
 * Private constants
 */
static uint32_t const _keyfobUI_press_count     = 5;   // interrupt ticks
static uint32_t const _keyfobUI_longpress_count = 50;
static uint32_t const _keyfobUI_release_count   = 10;

static unsigned const _keyfobUI_red_mask        = 1 << 2;
static unsigned const _keyfobUI_green_mask      = 1 << 1;
static unsigned const _keyfobUI_blue_mask       = 1 << 0;


/*
 * Private data
 */
typedef struct
{
    uint32_t press_count;
    uint32_t release_count;
    bool     pressed;
    bool     pressed_long;
}
_keyfobUI_button_t;


static struct
{
    button_t on_button;
    button_t off_button;
    button_t record_button;
    button_t stream_button;

    volatile struct
    {
        bool                            running;
        size_t                          index;
        size_t                          index_max;
        keyfobUI_led_seq_ele_t const*   p_seq;
        uint32_t                        repeat_count;
        uint32_t                        delay_count;
    }
    led_sequence;
}
_keyfobUI;


/*
 * Private functions
 */
void _keyfobUI_led_sequence_run(void);


unsigned _keyfobUI_get_rgb(keyfobUI_colour_t const Colour);


void _keyfobUI_led_rgb_colour_set(keyfobUI_colour_t const Colour);


/*
 * Public functions
 */
void keyfobUI_init(void)
{
    util_clear(&_keyfobUI, sizeof(_keyfobUI));

    button_init_class(_keyfobUI_press_count,
                      _keyfobUI_longpress_count,
                      _keyfobUI_release_count);

    button_init(&_keyfobUI.on_button);
    button_init(&_keyfobUI.off_button);
    button_init(&_keyfobUI.record_button);
    button_init(&_keyfobUI.stream_button);

    _keyfobUI_led_rgb_colour_set(keyfobUI_colour_black);
    keyfobUI_led_status_set(keyfobUI_colour_black);
    
    hal_periodic_set_period(10);
    hal_periodic_set_callback(keyfobUI_interrupt_task);
    hal_periodic_enable(true);
}


bool keyfobUI_button_is_busy(void)
{
    return  button_is_busy(&_keyfobUI.on_button)        ||
            button_is_busy(&_keyfobUI.off_button)       ||
            button_is_busy(&_keyfobUI.record_button)    ||
            button_is_busy(&_keyfobUI.stream_button);
}


keyfobUI_button_t keyfobUI_button_get(void)
{
    keyfobUI_button_t ret;

    bool const on_button_short = button_has_short_press(&_keyfobUI.on_button);
    bool const on_button_long = button_has_long_press(&_keyfobUI.on_button);

    bool const off_button_short = button_has_short_press(&_keyfobUI.off_button);
    //bool const off_button_long = button_has_long_press(&_keyfobUI.off_button);

    bool const record_button_short = button_has_short_press(&_keyfobUI.record_button);
    bool const record_button_long = button_has_long_press(&_keyfobUI.record_button);

    bool const stream_button_short = button_has_short_press(&_keyfobUI.stream_button);
    bool const stream_button_long = button_has_long_press(&_keyfobUI.stream_button);

    if (on_button_short)
    {
        ret = keyfobUI_button_on;
    }
    else if (on_button_long)
    {
        ret = keyfobUI_button_off;
    }
    else if (off_button_short)
    {
        ret = keyfobUI_button_status;
    }
    else if (record_button_short)
    {
        ret = keyfobUI_button_record;
    }
    else if (record_button_long)
    {
        if (stream_button_short)
        {
            ret = keyfobUI_button_record_stream;
        }
        else
        {
            ret = keyfobUI_button_off_record;
        }
    }
    else if (stream_button_short)
    {
        ret = keyfobUI_button_stream;
    }
    else if (stream_button_long)
    {
        if (record_button_short)
        {
            ret = keyfobUI_button_record_stream;
        }
        else
        {
            ret = keyfobUI_button_off_stream;
        }
    }
    else
    {
        ret = keyfobUI_button_none;
    }

    return ret;
}


void keyfobUI_led_status_set(keyfobUI_colour_t const Colour)
{
    unsigned const rgb = _keyfobUI_get_rgb(Colour);
    bool const red_on   = (rgb & _keyfobUI_red_mask) != 0;
    bool const green_on = (rgb & _keyfobUI_green_mask) != 0;

    hal_interrupt_enable_global(false);
    bool const sequence_was_running = _keyfobUI.led_sequence.running;
    _keyfobUI.led_sequence.running = false;
    hal_interrupt_enable_global(true);

    hal_led_stat_set(red_on, green_on);

    _keyfobUI.led_sequence.running = sequence_was_running;
}


void keyfobUI_led_sequence_start(keyfobUI_led_sequence_t const* const pSequence)
{
    _keyfobUI.led_sequence.running      = false;    // mutex
    
    _keyfobUI.led_sequence.index        = pSequence->length;   // Immediate change
    _keyfobUI.led_sequence.index_max    = pSequence->length - 1;
    _keyfobUI.led_sequence.p_seq        = pSequence->p_seq;
    _keyfobUI.led_sequence.delay_count  = 0;
    _keyfobUI.led_sequence.repeat_count = pSequence->repeat_count;
    
    _keyfobUI.led_sequence.running      = true;
}


bool keyfobUI_led_sequence_is_busy(void)
{
    return _keyfobUI.led_sequence.running;
}


void keyfobUI_interrupt_task(void)
{
    bool const on_pressed       = hal_button_is_pressed(hal_button_switch3);
    bool const off_pressed      = hal_button_is_pressed(hal_button_switch4);
    bool const record_pressed   = hal_button_is_pressed(hal_button_switch1);
    bool const stream_pressed   = hal_button_is_pressed(hal_button_switch2);

    button_run(&_keyfobUI.on_button,     on_pressed);
    button_run(&_keyfobUI.off_button,    off_pressed);
    button_run(&_keyfobUI.record_button, record_pressed);
    button_run(&_keyfobUI.stream_button, stream_pressed);

    if (_keyfobUI.led_sequence.running)
    {
        _keyfobUI_led_sequence_run();
    }
}


/*
 * Private functions
 */
void _keyfobUI_led_sequence_run(void)
{
    if (_keyfobUI.led_sequence.delay_count > 0)
    {
        _keyfobUI.led_sequence.delay_count--;
    }
    else
    {
        if (_keyfobUI.led_sequence.index < _keyfobUI.led_sequence.index_max)
        {
            _keyfobUI.led_sequence.index++;
        }
        else
        {        
            _keyfobUI.led_sequence.index = 0;
            
            if (_keyfobUI.led_sequence.repeat_count > 0)
            {
                _keyfobUI.led_sequence.repeat_count--;
            }
            else
            {                
                _keyfobUI_led_rgb_colour_set(keyfobUI_colour_black);
                _keyfobUI.led_sequence.running = false;
                return;
            }
        }

        keyfobUI_led_seq_ele_t const* const p_seq =
                &_keyfobUI.led_sequence.p_seq[_keyfobUI.led_sequence.index];

        _keyfobUI_led_rgb_colour_set(p_seq->colour);

        _keyfobUI.led_sequence.delay_count = p_seq->delay_msec;
    }
}


unsigned _keyfobUI_get_rgb(keyfobUI_colour_t const Colour)
{
    unsigned rgb;

    unsigned const red   = _keyfobUI_red_mask;
    unsigned const green = _keyfobUI_green_mask;
    unsigned const blue  = _keyfobUI_blue_mask;

    switch (Colour)
    {
        default:
        case keyfobUI_colour_black:     rgb = 0;                    break;
        case keyfobUI_colour_blue:      rgb = blue;                 break;
        case keyfobUI_colour_green:     rgb = green;                break;
        case keyfobUI_colour_cyan:      rgb = green | blue;         break;
        case keyfobUI_colour_red:       rgb = red;                  break;
        case keyfobUI_colour_magenta:   rgb = red | blue;           break;
        case keyfobUI_colour_yellow:    rgb = red | green;          break;
        case keyfobUI_colour_white:     rgb = red | green | blue;   break;
    }

    return rgb;
}


void _keyfobUI_led_rgb_colour_set(keyfobUI_colour_t const Colour)
{
    unsigned const rgb = _keyfobUI_get_rgb(Colour);
    bool const red_on = (rgb & _keyfobUI_red_mask) != 0;
    bool const green_on = (rgb & _keyfobUI_green_mask) != 0;
    bool const blue_on = (rgb & _keyfobUI_blue_mask) != 0;

    hal_led_rgb_set(red_on, green_on, blue_on);
}


const char* keyfobUI_button_str(keyfobUI_button_t const Value)
{
    const char* str;

    switch (Value)
    {
    case keyfobUI_button_none:          str = "keyfobUI_button_none";           break;
    case keyfobUI_button_on:            str = "keyfobUI_button_on";             break;
    case keyfobUI_button_off:           str = "keyfobUI_button_off";            break;
    case keyfobUI_button_status:        str = "keyfobUI_button_status";         break;
    case keyfobUI_button_record:        str = "keyfobUI_button_record";         break;
    case keyfobUI_button_stream:        str = "keyfobUI_button_stream";         break;
    case keyfobUI_button_off_record:    str = "keyfobUI_button_off_record";     break;
    case keyfobUI_button_off_stream:    str = "keyfobUI_button_off_stream";     break;
    case keyfobUI_button_record_stream: str = "keyfobUI_button_record_stream";  break;
    default:                            str = "keyfobUI_button <unknown>";      break;
    }

    return str;
}


const char* keyfobUI_colour_str(keyfobUI_colour_t const Value)
{
    const char* str;

    switch (Value)
    {
    case keyfobUI_colour_black:     str = "keyfobUI_colour_black";      break;
    case keyfobUI_colour_blue:      str = "keyfobUI_colour_blue";       break;
    case keyfobUI_colour_green:     str = "keyfobUI_colour_green";      break;
    case keyfobUI_colour_cyan:      str = "keyfobUI_colour_cyan";       break;
    case keyfobUI_colour_red:       str = "keyfobUI_colour_red";        break;
    case keyfobUI_colour_magenta:   str = "keyfobUI_colour_magenta";    break;
    case keyfobUI_colour_yellow:    str = "keyfobUI_colour_yellow";     break;
    case keyfobUI_colour_white:     str = "keyfobUI_colour_white";      break;
    default:                        str = "keyfobUI_colour <unknown>";  break;
    }

    return str;
}

