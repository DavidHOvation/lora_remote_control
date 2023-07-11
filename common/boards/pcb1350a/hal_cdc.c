#include "hal_cdc.h"

#include <stdbool.h>    // bool


#include "conf_usb.h"

// From module: USB - Universal Serial Bus
#include <usb.h>

// From module: USB CDC Protocol
#include <usb_protocol_cdc.h>

// From module: USB Device CDC (Single Interface Device)
#include <udi_cdc.h>

// From module: USB Device Stack Core (Common API)
#include <udc.h>
#include <udd.h>

#include <stdio_serial.h>   // ptr_put



#include "util.h"       // util_clear


/*
 * Private data
 */
static volatile struct
{
    bool enabled;
    bool is_open;
    bool usb_mode_is_host;
}
_hal_cdc;


/*
 * Public functions
 */
void hal_cdc_init(void)
{
    util_clear((void*)&_hal_cdc, sizeof(_hal_cdc));

    ptr_put = hal_cdc_putc_null;
}


void hal_cdc_deinit(void)
{
    udc_stop();
}


void hal_cdc_start(void)
{
    udc_start();
}


bool hal_cdc_is_open(void)
{
    return _hal_cdc.is_open;
}


void hal_cdc_usb_mode_change(bool IsHostMode)
{
    _hal_cdc.usb_mode_is_host = IsHostMode;
}


void hal_cdc_vbus_event(bool VBUSIsHigh)
{
    if (VBUSIsHigh)
    {
        udc_attach();
    }
    else
    {
        udc_detach();
    }
}


bool hal_cdc_enable(uint8_t PortNo)
{
    (void) PortNo;

    _hal_cdc.enabled = true;
    return true;
}


void hal_cdc_disable(uint8_t PortNo)
{
    (void) PortNo;

    ptr_put = hal_cdc_putc_null;
    _hal_cdc.enabled = false;
}


void hal_cdc_set_dtr(uint8_t PortNo, bool Enable)
{
    (void) PortNo;

    if (Enable)
    {
        ptr_put = hal_cdc_putc;        // Re-link stdout to a working function
        _hal_cdc.is_open = true;
    }
    else
    {
        ptr_put = hal_cdc_putc_null;   // Re-link stdout to a dummy consumer
        _hal_cdc.is_open = false;
    }
}


int hal_cdc_putc(void volatile* pParam, char Value)
{
    (void) pParam;
    return udi_cdc_putc(Value);
}


int hal_cdc_putc_null(void volatile* pParam, char Value)
{
    (void) pParam;
    (void) Value;
    return 0;
}
