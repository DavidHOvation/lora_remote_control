/**
 * Ovation HAL interface including PCB 1350A definitions
 *
 */
#pragma once

#include "pcb1350a.h"


#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>     // size_t


/*
 * Public types
 */


/*
 * Public functions
 */

void hal_cdc_init(void);


void hal_cdc_deinit(void);


void hal_cdc_start(void);


bool hal_cdc_is_open(void);


/**
 * Callback when the USB mode has been switched automatically.
 * This is possible only when ID pin is available.
 *
 * @param b_host_mode true, if the host mode has been selected
 */
void hal_cdc_usb_mode_change(bool IsHostMode);


/**
 * Called by the USB stack when VBUS has changed
 *
 * @param VBUSIsHigh    New VBUS state
 */
void hal_cdc_vbus_event(bool VBUSIsHigh);


/**
 * Called when CDC has been enabled
 *
 * @param PortNo    The CDC port
 * @return true if ok to enable; false otherwise
 */
bool hal_cdc_enable(uint8_t PortNo);


/**
 * Called when CDC has been disabled
 *
 * @param PortNo    The CDC port
 */
void hal_cdc_disable(uint8_t PortNo);


/**
 * Called when the CDC terminal has connected and is setting the Data Terminal
 * Ready flag
 *
 * @param PortNo    The CDC port
 * @param Enable    true to enable DTR; false to disable
 */
void hal_cdc_set_dtr(uint8_t PortNo, bool Enable);


/**
 * Function has the prototype suitable for stdio_serial.ptr_put and called by
 * printf et al to send data to stdout. This function actually sends data
 * @note This is re-linked to ptr_put to save the time executing the serial port
 *       open test
 */
int hal_cdc_putc(void volatile* pParam, char Value);


/**
 * Function has the prototype suitable for stdio_serial.ptr_put and called by
 * printf et al to send data to stdout. This function ignores the data and used
 * to silently consume printf output when the CDC port is not active.
 * @note This is re-linked to ptr_put to save the time executing the serial port
 *       open test
 */
int hal_cdc_putc_null(void volatile* pParam, char Value);

