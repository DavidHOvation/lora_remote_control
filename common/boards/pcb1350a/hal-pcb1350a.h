/**
 * Ovation HAL interface including PCB 1350A definitions
 *
 * @author  David Hughes
 * @date    30Jun23
 */
#pragma once

#include "pcb1350a.h"   // Pin definitions: Also used by ASF


#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>     // size_t


/*
 * Public types
 */
typedef enum
{
    hal_button_switch1, // Record
    hal_button_switch2, // Stream / WiFi
    hal_button_switch3, // On
    hal_button_switch4, // Off
    hal_button_button,  // Config Switch
}
hal_button_t;

typedef enum
{
    hal_output_power,
    hal_output_discrete_1,
    hal_output_discrete_2,
}
hal_output_t;

typedef enum
{
    hal_interrupt_source_switch1,
    hal_interrupt_source_switch2,
    hal_interrupt_source_switch3,
    hal_interrupt_source_switch4,
    hal_interrupt_source_button,
    hal_interrupt_source_vbus,
    hal_interrupt_source_usb_id,
    hal_interrupt_source_dio0,
    hal_interrupt_source_dio1,
    hal_interrupt_source_dio2,
    hal_interrupt_source_dio3,
    hal_interrupt_source_dio4,
    hal_interrupt_source_dio5,
}
hal_interrupt_source_t;

typedef void (hal_interrupt_callback_t)(void);


/*
 * Public functions
 */


/**
 * Module initialiser
 */
void hal_init(void);


void hal_shutdown(void);


/*
 * User Interface
 */
bool hal_button_is_pressed(hal_button_t const Identifier);

void hal_led_rgb_set(bool const RedOn, bool const GreenOn, bool const BlueOn);

void hal_led_stat_set(bool const RedOn, bool const GreenOn);


/*
 * Control output
 */
void hal_output_set(hal_output_t const Identifier, bool const Active);
bool hal_output_get(hal_output_t const Identifier);


/*
 * I2C peripheral access
 * @note Must conform to i2c_interface_t
 */
bool hal_i2c_rw(unsigned const DeviceAddress,
                void* pWData, size_t const WDataSize,
                void* pRData, size_t const RDataSize);


/*
 * Radio Interface
 */


/**
 * Asserts the CS line at the start of an SPI transaction
 */
void hal_rf_spi_start(void);


/**
 * De-asserts the CS line at the end of an SPI transaction
 */
void hal_rf_spi_end(void);


/**
 * Performs one SPI write-read operation
 *
 * @param MOSI  The value to present on the MOSI line
 * @return The value returned on the MISO line
 */
uint8_t hal_rf_spi_transfer(uint8_t const MOSI);


void hal_rf_set_tcxo(bool const Active);


void hal_rf_set_switch(bool const Active);


void hal_rf_set_reset(bool const Active);


/*
 * Blocking delays
 */
void hal_delay_ms(uint32_t const Time);


void hal_delay_us(uint32_t const Time);


void hal_periodic_set_period(uint32_t const PeriodMSec);

void hal_interrupt_set_callback(hal_interrupt_source_t const Source,
                                hal_interrupt_callback_t Callback);

void hal_interrupt_enable_callback(hal_interrupt_source_t const Source,
                                   bool const Enabled);


/**
 * Controls global interrupts, allowing for nested handlers
 */
void hal_interrupt_enable_global(bool const Enabled);


void hal_wait_for_interrupt(void);


/*
 * USB
 */
bool hal_usb_vbus_is_present(void);


void hal_usb_stdio_enable(void);


void hal_usb_stdio_disable(void);


bool hal_usb_stdio_is_running(void);


/*
 * 1msec Periodic timer
 */
void hal_periodic_set_callback(hal_interrupt_callback_t Callback);


void hal_periodic_enable(bool const Enabled);
