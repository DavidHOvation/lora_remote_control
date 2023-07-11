/**
 * \file
 *
 * \brief USB configuration file for  USB CDC application
 *
 * Copyright (c) 2019 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#pragma once

#include "compiler.h"
#include "usb_protocol_cdc.h"


//#warning You must refill the following definitions with a correct values

/**
 * USB Device Configuration
 * @{
 */

//! Device definition (mandatory)
#define USB_DEVICE_VENDOR_ID            USB_VID_ATMEL
#define USB_DEVICE_PRODUCT_ID           USB_PID_ATMEL_ASF_CDC
#define USB_DEVICE_MAJOR_VERSION        1
#define USB_DEVICE_MINOR_VERSION        0
#define USB_DEVICE_POWER                100 // Consumption on Vbus line (mA)
#define USB_DEVICE_ATTR                 \
    (USB_CONFIG_ATTR_SELF_POWERED)
// (USB_CONFIG_ATTR_BUS_POWERED)
// (USB_CONFIG_ATTR_REMOTE_WAKEUP|USB_CONFIG_ATTR_SELF_POWERED)
// (USB_CONFIG_ATTR_REMOTE_WAKEUP|USB_CONFIG_ATTR_BUS_POWERED)

//! USB Device string definitions (Optional)
#define USB_DEVICE_MANUFACTURE_NAME     "Ovation Ltd"
#define USB_DEVICE_PRODUCT_NAME         "LoRa stdio"
//#define USB_DEVICE_MANUFACTURE_NAME     "Manufacture name"
//#define USB_DEVICE_PRODUCT_NAME         "Product name"
//#define USB_DEVICE_SERIAL_NAME          "12...EF"


/**
 * Device speeds support
 * Low speed not supported by CDC
 * @{
 */
//! To authorize the High speed
#if (UC3A3 || UC3A4)
#define USB_DEVICE_HS_SUPPORT
#endif
//@}


#include "hal_cdc.h"

/**
 * USB Device Callbacks definitions (Optional)
 * @{
 */
#define UDC_VBUS_EVENT(b_vbus_high)         hal_cdc_vbus_event(b_vbus_high)

//#define UDC_SOF_EVENT()                   callback_sof_action()
//#define UDC_SUSPEND_EVENT()               callback_suspend_action()
//#define UDC_RESUME_EVENT()                callback_resume_action()

// Mandatory when USB_DEVICE_ATTR authorizes remote wakeup feature
//#define UDC_REMOTEWAKEUP_ENABLE()         user_callback_remotewakeup_enable()
//#define UDC_REMOTEWAKEUP_DISABLE()        user_callback_remotewakeup_disable()

//#ifdef USB_DEVICE_LPM_SUPPORT
//#define UDC_SUSPEND_LPM_EVENT()           suspend_lpm_action()
//#define UDC_REMOTEWAKEUP_LPM_ENABLE()     remotewakeup_lpm_enable()
//#define UDC_REMOTEWAKEUP_LPM_DISABLE()    remotewakeup_lpm_disable()
//#endif

// When a extra string descriptor must be supported
// other than manufacturer, product and serial string
//#define UDC_GET_EXTRA_STRING()
//@}

//@}


/**
 * USB Interface Configuration
 * @{
 */
/**
 * Configuration of CDC interface
 * @{
 */

//! Number of communication port used (1 to 3)
#define UDI_CDC_PORT_NB 1

//! Interface callback definition
#define UDI_CDC_ENABLE_EXT(port)            hal_cdc_enable(port)

#define UDI_CDC_DISABLE_EXT(port)           hal_cdc_disable(port)

#define UDI_CDC_RX_NOTIFY(port)
#define UDI_CDC_TX_EMPTY_NOTIFY(port)
#define UDI_CDC_SET_CODING_EXT(port, cfg)
#define UDI_CDC_SET_DTR_EXT(port, set)      hal_cdc_set_dtr(port, set)

#define UDI_CDC_SET_RTS_EXT(port, set)

//#define UDI_CDC_ENABLE_EXT(port)            callback_cdc_enable()
//#define UDI_CDC_DISABLE_EXT(port)           callback_cdc_disable()
//#define UDI_CDC_RX_NOTIFY(port)             callback_rx_notify(port)
//#define UDI_CDC_TX_EMPTY_NOTIFY(port)       callback_tx_empty_notify(port)
//#define UDI_CDC_SET_CODING_EXT(port,cfg)    callback_config(port, cfg)
//#define UDI_CDC_SET_DTR_EXT(port,set)       callback_cdc_set_dtr(port, set)
//#define UDI_CDC_SET_RTS_EXT(port,set)       callback_cdc_set_rts(port, set)

//! Define it when the transfer CDC Device to Host is a low rate (<512000 bauds)
//! to reduce CDC buffers size
#define UDI_CDC_LOW_RATE

//! Default configuration of communication port
#define UDI_CDC_DEFAULT_RATE                115200
#define UDI_CDC_DEFAULT_STOPBITS            CDC_STOP_BITS_1
#define UDI_CDC_DEFAULT_PARITY              CDC_PAR_NONE
#define UDI_CDC_DEFAULT_DATABITS            8
//@}
//@}


/**
 * USB Device Driver Configuration
 * @{
 */
//@}

//! The includes of classes and other headers must be done at the end of this
//! file to avoid compile error
#include "udi_cdc_conf.h"
