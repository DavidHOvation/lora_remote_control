/**
 * Receiver (Discrete interface) Demo
 *
 * @author  David Hughes
 * @date    29Jun23
 */

// C Libraries
#include <stdint.h>     // uint8_t, uint32_t
#include <inttypes.h>   // PRI* printf macros
#include <stdbool.h>    // bool, true, false
#include <stdio.h>      // printf
#include <string.h>     // memcpy
#include <assert.h>


// Project
#include <hal-pcb1350a.h>
#include <nvm.h>
#include "sx1276.h"
#include "lora.h"
#include "vt100.h"
#include "util.h"
#include "frequency.h"
#include "message.h"
#include "ReceiverUI.h"
#include "rctl.h"


/*
 * Private types
 */


/**
 * Non-volatile storage
 */
typedef struct
{
    uint32_t ident; // Mandatory field

    // Radio and link parameters
    rctl_link_params_t link;

    // Message sequence
    uint8_t sequence_number;
}
_nv_params_t;


/*
 * Constants
 */
#define _PARAMS_IDENT_VALUE         (0x88550000)    // Can be any value except FFFFFFFF

static const _nv_params_t* const _p_params_nv = (_nv_params_t*) NVMCTRL_RWW_EEPROM_ADDR;

static const _nv_params_t _params_defaults =
{
    .ident = _PARAMS_IDENT_VALUE,


    .link = // Set to the key fob default operating parameters (No pairing necessary)
    {
        .sync_word          = 0x15,
        .sf                 = sx1276_sf_9,
        .bw                 = sx1276_bw_125kHz,
        .cr                 = sx1276_cr_4_5,
        .rf_pin             = sx1276_rf_pin_rfo,// RF Output
        .pout               = 14,               // dBm
        .frf                = 14221312,         // sx1276_frequency_to_frf(LORA_CH_17_868)
        .preamble_length    = 16,
        .p_key              = { 0xA1568A62, 0x57F2B141, 0xEFEB7C2C, 0x23C49C91 },
    },

#if 0
    .link = // Set to the pairing parameters
    {
        .sync_word          = 0x15,
        .sf                 = sx1276_sf_7,
        .bw                 = sx1276_bw_125kHz,
        .cr                 = sx1276_cr_4_8,
        .rf_pin             = sx1276_rf_pin_rfo,// RF Output
        .pout               = 0,                // dBm
        .frf                = 14221312,         // sx1276_frequency_to_frf(LORA_CH_17_868)
        .preamble_length    = 16,
        .p_key              = { 0xA1568A62, 0x57F2B141, 0xEFEB7C2C, 0x23C49C91 },
    },
#endif

    .sequence_number = 0,
};


/*
 * Private data
 */
static struct
{
    _nv_params_t params;

    volatile bool _button_callback_called;
    volatile bool _dio_callback_called;
}
_main;


/*
 * Private functions
 */


/**
 * Initialises the NVM module then checks the stored structure for corrupt
 * parameters and loads the defaults if so.
 *
 * @param pParams   Parameter set to apply
 * @return true if restored; false otherwise
 */
bool _nv_params_load(_nv_params_t* const pParams);


/**
 * Writes a params structure to NV
 *
 * @param pParams   Parameter set to apply
 * @return true if ok; false otherwise
 */
bool _nv_params_save(_nv_params_t* const pParams);


/**
 * Reads and encodes the output state
 *
 * @param pBuffer       Pointer to the destination buffer
 * @param BufferLength  Number of bytes available in pBuffer
 */
void _output_state_serialise(uint8_t* const pBuffer,
                             size_t const BufferLength);


/**
 * Function called on falling or rising edge of button
 */
void _button_callback(void);


/**
 * Called when any DIO line becomes active
 */
void _dio_callback(void);


/*
 * Main Entry Point!
 */
int main(void)
{
    bool ok;

    util_clear(&_main, sizeof(_main));

    /* System Initialization */
    hal_init();
    ReceiverUI_init();
    
    hal_interrupt_enable_global(true);


    /*
     * Hardware Initialisation
     */
    if (hal_usb_vbus_is_present())
    {
        hal_usb_stdio_enable();
        while (!hal_usb_stdio_is_running());
    }

    printf(VT100_ResetDevice
           VT100_SetAttributeMode1(VT100_ForegroundGreen)
           "LoRa Receiver - Discrete Interface (PCB1350A)" UTIL_EOL
           "Ovation Systems Ltd (c) 2023" UTIL_EOL
           VT100_ResetAttributeMode);

    // load _params
    ok = _nv_params_load(&_main.params);
    printf("Loading Params from memory: %s" UTIL_EOL,
           ok ? "OK" : "Defaults Used");

    message_init();
    sx1276_init();

    {
        uint8_t const version = sx1276_get_version();
        printf("SX1276 Version: %02" PRIX8 " (%s)" UTIL_EOL,
               version, (version == sx1276_version_expected) ? "OK" : "ERROR");
    }

    sx1276_set_clock_source_type(sx1276_clock_source_type_tcxo);

    printf("SX1276 FIFO Test: %s" UTIL_EOL,
            sx1276_test_fifo(false) ? "PASSED" : "FAILED");

    rctl_link_params_apply(&_main.params.link);

    sx1276_set_header_mode(sx1276_header_mode_explicit);
    sx1276_set_rx_payload_crc_enabled(true);
    sx1276_set_lna(sx1276_lna_gain_auto, true);


    int             pout = -1;
    sx1276_rf_pin_t pin  = -1;
    sx1276_get_tx_power(&pout, &pin);

    printf("Radio Params:"          UTIL_EOL
            "    Freq    : %lu Hz"  UTIL_EOL
            "    Pout    : %d dBm"  UTIL_EOL
            "    Tx Pin  : %s"      UTIL_EOL
            "    BW      : %s"      UTIL_EOL
            "    SF      : %s"      UTIL_EOL
            "    CR      : %s"      UTIL_EOL
            "    Preamble: %d"      UTIL_EOL
            "    Sync    : 0x%02X"  UTIL_EOL
            "    Header  : %s"      UTIL_EOL
            "    Auto-CRC: %s"      UTIL_EOL
            "    Symb TO : %d"      UTIL_EOL,
            sx1276_get_frequency(),
            pout,
            sx1276_rf_pin_str(pin),
            sx1276_bw_str(sx1276_get_bandwidth()),
            sx1276_sf_str(sx1276_get_spreading_factor()),
            sx1276_cr_str(sx1276_get_coding_rate()),
            sx1276_get_preamble_length(),
            sx1276_get_sync_word(),
            sx1276_get_header_mode() > 0 ? "Implicit" : "Explicit",
            sx1276_get_rx_payload_crc_enabled() ? "Yes" : "No",
            sx1276_get_symbol_timeout());


    sx1276_set_irq_enable(sx1276_irq_tx_done    |
                          sx1276_irq_rx_done    |
                          sx1276_irq_rx_timeout |
                          sx1276_irq_payload_crc_error);

    // dio0 is tx_done and rx_done;
    sx1276_set_dio_mapping(sx1276_dio_1_rx_timeout);
    sx1276_set_dio_mapping(sx1276_dio_3_payload_crc_error);

    hal_interrupt_set_callback(hal_interrupt_source_dio0, _dio_callback);
    hal_interrupt_set_callback(hal_interrupt_source_dio1, _dio_callback);
    hal_interrupt_set_callback(hal_interrupt_source_dio3, _dio_callback);

    hal_interrupt_enable_callback(hal_interrupt_source_dio0, true);
    hal_interrupt_enable_callback(hal_interrupt_source_dio1, true);
    hal_interrupt_enable_callback(hal_interrupt_source_dio3, true);

    hal_interrupt_set_callback(hal_interrupt_source_button, _button_callback);
    hal_interrupt_enable_callback(hal_interrupt_source_button, true);

    while (1)
    {
        bool button_is_pressed = false;

        uint32_t p_buffer_line[sizeof(rctl_message_t) / sizeof(uint32_t)];
        uint32_t p_buffer_plain[sizeof(rctl_message_t) / sizeof(uint32_t)];

        rctl_message_t* const p_message = (rctl_message_t*) p_buffer_plain;


        printf("Awaiting request..." UTIL_EOL);

        ReceiverUI_led_status_set(ReceiverUI_colour_green);

        // Receive command
        sx1276_set_dio_mapping(sx1276_dio_0_rx_done);
        _main._button_callback_called = false;
        _main._dio_callback_called = false;   // rx done, timeout, crc error
        sx1276_set_mode(sx1276_mode_rx_continuous);
        while (!_main._dio_callback_called)   // Wait for packet to be received
        {
            hal_wait_for_interrupt();

            if (_main._button_callback_called)
            {
                button_is_pressed = ReceiverUI_button_is_pressed();

                rctl_link_params_t* p_link;

                if (button_is_pressed)
                {
                    printf("Button pressed. Setting pair mode params..." UTIL_EOL);
                    // Button pressed: Pair mode using default params
                    p_link = (rctl_link_params_t*) &_params_defaults.link;
                }
                else
                {
                    printf("Button released. Resetting original params..." UTIL_EOL);
                    // Button released; restore params
                    p_link = &_main.params.link;
                }

                sx1276_set_mode(sx1276_mode_stdby);
                rctl_link_params_apply(p_link);
                _main._button_callback_called = false;
                _main._dio_callback_called = false;   // rx done, timeout, crc error
                sx1276_set_mode(sx1276_mode_rx_continuous);

            }
            // The USB stack fires off lots of interrupts...
        }

        sx1276_set_mode(sx1276_mode_stdby);

        ReceiverUI_led_status_set(ReceiverUI_colour_black);


        do
        {
            sx1276_rssi_t rssi = sx1276_get_packet_rssi();
            sx1276_snr_t  snr  = sx1276_get_packet_snr();

            sx1276_irq_t irq = sx1276_get_clear_irq_flags();

            if (irq & sx1276_irq_rx_timeout)
            {
                printf("ERROR: Receive timeout" UTIL_EOL);
                break;
            }

            if (irq & sx1276_irq_payload_crc_error)
            {
                printf("ERROR: Payload CRC error" UTIL_EOL);
                break;
            }

             if (!(irq & sx1276_irq_rx_done))
             {
                 printf("ERROR: Expected Received Message Indicator" UTIL_EOL);
                 break;
             }

            size_t packet_len = sx1276_read_packet(p_buffer_line,
                                                   sizeof(p_buffer_line));

            // Check out the packet

            // Must be at least one block long and a multiple of the block size
            if ((packet_len < RCTL_MIN_PACKET_LEN) ||
                ((packet_len % MESSAGE_BLOCK_LENGTH_BYTES) != 0))
            {
                printf("ERROR: Invalid packet length (%u)" UTIL_EOL,
                       packet_len);
                break;
            }


            // Packet starts with the IV in plain
            // Copy the IV for maybe reporting?
            memcpy(p_message->p_iv, p_buffer_line, MESSAGE_BLOCK_LENGTH_BYTES);
            message_decrypt(p_message->p_iv,
                            &p_buffer_line[MESSAGE_BLOCK_LENGTH_U32],
                            &p_buffer_plain[MESSAGE_BLOCK_LENGTH_U32],
                            packet_len - MESSAGE_BLOCK_LENGTH_BYTES);

#ifdef PRINT_PACKETS
            printf("Line data:");
            util_print_uint8_hex(p_buffer_line, packet_len);
            printf(UTIL_EOL);

            printf("Plain data:");
            util_print_uint8_hex(p_buffer_plain, packet_len);
            printf(UTIL_EOL);
#endif

            if (p_message->id != rctl_id_controller_to_receiver)
            {
                printf("ERROR: Invalid identifier ("
                       "Received %" PRIX16 " "
                       "Expected %" PRIX16
                       ")" UTIL_EOL,
                       p_message->id,
                       rctl_id_controller_to_receiver);
                break;
            }

            // Simple Replay Attack
            if (p_message->seq == _main.params.sequence_number)
            {
                printf("ERROR: Sequence number (%" PRIu8 ") same as before"
                       UTIL_EOL,
                       p_message->seq);
                break;
            }

            if (p_message->cid != (~p_message->icid & 0xFF))
            {
                printf("ERROR: ICID isn't as expected ("
                       "CID: %02X" ", "
                       "ICID: %02X"
                       ")" UTIL_EOL,
                       p_message->cid,
                       p_message->icid);
                break;
            }

            size_t expected_length;
            switch (p_message->cid)
            {
            default:
                printf("ERROR: Unexpected CID %d (%s)" UTIL_EOL,
                       p_message->cid,
                       rctl_cid_str(p_message->cid));

                expected_length = 0;    // Cause next test to fail
                break;

            case rctl_cid_no_operation:
            case rctl_cid_set_power_on:
            case rctl_cid_set_power_off:
            case rctl_cid_set_record_on:
            case rctl_cid_set_record_off:
            case rctl_cid_set_stream_on:
            case rctl_cid_set_stream_off:
            case rctl_cid_get_output_state:
            case rctl_cid_get_params:
            case rctl_cid_apply_params:
                expected_length = MESSAGE_BLOCK_LENGTH_BYTES;
                break;

            case rctl_cid_set_params:
            case rctl_cid_pair:
                expected_length = MESSAGE_BLOCK_LENGTH_BYTES * 2;
                break;
            }

            if (expected_length == 0)
            {
                break;  // Save repeating potentially confusing error message
            }
            
            expected_length += MESSAGE_BLOCK_LENGTH_BYTES;  // for IV block

            if (packet_len != expected_length)
            {
                printf("ERROR: Reply not of the required length ("
                       "Received %u" " "
                       "Expected %u"
                       ")" UTIL_EOL,
                       packet_len,
                       expected_length);
                break;
            }

            // Don't necessarily save it to NV...
            _main.params.sequence_number = p_message->seq;

            size_t reply_message_length;

            // Do action
            printf("Processing request #%" PRIu8 " %s..." UTIL_EOL
                   "    RSSI: %d"       UTIL_EOL
                   "    SNR : %s"       UTIL_EOL,
                   p_message->seq,
                   rctl_cid_str(p_message->cid),
                   rssi,
                   sx1276_snr_str(snr));

            switch (p_message->cid)
            {
            default:
                reply_message_length = 0;
                break;

            case rctl_cid_no_operation:
                p_message->cid = rctl_cid_acknowledge;
                reply_message_length = MESSAGE_BLOCK_LENGTH_BYTES;
                break;

            case rctl_cid_set_power_on:
                hal_output_set(hal_output_power, true);

                p_message->cid = rctl_cid_acknowledge;
                reply_message_length = MESSAGE_BLOCK_LENGTH_BYTES;
                break;

            case rctl_cid_set_power_off:
                hal_output_set(hal_output_discrete_1, false);
                hal_output_set(hal_output_discrete_2, false);
                hal_output_set(hal_output_power, false);

                p_message->cid = rctl_cid_acknowledge;
                reply_message_length = MESSAGE_BLOCK_LENGTH_BYTES;
                break;

            case rctl_cid_set_record_on:
                hal_output_set(hal_output_power, true);
                hal_output_set(hal_output_discrete_1, true);

                p_message->cid = rctl_cid_acknowledge;
                reply_message_length = MESSAGE_BLOCK_LENGTH_BYTES;
                break;

            case rctl_cid_set_record_off:
                hal_output_set(hal_output_discrete_1, false);

                p_message->cid = rctl_cid_acknowledge;
                reply_message_length = MESSAGE_BLOCK_LENGTH_BYTES;
                break;

            case rctl_cid_set_stream_on:
                hal_output_set(hal_output_power, true);
                hal_output_set(hal_output_discrete_2, true);

                p_message->cid = rctl_cid_acknowledge;
                reply_message_length = MESSAGE_BLOCK_LENGTH_BYTES;
                break;

            case rctl_cid_set_stream_off:
                hal_output_set(hal_output_discrete_2, false);

                p_message->cid = rctl_cid_acknowledge;
                reply_message_length = MESSAGE_BLOCK_LENGTH_BYTES;
                break;

            case rctl_cid_get_output_state:
                p_message->cid = rctl_cid_get_output_state_reply;
                _output_state_serialise(p_message->p_payload,
                                        RCTL_SHORT_PARAMS_LENGTH);

                reply_message_length = MESSAGE_BLOCK_LENGTH_BYTES;
                break;

            case rctl_cid_get_params:
                rctl_link_params_serialise(&_main.params.link,
                                           p_message->p_payload,
                                           sizeof(p_message->p_payload));

                p_message->cid = rctl_cid_get_params_reply;
                reply_message_length = MESSAGE_BLOCK_LENGTH_BYTES * 2;
                break;

            case rctl_cid_set_params:
                rctl_link_params_deserialise(&_main.params.link,
                                             p_message->p_payload,
                                             sizeof(p_message->p_payload));

                p_message->cid = rctl_cid_acknowledge;
                reply_message_length = MESSAGE_BLOCK_LENGTH_BYTES;
                break;

            case rctl_cid_apply_params:
                printf("Applying params..." UTIL_EOL);
                rctl_link_params_apply(&_main.params.link);
                (void) _nv_params_save(&_main.params);

                reply_message_length = 0;   // No response
                break;

            case rctl_cid_pair:
                if (button_is_pressed)
                {
                    printf("Button is pressed. Applying params..." UTIL_EOL);
                    rctl_link_params_deserialise(&_main.params.link,
                                                 p_message->p_payload,
                                                 sizeof(p_message->p_payload));
                    rctl_link_params_apply(&_main.params.link);
                    ok = _nv_params_save(&_main.params);

                    p_message->cid = ok ?
                        rctl_cid_acknowledge : rctl_cid_not_acknowledge;
                    reply_message_length = MESSAGE_BLOCK_LENGTH_BYTES;
                }
                else
                {
                    printf("Button is not pressed. Ignoring..." UTIL_EOL);
                    reply_message_length = 0;   // No response
                }
                break;
            }


            if (reply_message_length == 0)
            {
                // Send no reply
                printf("No reply to send" UTIL_EOL);
                break;
            }
            
            
            if ((p_message->cid == rctl_cid_acknowledge)        ||
                (p_message->cid == rctl_cid_not_acknowledge))
            {
                memset(&p_message->p_payload[0], 0, RCTL_SHORT_PARAMS_LENGTH);
            }

            reply_message_length += MESSAGE_BLOCK_LENGTH_BYTES;   // Space for IV

            // Send message
            message_get_random_block(p_message->p_iv);
            p_message->id = rctl_id_receiver_to_controller;
            p_message->icid = ~p_message->cid;
            // sequence_number  same as received message
            // command_id       already set
            // params           already set

            // Message starts with the IV in plain
            memcpy(p_buffer_line, p_message->p_iv, MESSAGE_BLOCK_LENGTH_BYTES);
            message_encrypt(p_message->p_iv,
                            &p_buffer_plain[MESSAGE_BLOCK_LENGTH_U32],
                            &p_buffer_line[MESSAGE_BLOCK_LENGTH_U32],
                            reply_message_length - MESSAGE_BLOCK_LENGTH_BYTES);

            printf("Sending reply: %s" UTIL_EOL,
                   rctl_cid_str(p_message->cid));


#ifdef PRINT_PACKETS
            printf("Plain data:");
            util_print_uint8_hex(p_buffer_plain, reply_message_length);
            printf(UTIL_EOL);

            printf("Line data:");
            util_print_uint8_hex(p_buffer_line, reply_message_length);
            printf(UTIL_EOL);
#endif

            ReceiverUI_led_status_set(ReceiverUI_colour_red);

            sx1276_set_dio_mapping(sx1276_dio_0_tx_done);
            _main._dio_callback_called = false;
            sx1276_send_packet(p_buffer_line, reply_message_length);
            while (!_main._dio_callback_called)
            {
                hal_wait_for_interrupt();
                // The USB stack fires off lots of interrupts...
            }

            ReceiverUI_led_status_set(ReceiverUI_colour_black);
        }
        while (0);
    }
}


/*
 * Private functions
 */
bool _nv_params_load(_nv_params_t* const pParams)
{
    assert(pParams != NULL);

    // No need for fancy reading functions: just do a table read
    bool const ok = (_p_params_nv->ident == _PARAMS_IDENT_VALUE);

    const void* const p_src = ok ? _p_params_nv : &_params_defaults;

    memcpy(pParams, p_src, sizeof(*pParams));

    return ok;
}


bool _nv_params_save(_nv_params_t* const pParams)
{
    assert(pParams != NULL);

    bool ok = false;

    uint32_t const nv_addr = (uint32_t)_p_params_nv;    // Location in memory

    struct nvm_config cfg;
    nvm_get_config_defaults(&cfg);
    cfg.manual_page_write = false;
    nvm_set_config(&cfg);

    do
    {
        enum status_code stat;

        stat = nvm_erase_row(nv_addr);
        if (stat != STATUS_OK)
        {
            break;
        }

        stat = nvm_write_buffer(nv_addr,
                                (uint8_t*) pParams,
                                (uint16_t) sizeof(*pParams));
        if (stat != STATUS_OK)
        {
            break;
        }

        ok = true;
    }
    while (0);

    return ok;
}


void _output_state_serialise(uint8_t* const pBuffer,
                             size_t const BufferLength)
{
    assert(pBuffer != NULL);
    assert(BufferLength >= 4);

    bool const power_output_state   = hal_output_get(hal_output_power);
    bool const discrete_1_state     = hal_output_get(hal_output_discrete_1);
    bool const discrete_2_state     = hal_output_get(hal_output_discrete_2);

    bool const power_output_active  = power_output_state;   // Currently no hardware to detect between the cases

    memset(pBuffer, 0, BufferLength);

    pBuffer[rctl_output_flags_power_output_state]   = power_output_state;
    pBuffer[rctl_output_flags_input_1_state]        = discrete_1_state;
    pBuffer[rctl_output_flags_input_2_state]        = discrete_2_state;
    pBuffer[rctl_output_flags_power_output_active]  = power_output_active;
}


void _button_callback(void)
{
    _main._button_callback_called = true;
}


void _dio_callback(void)
{
    _main._dio_callback_called = true;
}
