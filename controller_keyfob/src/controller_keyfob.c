/**
 * LoRa Key fob Controller
 *
 * @author  David Hughes
 * @date    30Jun23
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
#include "keyfobUI.h"
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

static const _nv_params_t _params_default =
{
    .ident = _PARAMS_IDENT_VALUE,

    // Desired operating parameters
    .link =
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

    .sequence_number = 0,
};


/*
 * LED Sequences
 */
static const keyfobUI_led_sequence_t _seq_response_timeout =
{
    .repeat_count   = 10,
    .length         = 2,
    .p_seq =
    {
        { keyfobUI_colour_red,      10 },
        { keyfobUI_colour_black,    10 },
    },        
};

static const keyfobUI_led_sequence_t _seq_response_power_on =
{
    .repeat_count   = 2,
    .length         = 2,
    .p_seq =
    {
        { keyfobUI_colour_red,      50 },
        { keyfobUI_colour_black,    50 },
    },
};

static const keyfobUI_led_sequence_t _seq_response_power_off =
{
    .repeat_count   = 1,
    .length         = 1,
    .p_seq =
    {
        { keyfobUI_colour_red,      200 },
    },    
};

static const keyfobUI_led_sequence_t _seq_response_recording_on =
{
    .repeat_count   = 2,
    .length         = 2,
    .p_seq =
    {
        { keyfobUI_colour_green,    50 },
        { keyfobUI_colour_black,    50 },
    },
};

static const keyfobUI_led_sequence_t _seq_response_recording_off =
{
    .repeat_count   = 2,
    .length         = 2,
    .p_seq =
    {
        { keyfobUI_colour_green,    50 },
        { keyfobUI_colour_red,      50 },
    },        
};

static const keyfobUI_led_sequence_t _seq_response_streaming_on =
{
    .repeat_count   = 2,
    .length         = 2,
    .p_seq =
    {
        { keyfobUI_colour_blue,     50 },
        { keyfobUI_colour_black,    50 },
    },        
};

static const keyfobUI_led_sequence_t _seq_response_streaming_off =
{
    .repeat_count = 2,
    .length = 2,
    .p_seq =
    {
        { keyfobUI_colour_blue,     50 },
        { keyfobUI_colour_red,      50 },
    },    
};

static const keyfobUI_led_sequence_t _seq_response_recording_streaming =
{
    .repeat_count   = 2,
    .length         = 2,
    .p_seq =
    {
        { keyfobUI_colour_blue,     50 },
        { keyfobUI_colour_green,    50 },
    },
};


/*
 * Private data
 */
static struct
{
    _nv_params_t params;

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
 * Called when any DIO line becomes active
 */
void _dio_callback(void);

#include <system.h>

/*
 * Main Entry Point!
 */
int main(void)
{
    bool ok;

    util_clear(&_main, sizeof(_main));

    /* System Initialization */
    hal_init();

    keyfobUI_init();


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
           "LoRa Key fob Controller (PCB1350A)" UTIL_EOL
           "Ovation Systems Ltd (c) 2023" UTIL_EOL
           VT100_ResetAttributeMode);

    printf("GCLK_GENERATOR_0 = %" PRIu32 "Hz" UTIL_EOL, system_gclk_gen_get_hz(GCLK_GENERATOR_0));
    printf("GCLK_GENERATOR_1 = %" PRIu32 "Hz" UTIL_EOL, system_gclk_gen_get_hz(GCLK_GENERATOR_1));
    printf("GCLK_GENERATOR_2 = %" PRIu32 "Hz" UTIL_EOL, system_gclk_gen_get_hz(GCLK_GENERATOR_2));
    printf("GCLK_GENERATOR_3 = %" PRIu32 "Hz" UTIL_EOL, system_gclk_gen_get_hz(GCLK_GENERATOR_3));
    printf("GCLK_GENERATOR_4 = %" PRIu32 "Hz" UTIL_EOL, system_gclk_gen_get_hz(GCLK_GENERATOR_4));
    printf("GCLK_GENERATOR_5 = %" PRIu32 "Hz" UTIL_EOL, system_gclk_gen_get_hz(GCLK_GENERATOR_5));

restart:

    // load _params
    ok = _nv_params_load(&_main.params);
    printf("Loaded parameters from memory: %s" UTIL_EOL,
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

    rctl_link_params_t link_params = { 0 };

    if (hal_usb_vbus_is_present())
    {
        printf("Awaiting button press..." UTIL_EOL);
        while (!keyfobUI_button_is_busy())
        {
            hal_wait_for_interrupt();
        }
    }


    keyfobUI_button_t button;
    while (1)
    {
        hal_wait_for_interrupt();
        button = keyfobUI_button_get();
        if (button != keyfobUI_button_none)
        {
            break;
        }
    }
    

    do
    {
        uint32_t p_buffer_line[sizeof(rctl_message_t) / sizeof(uint32_t)];
        uint32_t p_buffer_plain[sizeof(rctl_message_t) / sizeof(uint32_t)] = { 0 };

        rctl_message_t* const p_message = (rctl_message_t*) p_buffer_plain;

        uint8_t request_id;
        size_t request_length;
        size_t response_length;

        switch (button)
        {
        default: // keyfobUI_button_none
            // Currently no way to send no operation or set|set|apply params commands
            request_id      = 0;
            request_length  = 0;
            response_length = 0;
            break;

        case keyfobUI_button_on:
            request_id      = rctl_cid_set_power_on;
            request_length  = MESSAGE_BLOCK_LENGTH_BYTES;
            response_length = MESSAGE_BLOCK_LENGTH_BYTES;
            break;

        case keyfobUI_button_off:
            request_id      = rctl_cid_set_power_off;
            request_length  = MESSAGE_BLOCK_LENGTH_BYTES;
            response_length = MESSAGE_BLOCK_LENGTH_BYTES;
            break;

        case keyfobUI_button_status:
            request_id      = rctl_cid_get_output_state;
            request_length  = MESSAGE_BLOCK_LENGTH_BYTES;
            response_length = MESSAGE_BLOCK_LENGTH_BYTES;
            break;

        case keyfobUI_button_record:
            request_id      = rctl_cid_set_record_on;
            request_length  = MESSAGE_BLOCK_LENGTH_BYTES;
            response_length = MESSAGE_BLOCK_LENGTH_BYTES;
            break;

        case keyfobUI_button_stream:
            request_id      = rctl_cid_set_stream_on;
            request_length  = MESSAGE_BLOCK_LENGTH_BYTES;
            response_length = MESSAGE_BLOCK_LENGTH_BYTES;
            break;

        case keyfobUI_button_off_record:
            request_id      = rctl_cid_set_record_off;
            request_length  = MESSAGE_BLOCK_LENGTH_BYTES;
            response_length = MESSAGE_BLOCK_LENGTH_BYTES;
            break;

        case keyfobUI_button_off_stream:
            request_id      = rctl_cid_set_stream_off;
            request_length  = MESSAGE_BLOCK_LENGTH_BYTES;
            response_length = MESSAGE_BLOCK_LENGTH_BYTES;
            break;

        case keyfobUI_button_record_stream: // pairing!
            printf("Pairing: Setting params to pairing" UTIL_EOL);
            rctl_link_params_apply((rctl_link_params_t*)&rctl_params_pairing);

            // Currently the only thing the pairing does is assigns a new IV and key
            memcpy(&link_params, &_main.params.link, sizeof(link_params));
            message_get_random_block(link_params.p_key);

            request_id      = rctl_cid_pair;
            rctl_link_params_serialise(&link_params,
                                   p_message->p_payload,
                                   sizeof(p_message->p_payload));
            request_length  = MESSAGE_BLOCK_LENGTH_BYTES * 2;
            response_length = MESSAGE_BLOCK_LENGTH_BYTES;
            break;
        }


        if (request_length == 0)
        {
            printf("No request to send" UTIL_EOL);
            break;
        }
        
        request_length += MESSAGE_BLOCK_LENGTH_BYTES;   // Space for IV


        // Send message
        _main.params.sequence_number++;
        message_get_random_block(p_message->p_iv);
        p_message->id   = rctl_id_controller_to_receiver;
        p_message->seq  = _main.params.sequence_number;
        p_message->cid  = request_id;
        p_message->icid = ~request_id;

        // Message starts with the IV in plain
        memcpy(p_buffer_line, p_message->p_iv, MESSAGE_BLOCK_LENGTH_BYTES);
        message_encrypt(p_message->p_iv,
                        &p_buffer_plain[MESSAGE_BLOCK_LENGTH_U32],
                        &p_buffer_line[MESSAGE_BLOCK_LENGTH_U32],
                        request_length - MESSAGE_BLOCK_LENGTH_BYTES);

        printf("Sending request #%" PRIu8" %s..." UTIL_EOL,
               p_message->seq,
               rctl_cid_str(p_message->cid));

#       ifdef PRINT_PACKETS
            printf("Plain data:");
            util_print_uint8_hex(p_buffer_plain, request_length);
            printf(UTIL_EOL);

            printf("Line data:");
            util_print_uint8_hex(p_buffer_line, request_length);
            printf(UTIL_EOL);
#       endif

        keyfobUI_led_status_set(keyfobUI_colour_red);

        sx1276_set_dio_mapping(sx1276_dio_0_tx_done);
        _main._dio_callback_called = false;  // tx done
        sx1276_send_packet(p_buffer_line, request_length);
        while (!_main._dio_callback_called)
        {
            hal_wait_for_interrupt();
            // The USB stack fires off lots of interrupts...
        }

        keyfobUI_led_status_set(keyfobUI_colour_black);

        if (response_length == 0)
        {
            printf("No response required" UTIL_EOL);
            break;
        }

        response_length += MESSAGE_BLOCK_LENGTH_BYTES;   // Space for IV

        printf("Awaiting response..." UTIL_EOL);

        keyfobUI_led_status_set(keyfobUI_colour_green);

        // Receive response
        sx1276_set_dio_mapping(sx1276_dio_0_rx_done);
        _main._dio_callback_called = false;  // rx done, timeout, crc error
        sx1276_set_mode(sx1276_mode_rx_single);
        while (!_main._dio_callback_called)
        {
            hal_wait_for_interrupt();
            // The USB stack fires off lots of interrupts...
        }

        keyfobUI_led_status_set(keyfobUI_colour_black);

        sx1276_rssi_t rssi = sx1276_get_packet_rssi();
        sx1276_snr_t  snr  = sx1276_get_packet_snr();

        sx1276_irq_t irq = sx1276_get_clear_irq_flags();

        if (irq & sx1276_irq_rx_timeout)
        {
            printf("ERROR: Reply timeout" UTIL_EOL);

            keyfobUI_led_sequence_start(&_seq_response_timeout);
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

#       ifdef PRINT_PACKETS
            printf("Line Data:");
            util_print_uint8_hex(p_buffer_line, packet_len);
            printf(UTIL_EOL);

            printf("Plain Data:");
            util_print_uint8_hex(p_buffer_plain, packet_len);
            printf(UTIL_EOL);
#       endif

        if (p_message->id != rctl_id_receiver_to_controller)
        {
            printf("ERROR: Invalid identifier ("
                   "Received: %" PRIX16 ", "
                   "Expected: %" PRIX16
                   ")" UTIL_EOL,
                   p_message->id,
                   rctl_id_receiver_to_controller);
            break;
        }

        if (p_message->seq != _main.params.sequence_number)
        {
            printf("ERROR: Reply has invalid sequence number ("
                   "Received: %" PRIu8 ", "
                   "Expected: %" PRIu8
                   ")" UTIL_EOL,
                   p_message->seq,
                   _main.params.sequence_number);
            break;
        }

        if (packet_len != response_length)
        {
            printf("ERROR: Reply not of the required length ("
                   "Received: %u" ", "
                   "Expected: %u"
                   ")" UTIL_EOL,
                   packet_len,
                   response_length);
            break;
        }

        if (p_message->cid != ((~p_message->icid) & 0xFF))
        {
            printf("ERROR: ICID isn't as expected ("
                   "CID: %02X" ", "
                   "ICID: %02X"
                   ")" UTIL_EOL,
                   p_message->cid,
                   p_message->icid);
            break;
        }

        printf("Received reply: %s" UTIL_EOL
               "    RSSI: %d"       UTIL_EOL
               "    SNR : %s"       UTIL_EOL,
               rctl_cid_str(p_message->cid),
               rssi,
               sx1276_snr_str(snr));

        // Show result
        switch (p_message->cid)
        {
        default:
            printf("ERROR: Reply invalid" UTIL_EOL);
            break;

        case rctl_cid_acknowledge:
            switch (request_id)
            {
            case rctl_cid_set_power_on:
                keyfobUI_led_sequence_start(&_seq_response_power_on);
                break;

            case rctl_cid_set_power_off:
                keyfobUI_led_sequence_start(&_seq_response_power_off);
                break;

            case rctl_cid_set_record_on:
                keyfobUI_led_sequence_start(&_seq_response_recording_on);
                break;

            case rctl_cid_set_record_off:
                keyfobUI_led_sequence_start(&_seq_response_recording_off);
                break;

            case rctl_cid_set_stream_on:
                keyfobUI_led_sequence_start(&_seq_response_streaming_on);
                break;

            case rctl_cid_set_stream_off:
                keyfobUI_led_sequence_start(&_seq_response_streaming_off);
                break;

            case rctl_cid_pair:
                memcpy(&_main.params.link, &link_params, sizeof(_main.params.link));
                rctl_link_params_apply(&_main.params.link);
                _nv_params_save(&_main.params);
                break;
            }

        case rctl_cid_not_acknowledge:
            break;

        case rctl_cid_get_output_state_reply:
            {
                bool const power_output_state  = p_message->p_payload[rctl_output_flags_power_output_state];
                bool const input_1_state       = p_message->p_payload[rctl_output_flags_input_1_state];
                bool const input_2_state       = p_message->p_payload[rctl_output_flags_input_2_state];
                bool const power_output_active = p_message->p_payload[rctl_output_flags_power_output_active];

                if (power_output_active)
                {
                    if (input_1_state && input_2_state)
                    {
                        keyfobUI_led_sequence_start(&_seq_response_recording_streaming);
                    }
                    else if (input_1_state)
                    {
                        keyfobUI_led_sequence_start(&_seq_response_recording_on);
                    }
                    else if (input_2_state)
                    {
                        keyfobUI_led_sequence_start(&_seq_response_streaming_on);
                    }
                    else
                    {
                        keyfobUI_led_sequence_start(&_seq_response_power_on);
                    }
                }
                else
                {
                    keyfobUI_led_sequence_start(&_seq_response_power_off);
                }

                printf("    power_output_state  : %s" UTIL_EOL
                       "    discrete_1_state    : %s" UTIL_EOL
                       "    discrete_2_state    : %s" UTIL_EOL
                       "    power_output_active : %s" UTIL_EOL,
                       power_output_state   ? "Powered" : "Un-powered",
                       input_1_state        ? "Active"  : "Inactive",
                       input_2_state        ? "Active"  : "Inactive",
                       power_output_active  ? "Yes"     : "No");
            }
            break;

        case rctl_cid_get_params_reply:
            {
                rctl_link_params_t link;
                rctl_link_params_deserialise(&link,
                                             p_message->p_payload,
                                             sizeof(link));
                printf("    Freq    : %lu Hz" UTIL_EOL
                       "    Pout    : %d dBm" UTIL_EOL
                       "    Tx Pin  : %s" UTIL_EOL
                       "    BW      : %s" UTIL_EOL
                       "    SF      : %s" UTIL_EOL
                       "    Sync    : 0x%02X" UTIL_EOL,
                       sx1276_frf_to_frequency(link.frf),
                       link.pout,
                       sx1276_rf_pin_str(link.rf_pin),
                       sx1276_bw_str(link.bw),
                       sx1276_sf_str(link.sf),
                       link.sync_word);
                printf("    Key     : ");
                util_print_uint32_hex(link.p_key, MESSAGE_BLOCK_LENGTH_U32);
                printf(UTIL_EOL);
            }
            break;
        }
    }
    while (0);

    
    sx1276_set_mode(sx1276_mode_sleep);
    
    
    ok = _nv_params_save(&_main.params); // Save the sequence number
    printf("Saved params to memory: %s" UTIL_EOL, ok ? "OK" : "FAILED");

    if (keyfobUI_button_is_busy())
    {
        printf("Waiting for buttons to clear..." UTIL_EOL);
        while (keyfobUI_button_is_busy());  // wait for the buttons to be released
    }

    if (keyfobUI_led_sequence_is_busy())
    {
        printf("Waiting for LED sequence to stop..." UTIL_EOL);
        while (keyfobUI_led_sequence_is_busy());
    }

    keyfobUI_led_status_set(keyfobUI_colour_yellow);

    if (hal_usb_stdio_is_running())
    {
        printf("USB Present. Restarting..." UTIL_EOL);
        goto restart;   // Otherwise it'll drop the USB link, which is a pain
    }
    else
    {
        hal_shutdown(); // Pull the plug. The hardware starts the system again...
    }

    keyfobUI_led_status_set(keyfobUI_colour_red);   // should never see this
}


/*
 * Private functions
 */
bool _nv_params_load(_nv_params_t* const pParams)
{
    assert(pParams != NULL);

    // No need for fancy reading functions: just do a table read
    bool const ok = (_p_params_nv->ident == _PARAMS_IDENT_VALUE);

    const void* const p_src = ok ? _p_params_nv : &_params_default;

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


void _dio_callback(void)
{
    _main._dio_callback_called = true;
}
