/**
 * Code common to the controller and receiver of the remote control application
 *
 * @author  David Hughes
 * @date    30Jun23
 */
#pragma once

#include <stdint.h>     // uint8_t, uint16_t, uint32_t, uint64_t
#include <stdbool.h>    // bool
#include <stddef.h>     // size_t

#include "sx1276.h"
#include "message.h"


/*
 * Public types
 */
#define RCTL_SHORT_PARAMS_LENGTH    (12)    // bytes
#define RCTL_MIN_PACKET_LEN         (2 * MESSAGE_BLOCK_LENGTH_BYTES)

#pragma pack(push, 1)
typedef struct
{
    // Plain:
    uint32_t p_iv[MESSAGE_BLOCK_LENGTH_U32];

    // AES:
    uint8_t id;
    uint8_t seq;
    uint8_t cid;
    uint8_t icid;
    uint8_t p_payload[RCTL_SHORT_PARAMS_LENGTH +
                      MESSAGE_BLOCK_LENGTH_BYTES];
}
rctl_message_t;
_Static_assert(((sizeof(rctl_message_t) % MESSAGE_BLOCK_LENGTH_BYTES) == 0),
               "rctl_message_t is not multiple of MESSAGE_BLOCK_LENGTH_BYTES");
#pragma pack(pop)


typedef struct
{
    uint8_t         sync_word;
    sx1276_sf_t     sf;
    sx1276_bw_t     bw;
    sx1276_cr_t     cr;
    sx1276_rf_pin_t rf_pin;
    int             pout;
    uint32_t        frf;
    uint16_t        preamble_length;
    uint32_t        p_key[MESSAGE_BLOCK_LENGTH_U32];
}
rctl_link_params_t;


/*
 * Public constants
 */
#define rctl_id_controller_to_receiver          (0x8D)
#define rctl_id_receiver_to_controller          (0x1C)


extern const rctl_link_params_t rctl_params_pairing;


// key fob -> receiver
#define rctl_cid_no_operation                   (0)
            
// Set output state         
#define rctl_cid_set_power_on                   (10)
#define rctl_cid_set_power_off                  (11)
#define rctl_cid_set_record_on                  (12)
#define rctl_cid_set_record_off                 (13)
#define rctl_cid_set_stream_on                  (14)
#define rctl_cid_set_stream_off                 (15)
            
#define rctl_cid_set_params                     (20)
#define rctl_cid_apply_params                   (21)
            
#define rctl_cid_pair                           (30)
            
#define rctl_cid_get_output_state               (110)
#define rctl_cid_get_params                     (120)
            
            
// receiver -> key fob          
#define rctl_cid_acknowledge                    (200)
            
#define rctl_cid_get_output_state_reply         (210)
#define rctl_cid_get_params_reply               (220)
            
#define rctl_cid_not_acknowledge                (255)
    
    
#define rctl_output_flags_power_output_state    (0)
#define rctl_output_flags_input_1_state         (1)
#define rctl_output_flags_input_2_state         (2)
#define rctl_output_flags_power_output_active   (3)

#define rctl_param_flags_use_pa_boost           (1 << 7)


/*
 * Public Functions
 */


/**
 * Applies the parameters to the hardware
 *
 * @param pParams   Parameter-set to apply (&_main.params or &_params_defaults)
 */
void rctl_link_params_apply(rctl_link_params_t* const pParams);


/**
 * Converts the structure to a sequence of bytes for transmission
 *
 * @param pParams       Parameter set to apply
 * @param pBuffer       Pointer to the destination buffer
 * @param BufferLength  Number of bytes available in pBuffer
 */
void rctl_link_params_serialise(rctl_link_params_t* const pParams,
                                uint8_t* const pBuffer,
                                size_t const BufferLength);


/**
 * Converts a sequence of bytes to the params structure
 *
 * @param pParams       Parameter set to apply
 * @param pBuffer       Pointer to the source buffer
 * @param BufferLength  Number of bytes available in pBuffer
 */
void rctl_link_params_deserialise(rctl_link_params_t* const pParams,
                                  uint8_t* const pBuffer,
                                  size_t const BufferLength);


/**
 * Converts the command Id to a string
 *
 * @param CommandID The command ID to convert
 * @return The value requested
 */
const char* rctl_cid_str(uint8_t const CID);
