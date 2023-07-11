#include "rctl.h"

#include <stdint.h>     // uint8_t, uint16_t, uint32_t, uint64_t
#include <stdbool.h>    // bool
#include <stddef.h>     // size_t
#include <assert.h>
#include <string.h>     // memset

#include "message.h"
#include "sx1276.h"
#include "util.h"


/*
 * Public constants
 */
const rctl_link_params_t rctl_params_pairing =
{
    .sync_word          = 0x15,
    .sf                 = sx1276_sf_7,
    .bw                 = sx1276_bw_125kHz,
    .cr                 = sx1276_cr_4_8,
    .rf_pin             = sx1276_rf_pin_rfo,// RF Output
    .pout               = 0,                // dBm
    .frf                = 14221312,         // sx1276_frequency_to_frf(LORA_CH_17_868)
    .preamble_length    =  16,
    .p_key              = { 0xA1568A62, 0x57F2B141, 0xEFEB7C2C, 0x23C49C91 },
};


/*
 * Public Functions
 */
void rctl_link_params_apply(rctl_link_params_t* const pParams)
{
    assert(pParams != NULL);

    sx1276_set_frf(pParams->frf);
    sx1276_set_tx_power(pParams->pout, pParams->rf_pin);
    sx1276_set_spreading_factor(pParams->sf);
    sx1276_set_bandwidth(pParams->bw);
    sx1276_set_coding_rate(pParams->cr);
    sx1276_set_sync_word(pParams->sync_word);
    sx1276_set_preamble_length(pParams->preamble_length);
    
    message_set_key(pParams->p_key);
}


void rctl_link_params_serialise(rctl_link_params_t* const pParams,
                                uint8_t* const pBuffer,
                                size_t const BufferLength)
{
    assert(pParams != NULL);
    assert(pBuffer != NULL);
    assert(BufferLength >= (RCTL_SHORT_PARAMS_LENGTH +
                            MESSAGE_BLOCK_LENGTH_BYTES));

    pBuffer[0] = pParams->sync_word;

    pBuffer[1] = ((((uint8_t)pParams->sf) & 0x0F) << 4) |   // Note enum values
                 ((((uint8_t)pParams->bw) & 0x0F) << 0);    // Note enum values

    pBuffer[2] = ((pParams->rf_pin == sx1276_rf_pin_pa_boost) ?
                    rctl_param_flags_use_pa_boost : 0)  |
                 ((((uint8_t)pParams->cr) & 0x07) << 5) |   // Note enum values
                 ((((uint8_t)pParams->pout) & 0x1F) << 0);

    util_uint24_to_bytes_le(&pBuffer[3], pParams->frf); // 3..5
    util_uint16_to_bytes_le(&pBuffer[6], pParams->preamble_length); // 6..7

    memset(&pBuffer[8], 0, RCTL_SHORT_PARAMS_LENGTH - 8); // Reserved

    memcpy(&pBuffer[12], pParams->p_key, sizeof(pParams->p_key));
}


void rctl_link_params_deserialise(rctl_link_params_t* const pParams,
                                  uint8_t* const pBuffer,
                                  size_t const BufferLength)
{
    assert(pParams != NULL);
    assert(pBuffer != NULL);
    assert(BufferLength >= (RCTL_SHORT_PARAMS_LENGTH +
                            MESSAGE_BLOCK_LENGTH_BYTES));

    pParams->sync_word          = pBuffer[0];
    pParams->sf                 = (sx1276_sf_t) ((pBuffer[1] >> 4) & 0x0F);
    pParams->bw                 = (sx1276_bw_t) ((pBuffer[1] >> 0) & 0x0F);
    pParams->cr                 = (sx1276_cr_t) ((pBuffer[2] >> 5) & 0x07);
                                
    pParams->rf_pin             = (pBuffer[2] & rctl_param_flags_use_pa_boost) ?
                                    sx1276_rf_pin_pa_boost : sx1276_rf_pin_rfo;
    pParams->pout               = pBuffer[2] & 0x1F;

    pParams->frf                = util_bytes_to_uint24_le(&pBuffer[3]); // 3..5
    pParams->preamble_length    = util_bytes_to_uint16_le(&pBuffer[6]); // 6..7

    memcpy(pParams->p_key, &pBuffer[12], sizeof(pParams->p_key));
}


const char* rctl_cid_str(uint8_t const CID)
{
    const char* str;

    switch (CID)
    {
    // controller -> receiver
    case rctl_cid_no_operation:             str = "No Operation";       break;
                                            
    // Set output state commands            
    case rctl_cid_set_power_on:             str = "Set Power On";       break;
    case rctl_cid_set_power_off:            str = "Set Power Off";      break;
    case rctl_cid_set_record_on:            str = "Set Record On";      break;
    case rctl_cid_set_record_off:           str = "Set Record Off";     break;
    case rctl_cid_set_stream_on:            str = "Set Stream On";      break;
    case rctl_cid_set_stream_off:           str = "Set Stream Off";     break;
                                            
    case rctl_cid_set_params:               str = "Set Params";         break;
    case rctl_cid_apply_params:             str = "Apply Params";       break;
                                            
    case rctl_cid_pair:                     str = "Pair Device";        break;
                                            
    case rctl_cid_get_output_state:         str = "Get Output State";   break;
    case rctl_cid_get_params:               str = "Get Params";         break;
                                            
    // receiver -> controller               
    case rctl_cid_acknowledge:              str = "Acknowledge";        break;
                                            
    case rctl_cid_get_output_state_reply:   str = "Output State Reply"; break;
    case rctl_cid_get_params_reply:         str = "Params Reply";       break;
                                            
    case rctl_cid_not_acknowledge:          str = "Not Acknowledge";    break;
    default:                                str = "<unknown>";          break;
    }

    return str;
}
