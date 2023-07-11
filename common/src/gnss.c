#include "gnss.h"

#include <stdint.h>     // uint8_t
#include <stddef.h>     // size_t
#include <stdbool.h>    // bool, false, true

#include "camm8.h"
#include "UBX.h"


/*
 * Public functions
 */
void gnss_flush(camm8_t* const pGNSS)
{
    size_t total_bytes = 0;
    
    while (1)
    {
        uint8_t p_msg[256];

        size_t const read_len = camm8_read(pGNSS, p_msg, sizeof(p_msg));
        if (read_len < sizeof(p_msg)) break;
        
        total_bytes += read_len;
    }
}


bool gnss_get_ubx(camm8_t* const pGNSS,
                  UBX_t* const pUBX)
{
    bool ok = false;
    uint8_t p_rxd[256];

    size_t const rxd_len = camm8_read(pGNSS, p_rxd, sizeof(p_rxd));
    
    size_t read_index = 0;
    while (read_index < rxd_len)
    {
        size_t consumed = UBX_decode(pUBX,
                                     p_rxd + read_index,
                                     rxd_len - read_index);

        if (pUBX->state == UBX_decode_state_done)
        {
            ok = true;
            break;
        }
        
        read_index += consumed;
    }

    return ok;
}


bool gnss_get_ack(camm8_t* const pGNSS)
{
    UBX_t ubx_rx;
    UBX_decode_init(&ubx_rx, NULL, 0);  // No payload required
    bool const ok = gnss_get_ubx(pGNSS, &ubx_rx);
    return ok && (ubx_rx.cls_id == UBX_ACK_ACK);
}


void gnss_set_NMEA_rate(camm8_t* const pGNSS,
                        UBX_cls_id_t const MsgClassID,  // NMEA message ident
                        uint8_t const Rate)
{
    uint8_t p_txd[16];  // Overhead (8) + payload (8)
    uint8_t p_payload[8] =
    {
        UBX_cls_id_get_cls(MsgClassID),
        UBX_cls_id_get_id(MsgClassID),
        Rate,   // DDC Port rate
        0,      // UART1 Rate
        0,      // USB Rate
        0,      // SPI Rate
        0,
        0,
    };
        
    size_t txd_len = UBX_encode(p_txd, sizeof(p_txd),
                                UBX_CFG_MSG,
                                p_payload, sizeof(p_payload));
        
    gnss_flush(pGNSS);
    camm8_write(pGNSS, p_txd, txd_len);
}


void gnss_set_sleep(camm8_t* const pGNSS)
{
    uint8_t const p_payload[16] =
    {
        0x00,       // version
        0, 0, 0,    // reserved1
        0,          // duration msec (0 = wakeup signal on pin)
        0, 0, 0, 0, // flags
        0x60, 0, 0, 0, // wakeupSources
    };

    uint8_t p_txd[UBX_PROTOCOL_OVERHEAD_BYTES + sizeof(p_payload)];
    
    size_t txd_len = UBX_encode(p_txd, sizeof(p_txd),
                                UBX_RXM_PMREQ,
                                p_payload, sizeof(p_payload));
    
    gnss_flush(pGNSS);
    camm8_write(pGNSS, p_txd, txd_len);
}
