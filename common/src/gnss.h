/**
 * GPS functionality using CAM-M8 and u-blox UBX protocol
 *
 * @author  David Hughes
 * @date    30Jun23
 */
#pragma once

#include <stdint.h>     // uint8_t
#include <stdbool.h>    // bool

#include "camm8.h"
#include "UBX.h"


/*
 * Public functions
 */


void gnss_flush(camm8_t* const pGNSS);


bool gnss_get_ubx(camm8_t* const pGNSS,
                  UBX_t* const pUBX);


bool gnss_get_ack(camm8_t* const pGNSS);


void gnss_set_NMEA_rate(camm8_t* const pGNSS,
                        UBX_cls_id_t const MsgClassID,  // NMEA message ident
                        uint8_t const Rate);


void gnss_set_sleep(camm8_t* const pGNSS);
