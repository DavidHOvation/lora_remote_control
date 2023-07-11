#include "UBX.h"

#include <stdint.h>     // uint8_t
#include <stddef.h>     // NULL, size_t

#include <string.h>     // memcpy


/*
 * Private constants
 */
#define _UBX_HEADER_LEN     (6) // bytes
#define _UBX_OVERHEAD       (8) // bytes

#define _UBX_SYNC_CHAR_1    (0xB5)
#define _UBX_SYNC_CHAR_2    (0x62)


/*
 * Public functions
 */
size_t UBX_encode(void* pOutput, size_t const OutputLength,
                  UBX_cls_id_t const ClsID,
                  void* const pPayload, size_t const PayloadLength)
{
    uint8_t* const p = (uint8_t*)pOutput;

    p[0] = _UBX_SYNC_CHAR_1;
    p[1] = _UBX_SYNC_CHAR_2;
    p[2] = UBX_cls_id_get_cls(ClsID);
    p[3] = UBX_cls_id_get_id(ClsID);
    p[4] = (uint8_t)PayloadLength;          // Payload Length LSB
    p[5] = (uint8_t)(PayloadLength >> 8);   // Payload Length MSB
    memcpy(&p[6], pPayload, PayloadLength);

    // Calculate checksum over the range byte #2 to the end of the payload
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    for (uint16_t i = 2; i < (PayloadLength + 6); i++)
    {
        ck_a += p[i];
        ck_b += ck_a;
    }

    p[PayloadLength + 6] = ck_a;
    p[PayloadLength + 7] = ck_b;

    return PayloadLength + _UBX_OVERHEAD;
}


void UBX_decode_init(UBX_t* const pUBX,
                     void* const pPayload, size_t const PayloadLength)
{
    pUBX->p_payload         = pPayload;
    pUBX->payload_length    = PayloadLength;

    UBX_decode_reset(pUBX);
}


void UBX_decode_reset(UBX_t* const pUBX)
{
    pUBX->cls_id    = UBX_NONE;
    pUBX->length    = 0;

    pUBX->state     = UBX_decode_state_sync_1;
    pUBX->count     = 0;
    pUBX->ck_a      = 0;
    pUBX->ck_b      = 0;

    pUBX->ck_a_r    = 0;
    pUBX->ck_b_r    = 0;
}


size_t UBX_decode(UBX_t* const pUBX,
                  void* const pInput, size_t const InputLength)
{
    size_t i;
    for (i = 0; i < InputLength; i++)
    {
        uint8_t const c = ((uint8_t*)pInput)[i];

        switch (pUBX->state)
        {
        default:
        case UBX_decode_state_sync_1:
            if (c == _UBX_SYNC_CHAR_1)
            {
                pUBX->state = UBX_decode_state_sync_2;
            }
            break;

        case UBX_decode_state_sync_2:
            if (c == _UBX_SYNC_CHAR_2)
            {
                pUBX->state = UBX_decode_state_class;
            }
            else
            {
                pUBX->state = UBX_decode_state_sync_error;
            }
            break;

        case UBX_decode_state_class:
            pUBX->cls_id = ((uint16_t)c) << 8;
            pUBX->ck_a  = c;
            pUBX->ck_b  = pUBX->ck_a;
            pUBX->state = UBX_decode_state_id;
            break;

        case UBX_decode_state_id:
            pUBX->cls_id |= (uint16_t)c;
            pUBX->ck_a += c;
            pUBX->ck_b += pUBX->ck_a;
            pUBX->state = UBX_decode_state_length_lsb;
            break;

        case UBX_decode_state_length_lsb:
            pUBX->length = (size_t)c;
            pUBX->ck_a += c;
            pUBX->ck_b += pUBX->ck_a;
            pUBX->state = UBX_decode_state_length_msb;
            break;

        case UBX_decode_state_length_msb:
            pUBX->length |= ((size_t)c) << 8;
            pUBX->ck_a += c;
            pUBX->ck_b += pUBX->ck_a;
            pUBX->state = UBX_decode_state_data;
            break;

        case UBX_decode_state_data:
            if (pUBX->count < pUBX->payload_length)
            {
                ((uint8_t*)pUBX->p_payload)[pUBX->count] = c;
            }

            pUBX->ck_a += c;
            pUBX->ck_b += pUBX->ck_a;

            pUBX->count++;
            if (pUBX->count >= pUBX->length)
            {
                pUBX->state = UBX_decode_state_ck_a;
            }
            break;

        case UBX_decode_state_ck_a:
            pUBX->ck_a_r = c;
            pUBX->state = UBX_decode_state_ck_b;
            break;

        case UBX_decode_state_ck_b:
            pUBX->ck_b_r = c;

            if ((pUBX->ck_a_r == pUBX->ck_a) && (pUBX->ck_b_r == pUBX->ck_b))
            {
                pUBX->state = UBX_decode_state_done;
            }
            else
            {
                pUBX->state = UBX_decode_state_ck_error;
            }
            break;

        case UBX_decode_state_done:
        case UBX_decode_state_sync_error:
        case UBX_decode_state_ck_error:
            break;
        }

        if ((pUBX->state == UBX_decode_state_done)        ||
            (pUBX->state == UBX_decode_state_sync_error)  ||
            (pUBX->state == UBX_decode_state_ck_error))
        {
            i++;    // Count the current byte
            break;
        }
    }

    return i;
}


uint8_t UBX_cls_id_get_cls(UBX_cls_id_t const ClsID)
{
    return ((uint16_t)ClsID) >> 8;
}


uint8_t UBX_cls_id_get_id(UBX_cls_id_t const ClsID)
{
    return (uint8_t)ClsID;
}


UBX_cls_id_t UBX_cls_id_from_uint8(uint8_t* pBuffer)
{
    return (((uint16_t)pBuffer[0]) << 8) | (((uint16_t)pBuffer[1]) << 0);
}


const char* UBX_cls_id_str(UBX_cls_id_t const ClsID)
{
    const char* str;

    switch (ClsID)
    {
    case UBX_NONE:                  str = "UBX_NONE";                   break;

    case UBX_ACK_ACK:               str = "UBX_ACK_ACK";                break;
    case UBX_ACK_NAK:               str = "UBX_ACK_NAK";                break;

    case UBX_AID_ALM:               str = "UBX_AID_ALM";                break;
    case UBX_AID_AOP:               str = "UBX_AID_AOP";                break;
    case UBX_AID_EPH:               str = "UBX_AID_EPH";                break;
    case UBX_AID_HUI:               str = "UBX_AID_HUI";                break;
    case UBX_AID_INI:               str = "UBX_AID_INI";                break;

    case UBX_CFG_ANT:               str = "UBX_CFG_ANT";                break;
    case UBX_CFG_BATCH:             str = "UBX_CFG_BATCH";              break;
    case UBX_CFG_CFG:               str = "UBX_CFG_CFG";                break;
    case UBX_CFG_DAT:               str = "UBX_CFG_DAT";                break;
    case UBX_CFG_DGNSS:             str = "UBX_CFG_DGNSS";              break;
    case UBX_CFG_DOSC:              str = "UBX_CFG_DOSC";               break;
    case UBX_CFG_ESFALG:            str = "UBX_CFG_ESFALG";             break;
    case UBX_CFG_ESFA:              str = "UBX_CFG_ESFA";               break;
    case UBX_CFG_ESFG:              str = "UBX_CFG_ESFG";               break;
    case UBX_CFG_ESFWT:             str = "UBX_CFG_ESFWT";              break;
    case UBX_CFG_ESRC:              str = "UBX_CFG_ESRC";               break;
    case UBX_CFG_GEOFENCE:          str = "UBX_CFG_GEOFENCE";           break;
    case UBX_CFG_GNSS:              str = "UBX_CFG_GNSS";               break;
    case UBX_CFG_HNR:               str = "UBX_CFG_HNR";                break;
    case UBX_CFG_INF:               str = "UBX_CFG_INF";                break;
    case UBX_CFG_ITFM:              str = "UBX_CFG_ITFM";               break;
    case UBX_CFG_LOGFILTER:         str = "UBX_CFG_LOGFILTER";          break;
    case UBX_CFG_MSG:               str = "UBX_CFG_MSG";                break;
    case UBX_CFG_NAV5:              str = "UBX_CFG_NAV5";               break;
    case UBX_CFG_NAVX5:             str = "UBX_CFG_NAVX5";              break;
    case UBX_CFG_NMEA:              str = "UBX_CFG_NMEA";               break;
    case UBX_CFG_ODO:               str = "UBX_CFG_ODO";                break;
    case UBX_CFG_PM2:               str = "UBX_CFG_PM2";                break;
    case UBX_CFG_PMS:               str = "UBX_CFG_PMS";                break;
    case UBX_CFG_PRT:               str = "UBX_CFG_PRT";                break;
    case UBX_CFG_PWR:               str = "UBX_CFG_PWR";                break;
    case UBX_CFG_RATE:              str = "UBX_CFG_RATE";               break;
    case UBX_CFG_RINV:              str = "UBX_CFG_RINV";               break;
    case UBX_CFG_RST:               str = "UBX_CFG_RST";                break;
    case UBX_CFG_RXM:               str = "UBX_CFG_RXM";                break;
    case UBX_CFG_SBAS:              str = "UBX_CFG_SBAS";               break;
    case UBX_CFG_SENIF:             str = "UBX_CFG_SENIF";              break;
    case UBX_CFG_SLAS:              str = "UBX_CFG_SLAS";               break;
    case UBX_CFG_SMGR:              str = "UBX_CFG_SMGR";               break;
    case UBX_CFG_SPT:               str = "UBX_CFG_SPT";                break;
    case UBX_CFG_TMODE2:            str = "UBX_CFG_TMODE2";             break;
    case UBX_CFG_TMODE3:            str = "UBX_CFG_TMODE3";             break;
    case UBX_CFG_TP5:               str = "UBX_CFG_TP5";                break;
    case UBX_CFG_TXSLOT:            str = "UBX_CFG_TXSLOT";             break;
    case UBX_CFG_USB:               str = "UBX_CFG_USB";                break;

    case UBX_ESF_ALG:               str = "UBX_ESF_ALG";                break;
    case UBX_ESF_INS:               str = "UBX_ESF_INS";                break;
    case UBX_ESF_MEAS:              str = "UBX_ESF_MEAS";               break;
    case UBX_ESF_RAW:               str = "UBX_ESF_RAW";                break;
    case UBX_ESF_STATUS:            str = "UBX_ESF_STATUS";             break;

    case UBX_HNR_ATT:               str = "UBX_HNR_ATT";                break;
    case UBX_HNR_INS:               str = "UBX_HNR_INS";                break;
    case UBX_HNR_PVT:               str = "UBX_HNR_PVT";                break;

    case UBX_INF_DEBUG:             str = "UBX_INF_DEBUG";              break;
    case UBX_INF_ERROR:             str = "UBX_INF_ERROR";              break;
    case UBX_INF_NOTICE:            str = "UBX_INF_NOTICE";             break;
    case UBX_INF_TEST:              str = "UBX_INF_TEST";               break;
    case UBX_INF_WARNING:           str = "UBX_INF_WARNING";            break;

    case UBX_LOG_BATCH:             str = "UBX_LOG_BATCH";              break;
    case UBX_LOG_CREATE:            str = "UBX_LOG_CREATE";             break;
    case UBX_LOG_ERASE:             str = "UBX_LOG_ERASE";              break;
    case UBX_LOG_FINDTIME:          str = "UBX_LOG_FINDTIME";           break;
    case UBX_LOG_INFO:              str = "UBX_LOG_INFO";               break;
    case UBX_LOG_RETRIEVEBATCH:     str = "UBX_LOG_RETRIEVEBATCH";      break;
    case UBX_LOG_RETRIEVEPOSEXTRA:  str = "UBX_LOG_RETRIEVEPOSEXTRA";   break;
    case UBX_LOG_RETRIEVEPOS:       str = "UBX_LOG_RETRIEVEPOS";        break;
    case UBX_LOG_RETRIEVESTRING:    str = "UBX_LOG_RETRIEVESTRING";     break;
    case UBX_LOG_RETRIEVE:          str = "UBX_LOG_RETRIEVE";           break;
    case UBX_LOG_STRING:            str = "UBX_LOG_STRING";             break;

    case UBX_MGA_ACK_DATA0:         str = "UBX_MGA_ACK_DATA0";          break;
    case UBX_MGA_ANO:               str = "UBX_MGA_ANO";                break;
    case UBX_MGA_BDS:               str = "UBX_MGA_BDS";                break;
    case UBX_MGA_DBD:               str = "UBX_MGA_DBD";                break;
    case UBX_MGA_FLASH:             str = "UBX_MGA_FLASH";              break;
    case UBX_MGA_GAL:               str = "UBX_MGA_GAL";                break;
    case UBX_MGA_GLO:               str = "UBX_MGA_GLO";                break;
    case UBX_MGA_GPS:               str = "UBX_MGA_GPS";                break;
    case UBX_MGA_INI:               str = "UBX_MGA_INI";                break;
    case UBX_MGA_QZSS:              str = "UBX_MGA_QZSS";               break;

    case UBX_MON_BATCH:             str = "UBX_MON_BATCH";              break;
    case UBX_MON_GNSS:              str = "UBX_MON_GNSS";               break;
    case UBX_MON_HW2:               str = "UBX_MON_HW2";                break;
    case UBX_MON_HW:                str = "UBX_MON_HW";                 break;
    case UBX_MON_IO:                str = "UBX_MON_IO";                 break;
    case UBX_MON_MSGPP:             str = "UBX_MON_MSGPP";              break;
    case UBX_MON_PATCH:             str = "UBX_MON_PATCH";              break;
    case UBX_MON_RXBUF:             str = "UBX_MON_RXBUF";              break;
    case UBX_MON_RXR:               str = "UBX_MON_RXR";                break;
    case UBX_MON_SMGR:              str = "UBX_MON_SMGR";               break;
    case UBX_MON_SPT:               str = "UBX_MON_SPT";                break;
    case UBX_MON_TXBUF:             str = "UBX_MON_TXBUF";              break;
    case UBX_MON_VER:               str = "UBX_MON_VER";                break;

    case UBX_NAV_AOPSTATUS:         str = "UBX_NAV_AOPSTATUS";          break;
    case UBX_NAV_ATT:               str = "UBX_NAV_ATT";                break;
    case UBX_NAV_CLOCK:             str = "UBX_NAV_CLOCK";              break;
    case UBX_NAV_COV:               str = "UBX_NAV_COV";                break;
    case UBX_NAV_DGPS:              str = "UBX_NAV_DGPS";               break;
    case UBX_NAV_DOP:               str = "UBX_NAV_DOP";                break;
    case UBX_NAV_EELL:              str = "UBX_NAV_EELL";               break;
    case UBX_NAV_EOE:               str = "UBX_NAV_EOE";                break;
    case UBX_NAV_GEOFENCE:          str = "UBX_NAV_GEOFENCE";           break;
    case UBX_NAV_HPPOSECEF:         str = "UBX_NAV_HPPOSECEF";          break;
    case UBX_NAV_HPPOSLLH:          str = "UBX_NAV_HPPOSLLH";           break;
    case UBX_NAV_NMI:               str = "UBX_NAV_NMI";                break;
    case UBX_NAV_ODO:               str = "UBX_NAV_ODO";                break;
    case UBX_NAV_ORB:               str = "UBX_NAV_ORB";                break;
    case UBX_NAV_POSECEF:           str = "UBX_NAV_POSECEF";            break;
    case UBX_NAV_POSLLH:            str = "UBX_NAV_POSLLH";             break;
    case UBX_NAV_PVT:               str = "UBX_NAV_PVT";                break;
    case UBX_NAV_RELPOSNED:         str = "UBX_NAV_RELPOSNED";          break;
    case UBX_NAV_RESETODO:          str = "UBX_NAV_RESETODO";           break;
    case UBX_NAV_SAT:               str = "UBX_NAV_SAT";                break;
    case UBX_NAV_SBAS:              str = "UBX_NAV_SBAS";               break;
    case UBX_NAV_SLAS:              str = "UBX_NAV_SLAS";               break;
    case UBX_NAV_SOL:               str = "UBX_NAV_SOL";                break;
    case UBX_NAV_STATUS:            str = "UBX_NAV_STATUS";             break;
    case UBX_NAV_SVINFO:            str = "UBX_NAV_SVINFO";             break;
    case UBX_NAV_SVIN:              str = "UBX_NAV_SVIN";               break;
    case UBX_NAV_TIMEBDS:           str = "UBX_NAV_TIMEBDS";            break;
    case UBX_NAV_TIMEGAL:           str = "UBX_NAV_TIMEGAL";            break;
    case UBX_NAV_TIMEGLO:           str = "UBX_NAV_TIMEGLO";            break;
    case UBX_NAV_TIMEGPS:           str = "UBX_NAV_TIMEGPS";            break;
    case UBX_NAV_TIMELS:            str = "UBX_NAV_TIMELS";             break;
    case UBX_NAV_TIMEUTC:           str = "UBX_NAV_TIMEUTC";            break;
    case UBX_NAV_VELECEF:           str = "UBX_NAV_VELECEF";            break;
    case UBX_NAV_VELNED:            str = "UBX_NAV_VELNED";             break;

    case UBX_RXM_IMES:              str = "UBX_RXM_IMES";               break;
    case UBX_RXM_MEASX:             str = "UBX_RXM_MEASX";              break;
    case UBX_RXM_PMREQ:             str = "UBX_RXM_PMREQ";              break;
    case UBX_RXM_RAWX:              str = "UBX_RXM_RAWX";               break;
    case UBX_RXM_RLM:               str = "UBX_RXM_RLM";                break;
    case UBX_RXM_RTCM:              str = "UBX_RXM_RTCM";               break;
    case UBX_RXM_SFRBX:             str = "UBX_RXM_SFRBX";              break;
    case UBX_RXM_SVSI:              str = "UBX_RXM_SVSI";               break;

    case UBX_SEC_UNIQID:            str = "UBX_SEC_UNIQID";             break;

    case UBX_TIM_DOSC:              str = "UBX_TIM_DOSC";               break;
    case UBX_TIM_FCHG:              str = "UBX_TIM_FCHG";               break;
    case UBX_TIM_HOC:               str = "UBX_TIM_HOC";                break;
    case UBX_TIM_SMEAS:             str = "UBX_TIM_SMEAS";              break;
    case UBX_TIM_SVIN:              str = "UBX_TIM_SVIN";               break;
    case UBX_TIM_TM2:               str = "UBX_TIM_TM2";                break;
    case UBX_TIM_TOS:               str = "UBX_TIM_TOS";                break;
    case UBX_TIM_TP:                str = "UBX_TIM_TP";                 break;
    case UBX_TIM_VCOCAL:            str = "UBX_TIM_VCOCAL";             break;
    case UBX_TIM_VRFY:              str = "UBX_TIM_VRFY";               break;

    case UBX_UPD_SOS:               str = "UBX_UPD_SOS";                break;

    case UBX_NMEA_DTM:              str = "UBX_NMEA_DTM";               break;
    case UBX_NMEA_GBQ:              str = "UBX_NMEA_GBQ";               break;
    case UBX_NMEA_GBS:              str = "UBX_NMEA_GBS";               break;
    case UBX_NMEA_GGA:              str = "UBX_NMEA_GGA";               break;
    case UBX_NMEA_GLL:              str = "UBX_NMEA_GLL";               break;
    case UBX_NMEA_GLQ:              str = "UBX_NMEA_GLQ";               break;
    case UBX_NMEA_GNQ:              str = "UBX_NMEA_GNQ";               break;
    case UBX_NMEA_GNS:              str = "UBX_NMEA_GNS";               break;
    case UBX_NMEA_GPQ:              str = "UBX_NMEA_GPQ";               break;
    case UBX_NMEA_GRS:              str = "UBX_NMEA_GRS";               break;
    case UBX_NMEA_GSA:              str = "UBX_NMEA_GSA";               break;
    case UBX_NMEA_GST:              str = "UBX_NMEA_GST";               break;
    case UBX_NMEA_GSV:              str = "UBX_NMEA_GSV";               break;
    case UBX_NMEA_RMC:              str = "UBX_NMEA_RMC";               break;
    case UBX_NMEA_THS:              str = "UBX_NMEA_THS";               break;
    case UBX_NMEA_TXT:              str = "UBX_NMEA_TXT";               break;
    case UBX_NMEA_VLW:              str = "UBX_NMEA_VLW";               break;
    case UBX_NMEA_VTG:              str = "UBX_NMEA_VTG";               break;
    case UBX_NMEA_ZDA:              str = "UBX_NMEA_ZDA";               break;

    case UBX_PUBX_CONFIG:           str = "UBX_PUBX_CONFIG";            break;
    case UBX_PUBX_POSITION:         str = "UBX_PUBX_POSITION";          break;
    case UBX_PUBX_RATE:             str = "UBX_PUBX_RATE";              break;
    case UBX_PUBX_SVSTATUS:         str = "UBX_PUBX_SVSTATUS";          break;
    case UBX_PUBX_TIME:             str = "UBX_PUBX_TIME";              break;

    default:                        str = "UBX <unknown>";              break;
    }

    return str;
}


const char* UBX_decode_state_str(UBX_decode_state_t const State)
{
    const char* str;

    switch (State)
    {
    case UBX_decode_state_sync_1:       str = "UBX_decode_state_sync_1";        break;
    case UBX_decode_state_sync_2:       str = "UBX_decode_state_sync_2";        break;
    case UBX_decode_state_class:        str = "UBX_decode_state_class";         break;
    case UBX_decode_state_id:           str = "UBX_decode_state_id";            break;
    case UBX_decode_state_length_lsb:   str = "UBX_decode_state_length_lsb";    break;
    case UBX_decode_state_length_msb:   str = "UBX_decode_state_length_msb";    break;
    case UBX_decode_state_data:         str = "UBX_decode_state_data";          break;
    case UBX_decode_state_ck_a:         str = "UBX_decode_state_ck_a";          break;
    case UBX_decode_state_ck_b:         str = "UBX_decode_state_ck_b";          break;
    case UBX_decode_state_done:         str = "UBX_decode_state_done";          break;
    case UBX_decode_state_sync_error:   str = "UBX_decode_state_sync_error";    break;
    case UBX_decode_state_ck_error:     str = "UBX_decode_state_ck_error";      break;

    default:                            str = "UBX_decode_state <unknown>";     break;
    }

    return str;
}


const char* UBX_NAV_PVT_fixType_str(uint8_t const FixType)
{
    const char* str;

    switch (FixType)
    {
    case UBX_NAV_PVT_fixType_no_fix:                str = "No Fix";                 break;
    case UBX_NAV_PVT_fixType_dead_reckoning_only:   str = "Dead Reckoning Only";    break;
    case UBX_NAV_PVT_fixType_2d_fix:                str = "2D-Fix";                 break;
    case UBX_NAV_PVT_fixType_3d_fix:                str = "3D-Fix";                 break;
    case UBX_NAV_PVT_fixType_gnss_dead_reckoning:   str = "GNSS + Dead Reckoning";  break;
    case UBX_NAV_PVT_fixType_time_fix_only:         str = "Time Fix Only";          break;
    default:                                        str = "<knknown>";              break;
    }

    return str;
}
