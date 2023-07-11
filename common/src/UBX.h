/**
 * u-blox UBX protocol
 *
 * @author  David Hughes
 * @date    30Jun23
 */
#pragma once

#include <stdint.h>     // uint8_t
#include <stddef.h>     // size_t


/*
 * Public types
 */
typedef enum
{
    UBX_NONE                = 0,

    // Mnemonic             Class/ID    Length  Type    Description
    // UBX Class ACK        Ack/Nak Messages
    // Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
    UBX_ACK_ACK             = 0x0501,   // 2                Output Message acknowledged
    UBX_ACK_NAK             = 0x0500,   // 2                Output Message not acknowledged

    // UBX Class AID                      AssistNow Aiding Messages
    // AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
    UBX_AID_ALM             = 0x0B30,   // 0                Poll Request Poll GPS aiding almanac data
                                        // 1                Poll Request Poll GPS aiding almanac data for a SV
                                        // 8 or 40          Input/Output GPS aiding almanac input/output...

    UBX_AID_AOP             = 0x0B33,   // 0                Poll Request Poll AssistNow Autonomous data, all...
                                        // 1                Poll Request Poll AssistNow Autonomous data, one...
                                        // 68               Input/Output AssistNow Autonomous data
            
    UBX_AID_EPH             = 0x0B31,   // 0                Poll Request Poll GPS aiding ephemeris data
                                        // 1                Poll Request Poll GPS aiding ephemeris data for a SV
                                        // 8 or 104         Input/Output GPS aiding ephemeris input/output...
    UBX_AID_HUI             = 0x0B02,   // 0                Poll Request Poll GPS health, UTC, ionosphere...
                                        // 72               Input/Output GPS health, UTC and ionosphere...
            
    UBX_AID_INI             = 0x0B01,   // 0                Poll Request Poll GPS initial aiding data
                                        // 48               Input/Output Aiding position, time, frequency, clock...

    // UBX Class CFG        Configuration Input Messages
    // Configuration Input Messages: Configure the receiver
    UBX_CFG_ANT             = 0x0613,   // 4                Get/set Antenna control settings
    UBX_CFG_BATCH           = 0x0693,   // 8                Get/set Get/set data batching configuration
    UBX_CFG_CFG             = 0x0609,   // 12 or 13         Command Clear, save and load configurations
    UBX_CFG_DAT             = 0x0606,   // 44               Set Set user-defined datum
                                        // 52               Get Get currently defined datum

    UBX_CFG_DGNSS           = 0x0670,   // 4                Get/set DGNSS configuration
    UBX_CFG_DOSC            = 0x0661,   // 4 + 32*numOsc    Get/set Disciplined oscillator configuration
    UBX_CFG_ESFALG          = 0x0656,   // 12               Get/set Get/set IMU-mount misalignment...
    UBX_CFG_ESFA            = 0x064C,   // 20               Get/set Get/set the Accelerometer (A) sensor...
    UBX_CFG_ESFG            = 0x064D,   // 20               Get/set Get/set the Gyroscope (G) sensor...
    UBX_CFG_ESFWT           = 0x0682,   // 32               Get/set Get/set wheel-tick configuration
    UBX_CFG_ESRC            = 0x0660,   // 4 + 36*numSources Get/set External synchronization source...
    UBX_CFG_GEOFENCE        = 0x0669,   // 8 + 12*numFences Get/set Geofencing configuration
    UBX_CFG_GNSS            = 0x063E,   // 4 + 8*numCfgBlocks Get/set GNSS system configuration
    UBX_CFG_HNR             = 0x065C,   // 4                Get/set High navigation rate settings
    UBX_CFG_INF             = 0x0602,   // 1                Poll Request Poll configuration for one protocol
                                        // 0 + 10*N         Get/set Information message configuration

    UBX_CFG_ITFM            = 0x0639,   // 8                Get/set Jamming/interference monitor...
    UBX_CFG_LOGFILTER       = 0x0647,   // 12               Get/set Data logger configuration
    UBX_CFG_MSG             = 0x0601,   // 2                Poll Request Poll a message configuration
                                        // 8                Get/set Set message rate(s)
                                        // 3                Get/set Set message rate
            
    UBX_CFG_NAV5            = 0x0624,   // 36               Get/set Navigation engine settings
    UBX_CFG_NAVX5           = 0x0623,   // 40               Get/set Navigation engine expert settings
                                        // 44               Get/set Navigation engine expert settings
            
    UBX_CFG_NMEA            = 0x0617,   // 4                Get/set NMEA protocol configuration...
                                        // 12               Get/set NMEA protocol configuration V0...
                                        // 20               Get/set Extended NMEA protocol configuration V1
            
    UBX_CFG_ODO             = 0x061E,   // 20               Get/set Odometer, low-speed COG engine...
    UBX_CFG_PM2             = 0x063B,   // 44               Get/set Extended power management...
                                        // 48               Get/set Extended power management...

    UBX_CFG_PMS             = 0x0686,   // 8                Get/set Power mode setup
    UBX_CFG_PRT             = 0x0600,   // 1                Poll Request Polls the configuration for one I/O port
                                        // 20               Get/set Port configuration for UART ports
                                        // 20               Get/set Port configuration for USB port
                                        // 20               Get/set Port configuration for SPI port
                                        // 20               Get/set Port configuration for I2C (DDC) port

    UBX_CFG_PWR             = 0x0657,   // 8                Set Put receiver in a defined power state
    UBX_CFG_RATE            = 0x0608,   // 6                Get/set Navigation/measurement rate settings
    UBX_CFG_RINV            = 0x0634,   // 1 + 1*N          Get/set Contents of remote inventory
    UBX_CFG_RST             = 0x0604,   // 4                Command Reset receiver / Clear backup data...
    UBX_CFG_RXM             = 0x0611,   // 2                Get/set RXM configuration
    UBX_CFG_SBAS            = 0x0616,   // 8                Get/set SBAS configuration
    UBX_CFG_SENIF           = 0x0688,   // 6                Get/set I2C sensor interface configuration
    UBX_CFG_SLAS            = 0x068D,   // 4                Get/set SLAS configuration
    UBX_CFG_SMGR            = 0x0662,   // 20               Get/set Synchronization manager configuration
    UBX_CFG_SPT             = 0x0664,   // 12               Get/set Configure and start a sensor...
    UBX_CFG_TMODE2          = 0x063D,   // 28               Get/set Time mode settings 2
    UBX_CFG_TMODE3          = 0x0671,   // 40               Get/set Time mode settings 3

    UBX_CFG_TP5             = 0x0631,   // 0                Poll Request Poll time pulse parameters for time...
                                        // 1                Poll Request Poll time pulse parameters
                                        // 32               Get/set Time pulse parameters

    UBX_CFG_TXSLOT          = 0x0653,   // 16               Set TX buffer time slots configuration
    UBX_CFG_USB             = 0x061B,   // 108              Get/set USB configuration

    // UBX Class ESF External Sensor Fusion Messages
    // External Sensor Fusion Messages: External Sensor Measurements and Status Information
    UBX_ESF_ALG             = 0x1014,   // 16               Periodic/Polled IMU alignment information
    UBX_ESF_INS             = 0x1015,   // 36               Periodic/Polled Vehicle dynamics information
    UBX_ESF_MEAS            = 0x1002,   // 8 + 4*numMeas    Input/Output External sensor fusion measurements
    UBX_ESF_RAW             = 0x1003,   // 4 + 8*N          Output Raw sensor measurements
    UBX_ESF_STATUS          = 0x1010,   // 16 + 4*numSens   Periodic/Polled External sensor fusion status

    // UBX Class HNR High Rate Navigation Results Messages
    // High Rate Navigation Results Messages: High rate time, position, speed, heading
    UBX_HNR_ATT             = 0x2801,   // 32               Periodic/Polled Attitude solution
    UBX_HNR_INS             = 0x2802,   // 36               Periodic/Polled Vehicle dynamics information
    UBX_HNR_PVT             = 0x2800,   // 72               Periodic/Polled High rate output of PVT solution

    // UBX Class INF Information Messages
    // Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
    UBX_INF_DEBUG           = 0x0404,   // 0 + 1*N          Output ASCII output with debug contents
    UBX_INF_ERROR           = 0x0400,   // 0 + 1*N          Output ASCII output with error contents
    UBX_INF_NOTICE          = 0x0402,   // 0 + 1*N          Output ASCII output with informational contents
    UBX_INF_TEST            = 0x0403,   // 0 + 1*N          Output ASCII output with test contents
    UBX_INF_WARNING         = 0x0401,   // 0 + 1*N          Output ASCII output with warning contents

    // UBX Class LOG Logging Messages
    // Logging Messages: Log creation, deletion, info and retrieval
    UBX_LOG_BATCH           = 0x2111,   // 100              Polled Batched data
    UBX_LOG_CREATE          = 0x2107,   // 8                Command Create log file
    UBX_LOG_ERASE           = 0x2103,   // 0                Command Erase logged data
    UBX_LOG_FINDTIME        = 0x210E,   // 10               Input Find index of a log entry based on a...
                                        // 8                Output Response to FINDTIME request
            
    UBX_LOG_INFO            = 0x2108,   // 0                Poll Request Poll for log information
                                        // 48               Output Log information
            
    UBX_LOG_RETRIEVEBATCH   = 0x2110,   // 4                Command Request batch data
    UBX_LOG_RETRIEVEPOSEXTRA= 0x210F,   // 32               Output Odometer log entry
    UBX_LOG_RETRIEVEPOS     = 0x210B,   // 40               Output Position fix log entry
    UBX_LOG_RETRIEVESTRING  = 0x210D,   // 16 + 1*byteCount Output Byte string log entry
    UBX_LOG_RETRIEVE        = 0x2109,   // 12               Command Request log data
    UBX_LOG_STRING          = 0x2104,   // 0 + 1*N          Command Store arbitrary string in on-board flash

    // UBX Class MGA Multiple GNSS Assistance Messages
    // Multiple GNSS Assistance Messages: Assistance data for various GNSS
    UBX_MGA_ACK_DATA0       = 0x1360,   // 8                Output Multiple GNSS acknowledge message
    UBX_MGA_ANO             = 0x1320,   // 76               Input Multiple GNSS AssistNow Offline...

    UBX_MGA_BDS             = 0x1303,   // 88               Input BeiDou ephemeris assistance
                                        // 40               Input BeiDou almanac assistance
                                        // 68               Input BeiDou health assistance
                                        // 20               Input BeiDou UTC assistance
                                        // 16               Input BeiDou ionosphere assistance

    UBX_MGA_DBD             = 0x1380,   // 0                Poll Request Poll the navigation database
                                        // 12 + 1*N         Input/Output Navigation database dump entry

    UBX_MGA_FLASH           = 0x1321,   // 6 + 1*size       Input Transfer MGA-ANO data block to flash
                                        // 2                Input Finish flashing MGA-ANO data
                                        // 6                Output Acknowledge last FLASH-DATA or -STOP
            
    UBX_MGA_GAL             = 0x1302,   // 76               Input Galileo ephemeris assistance
                                        // 32               Input Galileo almanac assistance
                                        // 12               Input Galileo GPS time offset assistance
                                        // 20               Input Galileo UTC assistance
            
    UBX_MGA_GLO             = 0x1306,   // 48               Input GLONASS ephemeris assistance
                                        // 36               Input GLONASS almanac assistance
                                        // 20               Input GLONASS auxiliary time offset assistance
            
    UBX_MGA_GPS             = 0x1300,   // 68               Input GPS ephemeris assistance
                                        // 36               Input GPS almanac assistance
                                        // 40               Input GPS health assistance
                                        // 20               Input GPS UTC assistance
                                        // 16               Input GPS ionosphere assistance
            
    UBX_MGA_INI             = 0x1340,   // 20               Input Initial position assistance
                                        // 24               Input Initial time assistance
                                        // 12               Input Initial clock drift assistance
                                        // 12               Input Initial frequency assistance
                                        // 72               Input Earth orientation parameters assistance
            
    UBX_MGA_QZSS            = 0x1305,   // 68               Input QZSS ephemeris assistance
                                        // 36               Input QZSS almanac assistance
                                        // 12               Input QZSS health assistance

    // UBX Class MON Monitoring Messages
    // Monitoring Messages: Communication Status, Stack Usage, Task Status
    UBX_MON_BATCH           = 0x0A32,   // 12               Polled Data batching buffer status
    UBX_MON_GNSS            = 0x0A28,   // 8                Polled Information message major GNSS...
    UBX_MON_HW2             = 0x0A0B,   // 28               Periodic/Polled Extended hardware status
    UBX_MON_HW              = 0x0A09,   // 60               Periodic/polled Hardware status
    UBX_MON_IO              = 0x0A02,   // 0 + 20*N         Periodic/Polled I/O system status
    UBX_MON_MSGPP           = 0x0A06,   // 120              Periodic/Polled Message parse and process status
    UBX_MON_PATCH           = 0x0A27,   // 0                Poll Request Poll request for installed patches
                                        // 4 + 16*nEntries  Polled Installed patches

    UBX_MON_RXBUF           = 0x0A07,   // 24               Periodic/Polled Receiver buffer status
    UBX_MON_RXR             = 0x0A21,   // 1                Output Receiver status information
    UBX_MON_SMGR            = 0x0A2E,   // 16               Periodic/Polled Synchronization manager status
    UBX_MON_SPT             = 0x0A2F,   // 4 + 12*numRes    Polled Sensor production test
    UBX_MON_TXBUF           = 0x0A08,   // 28               Periodic/Polled Transmitter buffer status
    UBX_MON_VER             = 0x0A04,   // 0                Poll Request Poll receiver and software version
                                        // 40 + 30*N        Polled Receiver and software version

    // UBX Class NAV Navigation Results Messages
    // Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
    UBX_NAV_AOPSTATUS       = 0x0160,   // 16               Periodic/Polled AssistNow Autonomous status
    UBX_NAV_ATT             = 0x0105,   // 32               Periodic/Polled Attitude solution
    UBX_NAV_CLOCK           = 0x0122,   // 20               Periodic/Polled Clock solution
    UBX_NAV_COV             = 0x0136,   // 64               Periodic/Polled Covariance matrices
    UBX_NAV_DGPS            = 0x0131,   // 16 + 12*numCh    Periodic/Polled DGPS data used for NAV
    UBX_NAV_DOP             = 0x0104,   // 18               Periodic/Polled Dilution of precision
    UBX_NAV_EELL            = 0x013D,   // 16               Periodic/Polled Position error ellipse parameters
    UBX_NAV_EOE             = 0x0161,   // 4                Periodic End of epoch
    UBX_NAV_GEOFENCE        = 0x0139,   // 8 + 2*numFences  Periodic/Polled Geofencing status
    UBX_NAV_HPPOSECEF       = 0x0113,   // 28               Periodic/Polled High precision position solution in ECEF
    UBX_NAV_HPPOSLLH        = 0x0114,   // 36               Periodic/Polled High precision geodetic position solution
    UBX_NAV_NMI             = 0x0128,   // 16               Periodic/Polled Navigation message cross-check...
    UBX_NAV_ODO             = 0x0109,   // 20               Periodic/Polled Odometer solution
    UBX_NAV_ORB             = 0x0134,   // 8 + 6*numSv      Periodic/Polled GNSS orbit database info
    UBX_NAV_POSECEF         = 0x0101,   // 20               Periodic/Polled Position solution in ECEF
    UBX_NAV_POSLLH          = 0x0102,   // 28               Periodic/Polled Geodetic position solution
    UBX_NAV_PVT             = 0x0107,   // 92               Periodic/Polled Navigation position velocity time solution
    UBX_NAV_RELPOSNED       = 0x013C,   // 40               Periodic/Polled Relative positioning information in...
    UBX_NAV_RESETODO        = 0x0110,   // 0                Command Reset odometer
    UBX_NAV_SAT             = 0x0135,   // 8 + 12*numSvs    Periodic/Polled Satellite information
    UBX_NAV_SBAS            = 0x0132,   // 12 + 12*cnt      Periodic/Polled SBAS status data
    UBX_NAV_SLAS            = 0x0142,   // 20 + 8*cnt       Periodic/Polled QZSS L1S SLAS status data
    UBX_NAV_SOL             = 0x0106,   // 52               Periodic/Polled Navigation solution information
    UBX_NAV_STATUS          = 0x0103,   // 16               Periodic/Polled Receiver navigation status
    UBX_NAV_SVINFO          = 0x0130,   // 8 + 12*numCh     Periodic/Polled Space vehicle information
    UBX_NAV_SVIN            = 0x013B,   // 40               Periodic/Polled Survey-in data
    UBX_NAV_TIMEBDS         = 0x0124,   // 20               Periodic/Polled BeiDou time solution
    UBX_NAV_TIMEGAL         = 0x0125,   // 20               Periodic/Polled Galileo time solution
    UBX_NAV_TIMEGLO         = 0x0123,   // 20               Periodic/Polled GLONASS time solution
    UBX_NAV_TIMEGPS         = 0x0120,   // 16               Periodic/Polled GPS time solution
    UBX_NAV_TIMELS          = 0x0126,   // 24               Periodic/Polled Leap second event information
    UBX_NAV_TIMEUTC         = 0x0121,   // 20               Periodic/Polled UTC time solution
    UBX_NAV_VELECEF         = 0x0111,   // 20               Periodic/Polled Velocity solution in ECEF
    UBX_NAV_VELNED          = 0x0112,   // 36               Periodic/Polled Velocity solution in NED frame

    // UBX Class RXM Receiver Manager Messages
    // Receiver Manager Messages: Satellite Status, RTC Status
    UBX_RXM_IMES            = 0x0261,   // 4 + 44*numTx     Periodic/Polled Indoor Messaging System information
    UBX_RXM_MEASX           = 0x0214,   // 44 + 24*numSV    Periodic/Polled Satellite measurements for RRLP
    UBX_RXM_PMREQ           = 0x0241,   // 8                Command Power management request
                                        // 16               Command Power management request

    UBX_RXM_RAWX            = 0x0215,   // 16 + 32*numMeas  Periodic/Polled Multi-GNSS raw measurement data
                                        // 16 + 32*numMeas  Periodic/Polled Multi-GNSS raw measurements

    UBX_RXM_RLM             = 0x0259,   // 16               Output Galileo SAR short-RLM report
                                        // 28               Output Galileo SAR long-RLM report

    UBX_RXM_RTCM            = 0x0232,   // 8                Output RTCM input status
    UBX_RXM_SFRBX           = 0x0213,   // 8 + 4*numWords   Output Broadcast navigation data subframe
    UBX_RXM_SVSI            = 0x0220,   // 8 + 6*numSV      Periodic/Polled SV status info

    // UBX Class SEC Security Feature Messages
    UBX_SEC_UNIQID          = 0x2703,   // 9                Output Unique chip ID

    // UBX Class TIM Timing Messages
    // Timing Messages: Time Pulse Output, Time Mark Results
    UBX_TIM_DOSC            = 0x0D11,   // 8                Output Disciplined oscillator control
    UBX_TIM_FCHG            = 0x0D16,   // 32               Periodic/Polled Oscillator frequency changed notification
    UBX_TIM_HOC             = 0x0D17,   // 8                Input Host oscillator control
    UBX_TIM_SMEAS           = 0x0D13,   // 12 + 24*numMeas  Input/Output Source measurement
    UBX_TIM_SVIN            = 0x0D04,   // 28               Periodic/Polled Survey-in data
    UBX_TIM_TM2             = 0x0D03,   // 28               Periodic/Polled Time mark data
    UBX_TIM_TOS             = 0x0D12,   // 56               Periodic Time pulse time and frequency data
    UBX_TIM_TP              = 0x0D01,   // 16               Periodic/Polled Time pulse time data
    UBX_TIM_VCOCAL          = 0x0D15,   // 1                Command Stop calibration
                                        // 12               Command VCO calibration extended command
                                        // 12               Periodic/Polled Results of the calibration

    UBX_TIM_VRFY            = 0x0D06,   // 20               Periodic/Polled Sourced time verification

    // UBX Class UPD Firmware Update Messages
    // Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc
    UBX_UPD_SOS             = 0x0914,   // 0                Poll Request Poll backup restore status
                                        // 4                Command Create backup in flash
                                        // 4                Command Clear backup in flash
                                        // 8                Output Backup creation acknowledge
                                        // 8                Output System restored from backup

    // NMEA Standard Messages Standard messages
    // Ref: M8 Receiver Description Manual §31.1.9
    // These are used with UBX-CFG-MSG
    UBX_NMEA_DTM            = 0xF00A,   // Datum reference
    UBX_NMEA_GBQ            = 0xF044,   // Poll a standard message (Talker ID GB)
    UBX_NMEA_GBS            = 0xF009,   // GNSS satellite fault detection
    UBX_NMEA_GGA            = 0xF000,   // Global positioning system fix data
    UBX_NMEA_GLL            = 0xF001,   // Latitude and longitude, with time of position fix and status
    UBX_NMEA_GLQ            = 0xF043,   // Poll a standard message (Talker ID GL)
    UBX_NMEA_GNQ            = 0xF042,   // Poll a standard message (Talker ID GN)
    UBX_NMEA_GNS            = 0xF00D,   // GNSS fix data
    UBX_NMEA_GPQ            = 0xF040,   // Poll a standard message (Talker ID GP)
    UBX_NMEA_GRS            = 0xF006,   // GNSS range residuals
    UBX_NMEA_GSA            = 0xF002,   // GNSS DOP and active satellites
    UBX_NMEA_GST            = 0xF007,   // GNSS pseudorange error statistics
    UBX_NMEA_GSV            = 0xF003,   // GNSS satellites in view
    UBX_NMEA_RMC            = 0xF004,   // Recommended minimum data
    UBX_NMEA_THS            = 0xF00E,   // True heading and status
    UBX_NMEA_TXT            = 0xF041,   // Text transmission
    UBX_NMEA_VLW            = 0xF00F,   // Dual ground/water distance
    UBX_NMEA_VTG            = 0xF005,   // Course over ground and ground speed
    UBX_NMEA_ZDA            = 0xF008,   // Time and date

    // NMEA PUBX Messages Proprietary messages
    UBX_PUBX_CONFIG         = 0xF141,   // Set protocols and baud rate
    UBX_PUBX_POSITION       = 0xF100,   // Lat/Long position data
    UBX_PUBX_RATE           = 0xF140,   // Set NMEA message output rate
    UBX_PUBX_SVSTATUS       = 0xF103,   // Satellite status
    UBX_PUBX_TIME           = 0xF104,   // Time of day and clock information
}
UBX_cls_id_t;


typedef enum
{
    UBX_decode_state_sync_1,  // initial
    UBX_decode_state_sync_2,
    UBX_decode_state_class,
    UBX_decode_state_id,
    UBX_decode_state_length_lsb,
    UBX_decode_state_length_msb,
    UBX_decode_state_data,
    UBX_decode_state_ck_a,
    UBX_decode_state_ck_b,
    UBX_decode_state_done,
    UBX_decode_state_sync_error,
    UBX_decode_state_ck_error,
}
UBX_decode_state_t;


typedef struct
{
    void* p_payload;        // Pointer to the message payload buffer
    size_t payload_length;  // Size of the payload buffer (bytes)

    UBX_cls_id_t cls_id;
    size_t length;          // Message length (bytes)
                            // @note This may be greater than payload_length.
                            // If reading p_payload, do not read past payload_length

    UBX_decode_state_t state;   // Test against _done or _error to ascertain message validity
    size_t count;

    uint8_t ck_a;           // Calculated checksum values
    uint8_t ck_b;

    uint8_t ck_a_r;         // Received checksum values
    uint8_t ck_b_r;
}
UBX_t;


#pragma pack(push, 1)
/*
 * Structured payload definitions
 */
typedef struct
{
    uint32_t iTOW;      // (ms) GPS time of week of the navigation epoch.
    uint16_t year;      // (y)  Year (UTC)
    uint8_t  month;     // (month) Month, range 1..12 (UTC)
    uint8_t  day;       // (d)  Day of month, range 1..31 (UTC)
    uint8_t  hour;      // (h)  Hour of day, range 0..23 (UTC)
    uint8_t  min;       // (min) Minute of hour, range 0..59 (UTC)
    uint8_t  sec;       // (s)  Seconds of minute, range 0..60 (UTC)
    uint8_t  valid;     // Validity flags
    uint32_t tAcc;      // (ns) Time accuracy estimate (UTC)
    int32_t  nano;      // (ns) Fraction of second, range -1e9 .. 1e9 (UTC)
    uint8_t  fixType;   // GNSSfix Type:
                        //      0: no fix
                        //      1: dead reckoning only
                        //      2: 2D-fix
                        //      3: 3D-fix
                        //      4: GNSS + dead reckoning combined
                        //      5: time only fix
    uint8_t  flags;     // Fix status flags
    uint8_t  flags2;    // Additional flags
    uint8_t  numSV;     // Number of satellites used in Nav Solution
    int32_t  lon;       // (*1e-7 deg) Longitude
    int32_t  lat;       // (*1e-7 deg) Latitude
    int32_t  height;    // (mm) Height above ellipsoid
    int32_t  hMSL;      // (mm) Height above mean sea level
    uint32_t hAcc;      // (mm) Horizontal accuracy estimate
    uint32_t vAcc;      // (mm) Vertical accuracy estimate
    int32_t  velN;      // (mm/s) NED north velocity
    int32_t  velE;      // (mm/s) NED east velocity
    int32_t  velD;      // (mm/s) NED down velocity
    int32_t  gSpeed;    // (mm/s) Ground Speed (2-D)
    int32_t  headMot;   // (*1e-5 deg) Heading of motion (2-D)
    uint32_t sAcc;      // (mm/s) Speed accuracy estimate
    uint32_t headAcc;   // (*1e-5 deg) Heading accuracy estimate (both motion
                        // and vehicle)
    uint16_t pDOP;      // (*0.01) Position DOP
    uint16_t flags3;    // Additional flags
    uint8_t  reserved1[4];   // Reserved
    int32_t  headVeh;   // (*1e-5 deg) Heading of vehicle (2-D), this is only
                        // valid when headVehValid is set, otherwise the output
                        // is set to the heading of motion
    int16_t  magDec;    // (*1e-2 deg) Magnetic declination.
                        // Only supported in ADR 4.10 and later.
    uint16_t magAcc;    // (*1e-2 deg) Magnetic declination accuracy.
                        // Only supported in ADR 4.10 and later.
}
UBX_NAV_PVT_t;

#define UBX_NAV_PVT_fixType_no_fix                      (0)
#define UBX_NAV_PVT_fixType_dead_reckoning_only         (1)
#define UBX_NAV_PVT_fixType_2d_fix                      (2)
#define UBX_NAV_PVT_fixType_3d_fix                      (3)
#define UBX_NAV_PVT_fixType_gnss_dead_reckoning         (4)
#define UBX_NAV_PVT_fixType_time_fix_only               (5)

#define UBX_NAV_PVT_valid_validDate                     (1 << 0)
#define UBX_NAV_PVT_valid_validTime                     (1 << 1)
#define UBX_NAV_PVT_valid_fullyResolved                 (1 << 2)
#define UBX_NAV_PVT_valid_validMag                      (1 << 3)

#define UBX_NAV_PVT_flags_gnssFixOK                     (1 << 0)
#define UBX_NAV_PVT_flags_diffSoln                      (1 << 1)
#define UBX_NAV_PVT_flags_headVehValid                  (1 << 5)

#define UBX_NAV_PVT_flags_carrSoln_noo_range_solution   (0 << 6)
#define UBX_NAV_PVT_flags_carrSoln_floating_ambiguities (1 << 6)
#define UBX_NAV_PVT_flags_carrSoln_fixed_ambiguities    (2 << 6)

#define UBX_NAV_PVT_flags2_confirmedAvai                (1 << 5)
#define UBX_NAV_PVT_flags2_confirmedData                (1 << 6)
#define UBX_NAV_PVT_flags2_confirmedTime                (1 << 7)

#define UBX_NAV_PVT_flags3_invalidLlh                   (1 << 0)    // Invalid lon, lat, height and hMSL

#define UBX_NAV_PVT_flags3_lastCorrectionAge_0_1        (1 << 1)
#define UBX_NAV_PVT_flags3_lastCorrectionAge_1_2        (2 << 1)
#define UBX_NAV_PVT_flags3_lastCorrectionAge_2_5        (3 << 1)
#define UBX_NAV_PVT_flags3_lastCorrectionAge_5_10       (4 << 1)
#define UBX_NAV_PVT_flags3_lastCorrectionAge_10_15      (5 << 1)
#define UBX_NAV_PVT_flags3_lastCorrectionAge_15_20      (6 << 1)
#define UBX_NAV_PVT_flags3_lastCorrectionAge_20_30      (7 << 1)
#define UBX_NAV_PVT_flags3_lastCorrectionAge_30_45      (8 << 1)
#define UBX_NAV_PVT_flags3_lastCorrectionAge_45_60      (9 << 1)
#define UBX_NAV_PVT_flags3_lastCorrectionAge_60_90      (10 << 1)
#define UBX_NAV_PVT_flags3_lastCorrectionAge_90_120     (11 << 1)
#define UBX_NAV_PVT_flags3_lastCorrectionAge_120_       (12 << 1)

_Static_assert(sizeof(UBX_NAV_PVT_t) == 92, "UBX_NAV_PVT_t size is wrong");

#pragma pack(pop)


#define UBX_PROTOCOL_OVERHEAD_BYTES (8)


/*
 * Public functions
 */


/**
 * Makes a UBX formatted block of data from the class and id and payloads
 *
 * @param pOutput       Pointer to the output buffer
 * @param OutputLength  Number of bytes available in the output buffer
 * @param ClsID         The combined Class and ID of the packet
 * @param pPayload      Pointer to the buffer containing the payload
 *                      Can be NULL if there is no payload
 * @param PayloadLength Number of bytes of payload. Can be zero.
 * @return Number of bytes written to pOutput
 */
size_t UBX_encode(void* pOutput, size_t const OutputLength,
                  UBX_cls_id_t const ClsID,
                  void* const pPayload, size_t const PayloadLength);


/**
 * Starts a new decode session
 *
 * @param pUBX          Pointer to the UBX structure to receive the packet into
 * @param pPayload      Pointer to the payload buffer
 * @param PayloadLength Number of buytes available in the Payload buffer
 */
void UBX_decode_init(UBX_t* const pUBX,
                     void* const pPayload, size_t const PayloadLength);


/**
 * Resets and prepares a UBX structure to receive a new packet.
 *
 * @param pUBX          Pointer to the UBX structure
 */
void UBX_decode_reset(UBX_t* const pUBX);


/**
 * Decodes an input buffer, looking for UBX frames. Stops after the first one
 *
 * @param pUBX          Pointer to the UBX structure
 * @param pInput        Pointer to the buffer of data to be decoded
 * @param InputLength   Number of bytes in the Input buffer
 * @return Number of bytes consumed from the input buffer
 */
size_t UBX_decode(UBX_t* const pUBX,
                  void* const pInput, size_t const InputLength);


/**
 * Utility to retrieve the class from the combined ClsID code
 *
 * @param ClsID         The combined Class and ID
 * @return The value requested
 */
uint8_t UBX_cls_id_get_cls(UBX_cls_id_t const ClsID);


/**
 * Utility to retrieve the ID from the combined ClsID code
 *
 * @param ClsID         The combined Class and ID
 * @return The value requested
 */
uint8_t UBX_cls_id_get_id(UBX_cls_id_t const ClsID);


/**
 * Utility to make a combined ClsID code from a buffer with first byte as Class
 * and second byte as ID
 *
 * @param pBuffer         A buffer containing the class and id values
 * @return The value requested
 */
UBX_cls_id_t UBX_cls_id_from_uint8(uint8_t* pBuffer);


/**
 * Utility to convert the combined class and id variable to a string
 *
 * @param ClsID         The value of the combined Class and ID
 * @return The value requested
 */
const char* UBX_cls_id_str(UBX_cls_id_t const ClsID);


/**
 * Utility to convert the decoder state variable to a string
 *
 * @param State         The value of the decoder state variable
 * @return The value requested
 */
const char* UBX_decode_state_str(UBX_decode_state_t const State);


/**
 * Utility to convert a UBX-NAV-PVT packet FixType value to a string
 *
 * @param FixType       The value of the FixType field in a UBX-NAV-PVT packet
 * @return The value requested
 */
const char* UBX_NAV_PVT_fixType_str(uint8_t const FixType);
