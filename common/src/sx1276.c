#include "sx1276.h"
/**
 * Semtech SX1276 magic numbers and drivers
 * @ref Semtech DS_SX1276-7-8-9_W_APP_V7 (Data sheet)
 * @ref Semtech _SX1276_77_8_ErrataNote_1.1_STD (Errata Note)
 *
 * @author  David Hughes
 * @date    15Jun23
 */
#include <inttypes.h>   // PRI printf macros
#include <stdint.h>     // uint8_t, uint32_t, uint64_t

#include <stdio.h>      // printf (test functions)

#include "hal.h"
#include "lora.h"
#include "frequency.h"
#include "util.h"       // util_clear, debugging printing


/*
 * Private constants
 */
#define _SX1276_MASK(BitH, BitL)                    ((1 << ((BitH) - (BitL) + 1)) - 1)  // Value Mask
#define _SX1276_REG_MASK(BitH, BitL)                (_SX1276_MASK(BitH, BitL) << (BitL))  // Register Mask
#define _SX1276_REG_VALUE(Value, BitH, BitL)        ((((uint8_t)(Value)) << (BitL)) &  _SX1276_REG_MASK(BitH, BitL))
#define _SX1276_SET_REG(RegVal, Value, BitH, BitL)  (((RegVal) & ~_SX1276_REG_MASK(BitH, BitL)) | _SX1276_REG_VALUE(Value, BitH, BitL))
#define _SX1276_GET_REG(RegVal, BitH, BitL)         (((RegVal) & _SX1276_REG_MASK(BitH, BitL)) >> (BitL))

#define _SX1276_BIT(BitNo)                  (1 << (BitNo))


#define _SX1276_FOSC                        FREQUENCY_MHZ(32)

#define _SX1276_SPI_SPACE                   (0xFF)  // Value to send if only reading

// Frequency threshold where some of the magic numbers change
#define _SX1276_HF_FREQ_HZ                  FREQUENCY_MHZ(862.0)
#define _SX1276_FRF_HF_FREQ                 14123008UL  // Corresponding FRF register value

#define _SX1276_RSSI_LF_OFFSET              (-164)
#define _SX1276_RSSI_HF_OFFSET              (-157)

#define _SX1276_LOW_DATA_SYMBOL_PERIOD_MS   (16)

#define _SX1276_REG_WRITE(RegID)            (0x80 | ((uint8_t)RegID))
#define _SX1276_REG_READ(RegID)             ((uint8_t)RegID)

#define _SX1276_FIFO_SIZE                   (256)   // bytes
#define _SX1276_FIFO_TX_BASE_ADDR           (0)     // TX set to start of FIFO for max capacity
#define _SX1276_FIFO_RX_BASE_ADDR           (0)     // RX set to start of FIFO for max capacity

#define _SX1276_SYMB_TIMEOUT_MAX            ((1 << 10) - 1)


// Register IDs
typedef enum
{
    _sx1276_reg_fifo                        = 0x00,
    _sx1276_reg_op_mode                     = 0x01,
    // Reserved 0x02 - 0x05
    _sx1276_reg_frf_msb                     = 0x06,
    _sx1276_reg_frf_mid                     = 0x07,
    _sx1276_reg_frf_lsb                     = 0x08,
    _sx1276_reg_pa_config                   = 0x09,
    _sx1276_reg_pa_ramp                     = 0x0A,
    _sx1276_reg_ocp                         = 0x0B,
    _sx1276_reg_lna                         = 0x0C,
    _sx1276_reg_fifo_addr_ptr               = 0x0D,
    _sx1276_reg_fifo_tx_base_addr           = 0x0E,
    _sx1276_reg_fifo_rx_base_addr           = 0x0F,
    _sx1276_reg_fifo_rx_current_addr        = 0x10,
    _sx1276_reg_irq_flags_mask              = 0x11,
    _sx1276_reg_irq_flags                   = 0x12,
    _sx1276_reg_rx_nb_bytes                 = 0x13,
    _sx1276_reg_rx_header_cnt_value_msb     = 0x14,
    _sx1276_reg_rx_header_cnt_value_lsb     = 0x15,
    _sx1276_reg_rx_packet_cnt_value_msb     = 0x16,
    _sx1276_reg_rx_packet_cnt_value_lsb     = 0x17,
    _sx1276_reg_modem_stat                  = 0x18,
    _sx1276_reg_pkt_snr_value               = 0x19,
    _sx1276_reg_pkt_rssi_value              = 0x1A,
    _sx1276_reg_rssi_value                  = 0x1B,
    _sx1276_reg_hop_channel                 = 0x1C,
    _sx1276_reg_modem_config_1              = 0x1D,
    _sx1276_reg_modem_config_2              = 0x1E,
    _sx1276_reg_symb_timeout_lsb            = 0x1F,
    _sx1276_reg_preamble_msb                = 0x20,
    _sx1276_reg_preamble_lsb                = 0x21,
    _sx1276_reg_payload_length              = 0x22,
    _sx1276_reg_max_payload_length          = 0x23,
    _sx1276_reg_hop_period                  = 0x24,
    _sx1276_reg_fifo_rx_byte_addr           = 0x25,
    _sx1276_reg_modem_config_3              = 0x26,
    _sx1276_reg_ppm_correction              = 0x27,
    _sx1276_reg_fei_msb                     = 0x28,
    _sx1276_reg_fei_mid                     = 0x29,
    _sx1276_reg_fei_lsb                     = 0x2A,
    // Reserved 0x2B
    _sx1276_reg_rssi_wideband               = 0x2C,
    // Reserved 0x2D - 0x2E
    _sx1276_reg_if_freq_2                   = 0x2F,  // See errata
    _sx1276_reg_if_freq_1                   = 0x30,  // See errata
    _sx1276_reg_detect_optimize             = 0x31,
    // Reserved 0x32
    _sx1276_reg_invert_iq                   = 0x33,
    // Reserved 0x34 - 0x35
    _sx1276_reg_high_bw_optimize_1          = 0x36,
    _sx1276_reg_detection_threshold         = 0x37,
    // Reserved 0x38
    _sx1276_reg_sync_word                   = 0x39,
    _sx1276_reg_high_bw_optimize_2          = 0x3A,
    _sx1276_reg_invert_iq_2                 = 0x3B,
    // Reserved 0x3C - 0x3F
    _sx1276_reg_dio_mapping_1               = 0x40,
    _sx1276_reg_dio_mapping_2               = 0x41,
    _sx1276_reg_version                     = 0x42,
    // Reserved 0x43
    // Unused 0x44
    // Reserved 0x45 - 0x4A
    _sx1276_reg_tcxo                        = 0x4B,
    // Reserved 0x4C
    _sx1276_reg_pa_dac                      = 0x4D,
    // Reserved 0x4E - 0x5A
    // Unused 0x5B
    // Reserved 0x5C - 0x60
    _sx1276_reg_agc_ref                     = 0x61,
    _sx1276_reg_agc_thresh_1                = 0x62,
    _sx1276_reg_agc_thresh_2                = 0x63,
    _sx1276_reg_agc_thresh_3                = 0x64,
    // Reserved 0x65 - 0x6F
    _sx1276_reg_pll                         = 0x70,
    // REG_TEST 0x71 - 0x7F
}
_sx1276_reg_t;

// FSK-mode registers
typedef enum
{
    _sx1276_fsk_reg_timer_resol             = 0x38,
    _sx1276_fsk_reg_timer_1_coefficient     = 0x39,
    _sx1276_fsk_reg_timer_2_coefficient     = 0x3A,
}
_sx1276_fsk_reg_t;


// Register bits and fields
// Macros ending "_VR" convert the Field value to Register value
// Macros ending "_RV" convert the Register value to Field value

// _sx1276_reg_op_mode
#define _SX1276_LONG_RANGE_MODE_SET(RegVal, Value)  _SX1276_SET_REG(RegVal, Value, 7, 7)  // AKA LoRa
#define _SX1276_LONG_RANGE_MODE_GET(RegVal)         _SX1276_GET_REG(RegVal, 7, 7)

#define _SX1276_ACCESS_SHARED_REG_SET(RegVal, Value) _SX1276_SET_REG(RegVal, Value, 6, 6)
#define _SX1276_ACCESS_SHARED_REG_GET(RegVal)       _SX1276_GET_REG(RegVal, 6, 6)

#define _SX1276_LOW_FREQUENCY_MODE_ON_SET(RegVal, Value) _SX1276_SET_REG(RegVal, Value, 3, 3)
#define _SX1276_LOW_FREQUENCY_MODE_ON_GET(RegVal)       _SX1276_GET_REG(RegVal, 3, 3)

#define _SX1276_MODE_SET(RegVal, Value)             _SX1276_SET_REG(RegVal, Value, 2, 0)
#define _SX1276_MODE_GET(RegVal)                    _SX1276_GET_REG(RegVal, 2, 0)

// Also sx1276_mode_t

// _sx1276_reg_pa_config
#define _SX1276_PA_SELECT_SET(RegVal, Value)        _SX1276_SET_REG(RegVal, Value, 7, 7)
#define _SX1276_PA_SELECT_GET(RegVal)               _SX1276_GET_REG(RegVal, 7, 7)

typedef enum
{
    _sx1276_max_power_10_8dBm   = 0,
    _sx1276_max_power_11_4dBm   = 1,
    _sx1276_max_power_12dBm     = 2,
    _sx1276_max_power_12_6dBm   = 3,
    _sx1276_max_power_13_2dBm   = 4,
    _sx1276_max_power_13_8dBm   = 5,
    _sx1276_max_power_14_4dBm   = 6,
    _sx1276_max_power_15dBm     = 7,
}
_sx1276_max_power_t;

#define _SX1276_MAX_POWER_SET(RegVal, Value)        _SX1276_SET_REG(RegVal, Value, 6, 4)
#define _SX1276_MAX_POWER_GET(RegVal)               _SX1276_GET_REG(RegVal, 6, 4)

#define _SX1276_OUTPUT_POWER_SET(RegVal, Value)     _SX1276_SET_REG(RegVal, Value, 3, 0)
#define _SX1276_OUTPUT_POWER_GET(RegVal)            _SX1276_GET_REG(RegVal, 3, 0)

// _sx1276_reg_lna
#define _SX1276_LNA_GAIN_VALUE(Value)               _SX1276_REG_VALUE(Value, 7, 5)
#define _SX1276_LNA_GAIN_GET(RegVal)                _SX1276_GET_REG(RegVal, 7, 5)

#define _SX1276_LNA_BOOST_HF                        _SX1276_REG_VALUE(3, 1, 0)

// _sx1276_reg_irq_flags_mask (set '1' to disable the flag)
#define _SX1276_RX_TIMEOUT_MASK                     _SX1276_BIT(7)
#define _SX1276_RX_DONE_MASK                        _SX1276_BIT(6)
#define _SX1276_PAYLOAD_CRC_ERROR_MASK              _SX1276_BIT(5)
#define _SX1276_VALID_HEADER_MASK                   _SX1276_BIT(4)
#define _SX1276_TX_DONE_MASK                        _SX1276_BIT(3)
#define _SX1276_CAD_DONE_MASK                       _SX1276_BIT(2)
#define _SX1276_FHSS_CHANGE_CHANNEL_MASK            _SX1276_BIT(1)
#define _SX1276_CAD_DETECTED_MASK                   _SX1276_BIT(0)

// _sx1276_reg_irq_flags (Write '1' to clear each bit)
#define _SX1276_RX_TIMEOUT                          _SX1276_RX_TIMEOUT_MASK
#define _SX1276_RX_DONE                             _SX1276_RX_DONE_MASK
#define _SX1276_PAYLOAD_CRC_ERROR                   _SX1276_PAYLOAD_CRC_ERROR_MASK
#define _SX1276_VALID_HEADER                        _SX1276_VALID_HEADER_MASK
#define _SX1276_TX_DONE                             _SX1276_TX_DONE_MASK
#define _SX1276_CAD_DONE                            _SX1276_CAD_DONE_MASK
#define _SX1276_FHSS_CHANGE_CHANNEL                 _SX1276_FHSS_CHANGE_CHANNEL_MASK
#define _SX1276_CAD_DETECTED                        _SX1276_CAD_DETECTED_MASK

#define _SX1276_IRQ_ALL                             _SX1276_REG_VALUE(0xFF, 7, 0)

// _sx1276_reg_modem_stat
#define _SX1276_RX_CODING_RATE_SET(RegVal, Value)   _SX1276_SET_REG(RegVal, Value, 7, 5)
#define _SX1276_RX_CODING_RATE_GET(RegVal)          _SX1276_GET_REG(RegVal, 7, 5)

#define _SX1276_MODEM_STATUS_MODEM_CLEAR            _SX1276_BIT(4)
#define _SX1276_MODEM_STATUS_HEADER_INFO_VALID      _SX1276_BIT(3)
#define _SX1276_MODEM_STATUS_RX_ONGOING             _SX1276_BIT(2)
#define _SX1276_MODEM_STATUS_SIGNAL_SYNCHRONIZED    _SX1276_BIT(1)
#define _SX1276_MODEM_STATUS_SIGNAL_DETECTED        _SX1276_BIT(0)

// _sx1276_reg_hop_channel
#define _SX1276_PLL_TIMEOUT                         _SX1276_BIT(7)
#define _SX1276_CRC_ON_PAYLOAD                      _SX1276_BIT(6)

#define _SX1276_FHSS_PRESENT_CHANNEL_SET(RegVal, Value)  _SX1276_SET_REG(RegVal, Value, 5, 0)
#define _SX1276_FHSS_PRESENT_CHANNEL_GET(RegVal)    _SX1276_GET_REG(RegVal, 5, 0)

// _sx1276_reg_modem_config_1
#define _SX1276_BW_SET(RegVal, Value)               _SX1276_SET_REG(RegVal, Value, 7, 4)
#define _SX1276_BW_GET(RegVal)                      _SX1276_GET_REG(RegVal, 7, 4)

#define _SX1276_CODING_RATE_SET(RegVal, Value)      _SX1276_SET_REG(RegVal, Value, 3, 1)
#define _SX1276_CODING_RATE_GET(RegVal)             _SX1276_GET_REG(RegVal, 3, 1)

#define _SX1276_IMPLICIT_HEADER_MODE_ON             _SX1276_BIT(0)

// _sx1276_reg_modem_config_2
#define _SX1276_SPREADING_FACTOR_SET(RegVal, Value) _SX1276_SET_REG(RegVal, Value, 7, 4)
#define _SX1276_SPREADING_FACTOR_GET(RegVal)        _SX1276_GET_REG(RegVal, 7, 4)

#define _SX1276_TX_CONTUINUOUS_MODE                 _SX1276_BIT(3)
#define _SX1276_RX_PAYLOAD_CRC_ON                   _SX1276_BIT(2)

#define _SX1276_SYMB_TIMEOUT_9_8_SET(RegVal, Value) _SX1276_SET_REG(RegVal, Value, 1, 0)
#define _SX1276_SYMB_TIMEOUT_9_8_GET(RegVal)        _SX1276_GET_REG(RegVal, 1, 0)

// _sx1276_reg_detect_optimize
// There's a non-zero reserved field in this register!
#define _SX1276_AUTOMATIC_IF_ON_SET(RegVal, Value)  _SX1276_SET_REG(RegVal, Value, 7, 7)
#define _SX1276_AUTOMATIC_IF_ON_GET(RegVal)         _SX1276_GET_REG(RegVal, 7, 7)

#define _SX1276_REG_DETECT_OPTIMIZE_DEFAULT         (0xC3)  // 0xC0 | 0x03

#define _SX1276_DETECTION_OPTIMIZE_SET(RegVal, Value) _SX1276_SET_REG(RegVal, Value, 2, 0)
#define _SX1276_DETECTION_OPTIMIZE_GET(RegVal)      _SX1276_GET_REG(RegVal, 2, 0)

// _sx1276_reg_modem_config_3
#define _SX1276_LOW_DATA_RATE_OPTIMIZE_SET(RegVal, Value) _SX1276_SET_REG(RegVal, Value, 3, 3)
#define _SX1276_LOW_DATA_RATE_OPTIMIZE_GET(RegVal)  _SX1276_GET_REG(RegVal, 3, 3)

#define _SX1276_AGC_AUTO_ON                         _SX1276_BIT(2)

// _sx1276_reg_dio_mapping_1
#define _SX1276_DIO0_MAPPING_SET(RegVal, Value)     _SX1276_SET_REG(RegVal, Value, 7, 6)
#define _SX1276_DIO0_MAPPING_GET(RegVal)            _SX1276_GET_REG(RegVal, 7, 6)

#define _SX1276_DIO1_MAPPING_SET(RegVal, Value)     _SX1276_SET_REG(RegVal, Value, 5, 4)
#define _SX1276_DIO1_MAPPING_GET(RegVal)            _SX1276_GET_REG(RegVal, 5, 4)

#define _SX1276_DIO2_MAPPING_SET(RegVal, Value)     _SX1276_SET_REG(RegVal, Value, 3, 2)
#define _SX1276_DIO2_MAPPING_GET(RegVal)            _SX1276_GET_REG(RegVal, 3, 2)

#define _SX1276_DIO3_MAPPING_SET(RegVal, Value)     _SX1276_SET_REG(RegVal, Value, 1, 0)
#define _SX1276_DIO3_MAPPING_GET(RegVal)            _SX1276_GET_REG(RegVal, 1, 0)

// _sx1276_reg_dio_mapping_2
#define _SX1276_DIO4_MAPPING_SET(RegVal, Value)      _SX1276_SET_REG(RegVal, Value, 7, 6)
#define _SX1276_DIO4_MAPPING_GET(RegVal)             _SX1276_GET_REG(RegVal, 7, 6)

#define _SX1276_DIO5_MAPPING_SET(RegVal, Value)     _SX1276_SET_REG(RegVal, Value, 5, 4)
#define _SX1276_DIO5_MAPPING_GET(RegVal)            _SX1276_GET_REG(RegVal, 5, 4)

#define _SX1276_MAP_PREAMBLE_DETECT                 _SX1276_BIT(0)

// _sx1276_reg_tcxo
#define _SX1276_REG_TCXO_DEFAULT                    (0x09)
#define _SX1276_TCXO_INPUT_ON                       _SX1276_BIT(4)

// _sx1276_reg_if_freq_2
#define _SX1276_REG_IF_FREQ_2_DEFAULT               (0x20)

// _sx1276_reg_timer_resol (FSK-mode Register)
#define _SX1276_TIMER1_RESOLUTION_SET(RegVal, Value) _SX1276_SET_REG(RegVal, Value, 3, 2)
#define _SX1276_TIMER2_RESOLUTION_SET(RegVal, Value) _SX1276_SET_REG(RegVal, Value, 1, 0)


/*
 * Public constants
 */
uint8_t const sx1276_version_expected               = 0x12;


uint8_t const sx1276_header_mode_explicit           = 0;


frequency_t const sx1276_frequency_delta            = 61;   // FOSC >> 19


const sx1276_irq_t sx1276_irq_rx_timeout            = _SX1276_RX_TIMEOUT;
const sx1276_irq_t sx1276_irq_rx_done               = _SX1276_RX_DONE;
const sx1276_irq_t sx1276_irq_payload_crc_error     = _SX1276_PAYLOAD_CRC_ERROR;
const sx1276_irq_t sx1276_irq_valid_header          = _SX1276_VALID_HEADER;
const sx1276_irq_t sx1276_irq_tx_done               = _SX1276_TX_DONE;
const sx1276_irq_t sx1276_irq_cad_done              = _SX1276_CAD_DONE;
const sx1276_irq_t sx1276_irq_fhss_change_channel   = _SX1276_FHSS_CHANGE_CHANNEL;
const sx1276_irq_t sx1276_irq_cad_detected          = _SX1276_CAD_DETECTED;


// DIO Pin and Mapping values are concatenated for ease:
// bits 7:4 specifies the pin; 3:0 is the mapping
#define _SX1276_DIO_0_RX_DONE                       (0x00)
#define _SX1276_DIO_0_TX_DONE                       (0x01)
#define _SX1276_DIO_0_CAD_DONE                      (0x02)

#define _SX1276_DIO_1_RX_TIMEOUT                    (0x10)
#define _SX1276_DIO_1_FHSS_CHANGE_CHANNEL           (0x11)
#define _SX1276_DIO_1_CAD_DETECTED                  (0x12)

#define _SX1276_DIO_2_FHSS_CHANGE_CHANNEL           (0x20)

#define _SX1276_DIO_3_CAD_DONE                      (0x30)
#define _SX1276_DIO_3_VALID_HEADER                  (0x31)
#define _SX1276_DIO_3_PAYLOAD_CRC_ERROR             (0x32)

#define _SX1276_DIO_4_CAD_DETECTED                  (0x40)
#define _SX1276_DIO_4_PLL_LOCK                      (0x41)

#define _SX1276_DIO_5_MODE_READY                    (0x50)
#define _SX1276_DIO_5_CLK_OUT                       (0x51)


sx1276_dio_mapping_t const sx1276_dio_0_rx_done                 = _SX1276_DIO_0_RX_DONE;
sx1276_dio_mapping_t const sx1276_dio_0_tx_done                 = _SX1276_DIO_0_TX_DONE;
sx1276_dio_mapping_t const sx1276_dio_0_cad_done                = _SX1276_DIO_0_CAD_DONE;

sx1276_dio_mapping_t const sx1276_dio_1_rx_timeout              = _SX1276_DIO_1_RX_TIMEOUT;
sx1276_dio_mapping_t const sx1276_dio_1_fhss_change_channel     = _SX1276_DIO_1_FHSS_CHANGE_CHANNEL;
sx1276_dio_mapping_t const sx1276_dio_1_cad_detected            = _SX1276_DIO_1_CAD_DETECTED;

sx1276_dio_mapping_t const sx1276_dio_2_fhss_change_channel     = _SX1276_DIO_2_FHSS_CHANGE_CHANNEL;

sx1276_dio_mapping_t const sx1276_dio_3_cad_done                = _SX1276_DIO_3_CAD_DONE;
sx1276_dio_mapping_t const sx1276_dio_3_valid_header            = _SX1276_DIO_3_VALID_HEADER;
sx1276_dio_mapping_t const sx1276_dio_3_payload_crc_error       = _SX1276_DIO_3_PAYLOAD_CRC_ERROR;

sx1276_dio_mapping_t const sx1276_dio_4_cad_detected            = _SX1276_DIO_4_CAD_DETECTED;
sx1276_dio_mapping_t const sx1276_dio_4_pll_lock                = _SX1276_DIO_4_PLL_LOCK;

sx1276_dio_mapping_t const sx1276_dio_5_mode_ready              = _SX1276_DIO_5_MODE_READY;
sx1276_dio_mapping_t const sx1276_dio_5_clk_out                 = _SX1276_DIO_5_CLK_OUT;


/*
 * Private data
 */
static struct
{
    uint8_t     op_mode_val;    // RegOpMode register value

    // Cached parameters for receiver optimisation
    uint32_t    frf;            // Current frequency register
#ifdef MADE_RECEIVER_OPTIMISATION_WORK
    uint32_t    frf_offset;     // Optimal receive offset
    uint8_t     if_freq_2_val;  // Optimal if_freq_2 register value
                                // Optimal if_freq_1 register value is 0x00
#endif
    uint8_t     det_opt_val;    // Optimal detection_optmisation register value

    // Logic term for control of the RF switch
    bool        pa_boost;

    // Implicit-header mode length (also used as flag)
    uint8_t     expected_payload_length;

    // Current encoding parameters for calculating the symbol period
    frequency_t bw_hz;  // low-rate optimise and frequency error calculations
    sx1276_sf_t sf;
}
_sx1276;


/*
 * Private functions
 */


/**
 * Calculates the current symbol period and applies it to the device
 * @note Depends on SF and BW
 */
void _sx1276_set_low_rate_optimise(void);


/**
 * Calculates the sensitivity optimisation values and applies it to the device
 * @note Depends on FRF and BW
 */
void _sx1276_set_500khz_optimise(void);


/**
 * Sets the FRF and  calls 500kHz optimise
 */
void _sx1276_set_frf(uint32_t const Value);


/**
 * Reads from a register. It would be known as 'read_register', but the
 * function is called this to align with the data-sheet.
 *
 * @param RegID Index of the register
 * @return The value of the register
 */
static uint8_t _sx1276_single_read(_sx1276_reg_t const RegID);


/**
 * Writes to a register, returning its value before it was written. This is an
 * efficient way to read and clear the IRQ flags, for example. Not
 * Writes to a register. It would be known as 'write_register', but the
 * function is called this to align with the data-sheet.
 *
 * @param RegID Index of the register
 * @param Value The value to send
 * @return The value returned as Value was written
 */
static uint8_t _sx1276_single_write(_sx1276_reg_t const RegID,
                                    uint8_t const Value);


/**
 * Reads from register(s). If RegID is set to the FIFO, this efficiently reads
 * Size bytes from it, otherwise the RegID is auto-incremented on every
 * successive read.
 * @note The function guarantees to write Size bytes to pData
 *
 * @param RegID Index of the register
 * @param pData Pointer to the data buffer
 * @param Size  Number of bytes to read
 */
static void _sx1276_burst_read(_sx1276_reg_t const RegID,
                               uint8_t* const pData, size_t const Size);


/**
 * Reads from register(s). If RegID is set to the FIFO, this efficiently writes
 * Size bytes to it, otherwise the RegID is auto-incremented on every
 * successive write.
 *
 * @param RegID Index of the register
 * @param pData Pointer to the data buffer
 * @param Size  Number of bytes to write
 */
static void _sx1276_burst_write(_sx1276_reg_t const RegID,
                                uint8_t* const pData, size_t const Size);


/**
 * Reads two bytes. If RegID is set to the FIFO, this efficiently reads two
 * bytes from it, otherwise it reads from RegID as bits 15:8 then RegID + 1 as
 * bits 7:0
 *
 * @param RegID Index of the register
 * @return The value requested
 */
uint16_t _sx1276_uint16_read(_sx1276_reg_t const RegID);


/**
 * Writes two bytes. If RegID is set to the FIFO, this efficiently writes two
 * bytes to it, otherwise it writes to RegID with bits 15:8 then RegID + 1 with
 * bits 7:0
 *
 * @param RegID Index of the register
 * @param Value The value to write
 */
void _sx1276_uint16_write(_sx1276_reg_t const RegID, uint16_t const Value);


/**
 * Reads three bytes. If RegID is set to the FIFO, this efficiently reads three
 * bytes from it, otherwise it reads from RegID as bits 23:8 then RedID + 1 as
 * bits 15:8 then RegID + 2 as bits 7:0
 *
 * @param RegID Index of the register
 * @return The value requested
 */
uint32_t _sx1276_uint24_read(_sx1276_reg_t const RegID);


/**
 * Writes three bytes. If RegID is set to the FIFO, this efficiently writes
 * three bytes to it, otherwise it writes to RegID with bits 23:16 then
 * RegID + 1with bits 15:8 then RegID + 2 with bits 7:0
 *
 * @param RegID Index of the register
 * @param Value The value to write
 */
void _sx1276_uint24_write(_sx1276_reg_t const RegID, uint32_t const Value);


/*
 * Public functions
 */
void sx1276_init(void)
{
    util_clear(&_sx1276, sizeof(_sx1276));
    _sx1276.op_mode_val     = _SX1276_LONG_RANGE_MODE_SET(0, 1) |
                              _SX1276_MODE_SET(0, sx1276_mode_sleep);

#ifdef MADE_RECEIVER_OPTIMISATION_WORK
    _sx1276.if_freq_2_val   = _SX1276_REG_IF_FREQ_2_DEFAULT;
#endif

    // This default value. Turn off AutomaticIFOn as per the errata note
    _sx1276.det_opt_val     = _SX1276_AUTOMATIC_IF_ON_SET(_SX1276_REG_DETECT_OPTIMIZE_DEFAULT, 0);

    _sx1276.bw_hz           = sx1276_bw_hz(sx1276_bw_125kHz);
    _sx1276.sf              = sx1276_sf_7;

    hal_rf_set_reset(true);
    hal_delay_us(100);

    hal_rf_set_reset(false);    // Release RESET
    hal_delay_ms(10);           // Wait for the chip to become active

    // Select LoRa mode (Can only select LoRa mode during sleep)
    _sx1276_single_write(_sx1276_reg_op_mode, _sx1276.op_mode_val);

    _sx1276_single_write(_sx1276_reg_detect_optimize, _sx1276.det_opt_val);
    
#ifdef MADE_RECEIVER_OPTIMISATION_WORK
    _sx1276_single_write(_sx1276_reg_if_freq_2, _sx1276.if_freq_2_val);
    _sx1276_single_write(_sx1276_reg_if_freq_1, 0x00);
#endif

    _sx1276_set_low_rate_optimise();
    _sx1276_set_500khz_optimise();

    // Set the base-addresses for the Rx and Tx to the bottom of the FIFO to
    // allow the full capacity to be used for sending and receiving
    _sx1276_single_write(_sx1276_reg_fifo_rx_base_addr,
                         _SX1276_FIFO_RX_BASE_ADDR);

    _sx1276_single_write(_sx1276_reg_fifo_tx_base_addr,
                         _SX1276_FIFO_TX_BASE_ADDR);
}


void sx1276_deinit(void)
{
    sx1276_set_mode(sx1276_mode_sleep);
}


uint8_t sx1276_get_version(void)
{
    return _sx1276_single_read(_sx1276_reg_version);
}


void sx1276_set_clock_source_type(sx1276_clock_source_type_t const Source)
{
    uint8_t const reg = (Source == sx1276_clock_source_type_tcxo) ?
                        _SX1276_TCXO_INPUT_ON : 0;

    _sx1276_single_write(_sx1276_reg_tcxo, _SX1276_REG_TCXO_DEFAULT | reg);
}


sx1276_clock_source_type_t sx1276_get_clock_source_type(void)
{
    bool const tcxo_en =
        (_sx1276_single_read(_sx1276_reg_tcxo) & _SX1276_TCXO_INPUT_ON) != 0;

    return tcxo_en ? sx1276_clock_source_type_tcxo :
                     sx1276_clock_source_type_crystal;
}


void sx1276_set_frf(uint32_t const Value)
{
    _sx1276.frf = Value;

    _sx1276_set_frf(Value);
}


uint32_t sx1276_get_frf(void)
{
    return _sx1276.frf;
}


void sx1276_set_frequency(frequency_t const Value)
{
    _sx1276.frf = sx1276_frequency_to_frf(Value);
#ifdef MADE_RECEIVER_OPTIMISATION_WORK
    _sx1276_set_frf(_sx1276.frf + _sx1276.frf_offset);
#else
    _sx1276_set_frf(_sx1276.frf);
#endif
}


frequency_t sx1276_get_frequency(void)
{
    return sx1276_frf_to_frequency(_sx1276.frf);
}


frequency_t sx1276_get_real_frequency(void)
{
#ifdef MADE_RECEIVER_OPTIMISATION_WORK
    return sx1276_frf_to_frequency(_sx1276.frf + _sx1276.frf_offset);
#else
    return sx1276_frf_to_frequency(_sx1276.frf);
#endif
}


frequency_t sx1276_get_dev_frequency(void)
{
    return sx1276_frf_to_frequency(_sx1276_uint24_read(_sx1276_reg_frf_msb));
}


int32_t sx1276_get_frequency_error(void)
{
    uint32_t reg = _sx1276_uint24_read(_sx1276_reg_fei_msb) & 0x000FFFFF;

    // Extend the sign of 20-bit integer to fill the 32-bit signed integer type
    if (reg & 0x00080000) reg |= 0xFFF00000;

    int32_t const fei_value = (int32_t)reg;

    return (((int64_t)fei_value << 24) * (int64_t) _sx1276.bw_hz) /
        ((int64_t)_SX1276_FOSC * 500000);
}


void sx1276_set_ppm_correction(int8_t const Value)
{
    _sx1276_single_write(_sx1276_reg_ppm_correction, (uint8_t)Value);
}


int8_t sx1276_get_ppm_correction(void)
{
    return (int8_t) _sx1276_single_read(_sx1276_reg_ppm_correction);
}


void sx1276_set_lna(sx1276_lna_gain_t const Value, bool const BoostEnabled)
{
    uint8_t lna_val;
    uint8_t cfg_val = _sx1276_single_read(_sx1276_reg_modem_config_3);

    if (Value == sx1276_lna_gain_auto)
    {
        lna_val = _sx1276_single_read(_sx1276_reg_lna);

        cfg_val |= _SX1276_AGC_AUTO_ON;
    }
    else
    {
        lna_val = _SX1276_LNA_GAIN_VALUE(Value);

        cfg_val &= ~_SX1276_AGC_AUTO_ON;
    }

    if ((_sx1276.frf >= _SX1276_FRF_HF_FREQ) && BoostEnabled)
    {
        lna_val |= _SX1276_LNA_BOOST_HF;
    }

    _sx1276_single_write(_sx1276_reg_modem_config_3, cfg_val);
    _sx1276_single_write(_sx1276_reg_lna, lna_val);
}


void sx1276_set_tx_power(int const Pout, sx1276_rf_pin_t const Pin)
{
    uint8_t reg = 0;

    // Required for RF switch logic
    _sx1276.pa_boost = (Pin == sx1276_rf_pin_pa_boost);

    if (_sx1276.pa_boost)
    {
        int output_power = 17 - (15 - UTIL_LIMIT(Pout, 2, 17));

        reg = _SX1276_OUTPUT_POWER_SET(reg, output_power);
        reg = _SX1276_PA_SELECT_SET(reg, 1);
        // Pout = 17 - (15 - Value)   { 2 .. 17dBm }
    }
    else    // sx1276_rf_pin_rfo
    {
        int output_power = 15 - (15 - UTIL_LIMIT(Pout, 0, 14));

        reg = _SX1276_OUTPUT_POWER_SET(reg, output_power);
        reg = _SX1276_MAX_POWER_SET(reg, _sx1276_max_power_15dBm);
        // Pout = MaxPower - (15 - Value) { 0 .. 14dBm }
    }

    _sx1276_single_write(_sx1276_reg_pa_config, reg);
}


void sx1276_get_tx_power(int* const pPout, sx1276_rf_pin_t* const pPin)
{
    uint8_t reg = _sx1276_single_read(_sx1276_reg_pa_config);

    if (_SX1276_PA_SELECT_GET(reg))
    {
        *pPout  = _SX1276_OUTPUT_POWER_GET(reg) + 2;
        *pPin   = sx1276_rf_pin_pa_boost;
    }
    else
    {
        *pPout  = _SX1276_OUTPUT_POWER_GET(reg);
        *pPin   = sx1276_rf_pin_rfo;
    }
}


void sx1276_set_spreading_factor(sx1276_sf_t const Value)
{
    uint8_t const opt_val = (Value == sx1276_sf_6) ? 0x05 : 0x03;
    uint8_t const thr_val = (Value == sx1276_sf_6) ? 0x0C : 0x0A;

    uint8_t cfg2_val = _sx1276_single_read(_sx1276_reg_modem_config_2);
    cfg2_val = _SX1276_SPREADING_FACTOR_SET(cfg2_val, Value);

    _sx1276.det_opt_val =
        _SX1276_DETECTION_OPTIMIZE_SET(_sx1276.det_opt_val, opt_val);

    _sx1276_single_write(_sx1276_reg_detection_threshold, thr_val);
    _sx1276_single_write(_sx1276_reg_modem_config_2, cfg2_val);
    _sx1276_single_write(_sx1276_reg_detect_optimize, _sx1276.det_opt_val);

    _sx1276.sf = Value;
    _sx1276_set_low_rate_optimise();
}


sx1276_sf_t sx1276_get_spreading_factor(void)
{
    return (sx1276_sf_t) _SX1276_SPREADING_FACTOR_GET(
        _sx1276_single_read(_sx1276_reg_modem_config_2));
}


void sx1276_set_bandwidth(sx1276_bw_t const Value)
{
    // Calculate the values and settings to be made on entering receive mode
    bool auto_if_on = false;

    switch (Value)
    {
    default:
        break;
#ifdef MADE_RECEIVER_OPTIMISATION_WORK
    case sx1276_bw_7_8kHz:
        _sx1276.if_freq_2_val   = 0x48;
        _sx1276.frf_offset      = 128;  // Rx +7.81kHz * FOSC >> 19
        break;

    case sx1276_bw_10_4kHz:
        _sx1276.if_freq_2_val   = 0x44;
        _sx1276.frf_offset      = 171;  // Rx +10.42kHz * FOSC >> 19
        break;

    case sx1276_bw_15_6kHz:
        _sx1276.if_freq_2_val   = 0x44;
        _sx1276.frf_offset      = 256;  // Rx +15.62kHz * FOSC >> 19
        break;

    case sx1276_bw_20_8kHz:
        _sx1276.if_freq_2_val   = 0x44;
        _sx1276.frf_offset      = 341;  // Rx +20.83kHz * FOSC >> 19
        break;

    case sx1276_bw_31_25kHz:
        _sx1276.if_freq_2_val   = 0x44;
        _sx1276.frf_offset      = 512;  // Rx +31.25kHz * FOSC >> 19
        break;

    case sx1276_bw_41_7kHz:
        _sx1276.if_freq_2_val   = 0x44;
        _sx1276.frf_offset      = 683;  // Rx +41.67kHz * FOSC >> 19
        break;

    case sx1276_bw_62_5kHz:
        _sx1276.if_freq_2_val   = 0x40;
        _sx1276.frf_offset      = 0;
        break;

    case sx1276_bw_125kHz:
        _sx1276.if_freq_2_val   = 0x40;
        _sx1276.frf_offset      = 0;
        break;

    case sx1276_bw_250kHz:
        _sx1276.if_freq_2_val   = 0x40;
        _sx1276.frf_offset      = 0;
        break;
#endif

    case sx1276_bw_500kHz:
        auto_if_on              = true;
#ifdef MADE_RECEIVER_OPTIMISATION_WORK
        _sx1276.if_freq_2_val   = 0;
        _sx1276.frf_offset      = 0;
#endif
        break;
    }


    _sx1276.det_opt_val = _SX1276_AUTOMATIC_IF_ON_SET(_sx1276.det_opt_val,
                                                      auto_if_on ? 1 : 0);

    _sx1276_single_write(_sx1276_reg_detect_optimize, _sx1276.det_opt_val);

#ifdef MADE_RECEIVER_OPTIMISATION_WORK
    if (_sx1276.frf > 0)    // Set frequency has already been called
    {
        _sx1276_set_frf(_sx1276.frf + _sx1276.frf_offset);
    }

    if (_SX1276_AUTOMATIC_IF_ON_GET(_sx1276.det_opt_val) == 0)
    {
        // Apply the manual settings
        _sx1276_single_write(_sx1276_reg_if_freq_2, _sx1276.if_freq_2_val);
        _sx1276_single_write(_sx1276_reg_if_freq_1, 0x00);
    }
#endif

    {
        uint8_t cfg_val = _sx1276_single_read(_sx1276_reg_modem_config_1);
        cfg_val = _SX1276_BW_SET(cfg_val, Value);

        _sx1276_single_write(_sx1276_reg_modem_config_1, cfg_val);
    }

    _sx1276.bw_hz = sx1276_bw_hz(Value);
    _sx1276_set_low_rate_optimise();
}


sx1276_bw_t sx1276_get_bandwidth(void)
{
    return (sx1276_bw_t) _SX1276_BW_GET(
        _sx1276_single_read(_sx1276_reg_modem_config_1));
}


void sx1276_set_coding_rate(sx1276_cr_t const Value)
{
    uint8_t reg_val = _sx1276_single_read(_sx1276_reg_modem_config_1);
    reg_val = _SX1276_CODING_RATE_SET(reg_val, Value);
    _sx1276_single_write(_sx1276_reg_modem_config_1, reg_val);
}


sx1276_cr_t sx1276_get_coding_rate(void)
{
    return (sx1276_cr_t) _SX1276_CODING_RATE_GET(
        _sx1276_single_read(_sx1276_reg_modem_config_1));
}


void sx1276_set_preamble_length(uint16_t const Value)
{
    _sx1276_uint16_write(_sx1276_reg_preamble_msb, Value);
}


uint16_t sx1276_get_preamble_length(void)
{
    return _sx1276_uint16_read(_sx1276_reg_preamble_msb);
}


void sx1276_set_symbol_timeout(uint16_t const Value)
{
    uint16_t const sto = (Value < _SX1276_SYMB_TIMEOUT_MAX) ?
                            Value : _SX1276_SYMB_TIMEOUT_MAX;

    uint8_t const cfg_val = _sx1276_single_read(_sx1276_reg_modem_config_2);

    uint8_t p_reg[2] =
    {
        _SX1276_SYMB_TIMEOUT_9_8_SET(cfg_val, (sto >> 8) & 3),
        Value,
    };

    _sx1276_burst_write(_sx1276_reg_modem_config_2, p_reg, sizeof(p_reg));
}


uint16_t sx1276_get_symbol_timeout(void)
{
    return _sx1276_uint16_read(_sx1276_reg_modem_config_2) & _SX1276_MASK(9, 0);
}


void sx1276_set_sync_word(uint8_t const Value)
{
    _sx1276_single_write(_sx1276_reg_sync_word, Value);
}


uint8_t sx1276_get_sync_word(void)
{
    return _sx1276_single_read(_sx1276_reg_sync_word);
}


void sx1276_set_header_mode(uint8_t const PacketSize)
{
    _sx1276.expected_payload_length = PacketSize;

    uint8_t reg = _sx1276_single_read(_sx1276_reg_modem_config_1);

    if (sx1276_is_header_mode_implicit())
    {
        reg |= _SX1276_IMPLICIT_HEADER_MODE_ON;
    }
    else
    {
        reg &= ~_SX1276_IMPLICIT_HEADER_MODE_ON;
    }

    _sx1276_single_write(_sx1276_reg_modem_config_1, reg);
}


uint8_t sx1276_get_header_mode(void)
{
    bool const implicit_on =
        (_sx1276_single_read(_sx1276_reg_modem_config_1) &
        _SX1276_IMPLICIT_HEADER_MODE_ON) != 0;

    return implicit_on ? _sx1276.expected_payload_length : 0;
}


bool sx1276_is_header_mode_implicit(void)
{
    return _sx1276.expected_payload_length > 0;
}


void sx1276_set_rx_payload_crc_enabled(bool const Enabled)
{
    uint8_t reg = _sx1276_single_read(_sx1276_reg_modem_config_2);

    if (Enabled)
    {
        reg |= _SX1276_RX_PAYLOAD_CRC_ON;
    }
    else
    {
        reg &= ~_SX1276_RX_PAYLOAD_CRC_ON;
    }

    _sx1276_single_write(_sx1276_reg_modem_config_2, reg);
}


bool sx1276_get_rx_payload_crc_enabled(void)
{
    return (_sx1276_single_read(_sx1276_reg_modem_config_2) &
            _SX1276_RX_PAYLOAD_CRC_ON) != 0;
}


size_t sx1276_send_packet(void* const pData, size_t const Size)
{
    // Set FIFO address to Tx base address
    _sx1276_single_write(_sx1276_reg_fifo_addr_ptr, _SX1276_FIFO_TX_BASE_ADDR);

    // Limit Size so the packet length does not exceed the maximum
    size_t const num_write = (Size < LORA_MAX_PACKET_SIZE) ?
                                Size : LORA_MAX_PACKET_SIZE;

    // Write data
    _sx1276_burst_write(_sx1276_reg_fifo, pData, num_write);

    // Update length
    _sx1276_single_write(_sx1276_reg_payload_length, (uint8_t)num_write);

    // Start transmitting
    // NOTE: Using public function to ensure correct steps are taken
    sx1276_set_mode(sx1276_mode_tx);

    return num_write;
}


size_t sx1276_read_packet(void* const pData, size_t const Size)
{
    size_t const packet_len = sx1276_is_header_mode_implicit() ?
        (size_t) _sx1276.expected_payload_length : 
        _sx1276_single_read(_sx1276_reg_rx_nb_bytes);


    // Reset FIFO address to current Rx address, which is the same as the base
    // address
    _sx1276_single_write(_sx1276_reg_fifo_addr_ptr,
                         _SX1276_FIFO_RX_BASE_ADDR);

    // Retrieve the data!
    size_t const num_read = (Size < packet_len) ? Size : packet_len;

    _sx1276_burst_read(_sx1276_reg_fifo, pData, num_read);

    return num_read;    // Just make Size >= max packet length!
}


sx1276_rssi_t sx1276_get_packet_rssi(void)
{
    sx1276_rssi_t reg = _sx1276_single_read(_sx1276_reg_pkt_rssi_value);

    return reg + ((_sx1276.frf < _SX1276_FRF_HF_FREQ) ?
        _SX1276_RSSI_LF_OFFSET : _SX1276_RSSI_HF_OFFSET);
}


uint8_t sx1276_get_wideband_rssi(void)
{
    return _sx1276_single_read(_sx1276_reg_rssi_wideband);
}


sx1276_snr_t sx1276_get_packet_snr(void)
{
    return (sx1276_snr_t)_sx1276_single_read(_sx1276_reg_pkt_snr_value);
}


uint16_t sx1276_get_valid_header_count(void)
{
   return _sx1276_uint16_read(_sx1276_reg_rx_header_cnt_value_msb);
}


uint16_t sx1276_get_valid_packet_count(void)
{
    return _sx1276_uint16_read(_sx1276_reg_rx_packet_cnt_value_msb);
}


void sx1276_set_dio_mapping(sx1276_dio_mapping_t const DIOMapping)
{
    sx1276_dio_t const dio  = (sx1276_dio_t) (DIOMapping >> 4) & 0x0F;
    uint8_t const value     = (DIOMapping >> 0) & 0x0F;

    uint8_t p_reg[2] = { 0 };
    _sx1276_burst_read(_sx1276_reg_dio_mapping_1, p_reg, sizeof(p_reg));

    switch (dio)
    {
    case sx1276_dio_0:
        p_reg[0] = _SX1276_DIO0_MAPPING_SET(p_reg[0], value);
        break;

    case sx1276_dio_1:
        p_reg[0] = _SX1276_DIO1_MAPPING_SET(p_reg[0], value);
        break;

    case sx1276_dio_2:
        p_reg[0] = _SX1276_DIO2_MAPPING_SET(p_reg[0], value);
        break;

    case sx1276_dio_3:
        p_reg[0] = _SX1276_DIO3_MAPPING_SET(p_reg[0], value);
        break;

    case sx1276_dio_4:
        p_reg[1] = _SX1276_DIO4_MAPPING_SET(p_reg[1], value);
        break;

    case sx1276_dio_5:
        p_reg[1] = _SX1276_DIO5_MAPPING_SET(p_reg[1], value);
        break;
    }

    _sx1276_burst_write(_sx1276_reg_dio_mapping_1, p_reg, sizeof(p_reg));
}


sx1276_dio_mapping_t sx1276_get_dio_mapping(sx1276_dio_t const DIOPin)
{
    uint8_t p_reg[2] = { 0 };
    _sx1276_burst_read(_sx1276_reg_dio_mapping_1, p_reg, sizeof(p_reg));

    uint8_t value = -1;

    switch (DIOPin)
    {
    case sx1276_dio_0:
        value = _SX1276_DIO0_MAPPING_GET(p_reg[0]);
        break;

    case sx1276_dio_1:
        value = _SX1276_DIO1_MAPPING_GET(p_reg[0]);
        break;

    case sx1276_dio_2:
        value = _SX1276_DIO2_MAPPING_GET(p_reg[0]);
        break;

    case sx1276_dio_3:
        value = _SX1276_DIO3_MAPPING_GET(p_reg[0]);
        break;

    case sx1276_dio_4:
        value = _SX1276_DIO4_MAPPING_GET(p_reg[1]);
        break;

    case sx1276_dio_5:
        value = _SX1276_DIO5_MAPPING_GET(p_reg[1]);
        break;
    }

    return (sx1276_dio_mapping_t) (((uint8_t)DIOPin) << 4) | value;
}


void sx1276_set_irq_enable(sx1276_irq_t const IRQs)
{
    _sx1276_single_write(_sx1276_reg_irq_flags_mask, ~((uint8_t) IRQs));
}


sx1276_irq_t sx1276_get_irq_enable(void)
{
    return (sx1276_irq_t) ~_sx1276_single_read(_sx1276_reg_irq_flags_mask);
}


sx1276_irq_t sx1276_get_irq_flags(void)
{
    return (sx1276_irq_t) _sx1276_single_read(_sx1276_reg_irq_flags);
}


sx1276_irq_t sx1276_get_clear_irq_flags(void)
{
    return (sx1276_irq_t) _sx1276_single_write(_sx1276_reg_irq_flags,
                                               _SX1276_IRQ_ALL);
}


void sx1276_set_mode(sx1276_mode_t const Mode)
{
    bool const mode_rx      = (Mode == sx1276_mode_rx_continuous) ||
                              (Mode == sx1276_mode_rx_single);

    bool const mode_rx_cad  = mode_rx || (Mode == sx1276_mode_cad);

    bool const mode_tx      = (Mode == sx1276_mode_tx);

    // Always set this
    hal_rf_set_switch(mode_rx_cad || (mode_tx && !_sx1276.pa_boost));

    uint8_t irqs_to_clear;
    
    if (mode_rx)
    {
        // Set FIFO back to start
        _sx1276_single_write(_sx1276_reg_fifo_addr_ptr,
                             _SX1276_FIFO_RX_BASE_ADDR);

        if (sx1276_is_header_mode_implicit())
        {
            // Packet doesn't contain the length; so it needs programming
            _sx1276_single_write(_sx1276_reg_payload_length,
                                 _sx1276.expected_payload_length);
        }

        // Clear Rx IRQs
        irqs_to_clear = _SX1276_RX_DONE                |
                        _SX1276_RX_TIMEOUT             |
                        _SX1276_PAYLOAD_CRC_ERROR_MASK |
                        _SX1276_VALID_HEADER;
    }
    else if (mode_tx)
    {
        // Clear Tx IRQ
        irqs_to_clear = _SX1276_TX_DONE;
    }
    else if (Mode == sx1276_mode_cad)
    {
        // Clear CAD Mode IRQs
        irqs_to_clear = _SX1276_CAD_DETECTED | _SX1276_CAD_DONE;
    }
    else
    {
        irqs_to_clear = 0;
    }

    if (irqs_to_clear != 0)
    {
        _sx1276_single_write(_sx1276_reg_irq_flags, irqs_to_clear);
    }

    _sx1276.op_mode_val = _SX1276_MODE_SET(_sx1276.op_mode_val, Mode);
    _sx1276_single_write(_sx1276_reg_op_mode, _sx1276.op_mode_val);
}


sx1276_mode_t sx1276_get_mode(void)
{
    return (sx1276_mode_t) _SX1276_MODE_GET(
        _sx1276_single_read(_sx1276_reg_op_mode));
}


void sx1276_set_timer1(sx1276_timer_resolution_t const Resolution,
                       uint8_t const Coefficient)
{
    // Switch register access mode to FSK
    uint8_t op_mode_val = _SX1276_ACCESS_SHARED_REG_SET(_sx1276_reg_op_mode, 1);
    _sx1276_single_write(_sx1276_reg_op_mode, op_mode_val);

    uint8_t res_val = _sx1276_single_read(_sx1276_fsk_reg_timer_resol);
    res_val = _SX1276_TIMER1_RESOLUTION_SET(res_val, Resolution);
    _sx1276_single_write(_sx1276_fsk_reg_timer_resol, res_val);

    _sx1276_single_write(_sx1276_fsk_reg_timer_1_coefficient, Coefficient);

    // Restore mode
    _sx1276_single_write(_sx1276_reg_op_mode, _sx1276.op_mode_val);
}


void sx1276_set_timer2(sx1276_timer_resolution_t const Resolution,
                       uint8_t const Coefficient)
{
    // Switch register access mode to FSK
    uint8_t op_mode_val = _SX1276_ACCESS_SHARED_REG_SET(_sx1276_reg_op_mode, 1);
    _sx1276_single_write(_sx1276_reg_op_mode, op_mode_val);

    uint8_t res_val = _sx1276_single_read(_sx1276_fsk_reg_timer_resol);
    res_val = _SX1276_TIMER2_RESOLUTION_SET(res_val, Resolution);
    _sx1276_single_write(_sx1276_fsk_reg_timer_resol, res_val);

    _sx1276_single_write(_sx1276_fsk_reg_timer_2_coefficient, Coefficient);

    // Restore mode
    _sx1276_single_write(_sx1276_reg_op_mode, _sx1276.op_mode_val);
}


const char* sx1276_clock_source_type_str(sx1276_clock_source_type_t const Value)
{
    const char* p_str;

    switch (Value)
    {
    case sx1276_clock_source_type_crystal:
        p_str = "sx1276_clock_source_type_crystal";
        break;

    case sx1276_clock_source_type_tcxo:
        p_str = "sx1276_clock_source_type_tcxo";
        break;

    default:
        p_str = "sx1276_clock_source_type <unknown>";
        break;
    }

    return p_str;
}


const char* sx1276_sf_str(sx1276_sf_t const Value)
{
    const char* p_str;

    switch (Value)
    {
    case sx1276_sf_6:           p_str = "sx1276_sf_6";                  break;
    case sx1276_sf_7:           p_str = "sx1276_sf_7";                  break;
    case sx1276_sf_8:           p_str = "sx1276_sf_8";                  break;
    case sx1276_sf_9:           p_str = "sx1276_sf_9";                  break;
    case sx1276_sf_10:          p_str = "sx1276_sf_10";                 break;
    case sx1276_sf_11:          p_str = "sx1276_sf_11";                 break;
    case sx1276_sf_12:          p_str = "sx1276_sf_12";                 break;
    default:                    p_str = "sx1276_sf <unknown>";          break;
    }

    return p_str;
}


const char* sx1276_bw_str(sx1276_bw_t const Value)
{
    const char* p_str;

    switch (Value)
    {
    case sx1276_bw_7_8kHz:      p_str = "sx1276_bw_7_8kHz";             break;
    case sx1276_bw_10_4kHz:     p_str = "sx1276_bw_10_4kHz";            break;
    case sx1276_bw_15_6kHz:     p_str = "sx1276_bw_15_6kHz";            break;
    case sx1276_bw_20_8kHz:     p_str = "sx1276_bw_20_8kHz";            break;
    case sx1276_bw_31_25kHz:    p_str = "sx1276_bw_31_25kHz";           break;
    case sx1276_bw_41_7kHz:     p_str = "sx1276_bw_41_7kHz";            break;
    case sx1276_bw_62_5kHz:     p_str = "sx1276_bw_62_5kHz";            break;
    case sx1276_bw_125kHz:      p_str = "sx1276_bw_125kHz";             break;
    case sx1276_bw_250kHz:      p_str = "sx1276_bw_250kHz";             break;
    case sx1276_bw_500kHz:      p_str = "sx1276_bw_500kHz";             break;
    default:                    p_str = "sx1276_bw <unknown>";          break;
    }

    return p_str;
}


const char* sx1276_cr_str(sx1276_cr_t const Value)
{
    const char* p_str;

    switch (Value)
    {
    case sx1276_cr_4_5:         p_str = "sx1276_cr_4_5";                break;
    case sx1276_cr_4_6:         p_str = "sx1276_cr_4_6";                break;
    case sx1276_cr_4_7:         p_str = "sx1276_cr_4_7";                break;
    case sx1276_cr_4_8:         p_str = "sx1276_cr_4_8";                break;
    default:                    p_str = "sx1276_cr <unknown>";          break;
    }

    return p_str;
}


const char* sx1276_lna_gain_str(sx1276_lna_gain_t const Value)
{
    const char* p_str;

    switch (Value)
    {
    case sx1276_lna_gain_auto:  p_str = "sx1276_lna_gain_auto";         break;
    case sx1276_lna_gain_G1:    p_str = "sx1276_lna_gain_G1";           break;
    case sx1276_lna_gain_G2:    p_str = "sx1276_lna_gain_G2";           break;
    case sx1276_lna_gain_G3:    p_str = "sx1276_lna_gain_G3";           break;
    case sx1276_lna_gain_G4:    p_str = "sx1276_lna_gain_G4";           break;
    case sx1276_lna_gain_G5:    p_str = "sx1276_lna_gain_G5";           break;
    case sx1276_lna_gain_G6:    p_str = "sx1276_lna_gain_G6";           break;
    default:                    p_str = "sx1276_lna_gain <unknown>";    break;
    }

    return p_str;
}


const char* sx1276_rf_pin_str(sx1276_rf_pin_t const Value)
{
    const char* p_str;

    switch (Value)
    {
    case sx1276_rf_pin_rfo:     p_str = "sx1276_rf_pin_rfo";            break;
    case sx1276_rf_pin_pa_boost:p_str = "sx1276_rf_pin_pa_boost";       break;
    default:                    p_str = "sx1276_rf_pin <unknown>";      break;
    }

    return p_str;
}


char* sx1276_snr_str(sx1276_snr_t const Value)
{
    static char p_buf[8];  // 8 chars max (including terminator)
    snprintf(p_buf, sizeof(p_buf), "%d.%u", Value >> 2, 25 * (Value & 3));
    return p_buf;
}


const char* sx1276_dio_str(sx1276_dio_t const Value)
{
    const char* p_str;

    switch (Value)
    {
    case sx1276_dio_0:          p_str = "sx1276_dio_0";                 break;
    case sx1276_dio_1:          p_str = "sx1276_dio_1";                 break;
    case sx1276_dio_2:          p_str = "sx1276_dio_2";                 break;
    case sx1276_dio_3:          p_str = "sx1276_dio_3";                 break;
    case sx1276_dio_4:          p_str = "sx1276_dio_4";                 break;
    case sx1276_dio_5:          p_str = "sx1276_dio_5";                 break;
    default:                    p_str = "sx1276_dio <unknown>";         break;
    }

    return p_str;
}


const char* sx1276_dio_mapping_str(sx1276_dio_mapping_t const Value)
{
    const char* p_str;

    switch (Value)
    {
    case _SX1276_DIO_0_RX_DONE:
        p_str = "sx1276_dio_0_rx_done";
        break;

    case _SX1276_DIO_0_TX_DONE:
        p_str = "sx1276_dio_0_tx_done";
        break;

    case _SX1276_DIO_0_CAD_DONE:
        p_str = "sx1276_dio_0_cad_done";
        break;

    case _SX1276_DIO_1_RX_TIMEOUT:
        p_str = "sx1276_dio_1_rx_timeout";
        break;

    case _SX1276_DIO_1_FHSS_CHANGE_CHANNEL:
        p_str = "sx1276_dio_1_fhss_change_channel";
        break;

    case _SX1276_DIO_1_CAD_DETECTED:
        p_str = "sx1276_dio_1_cad_detected";
        break;

    case _SX1276_DIO_2_FHSS_CHANGE_CHANNEL:
        p_str = "sx1276_dio_2_fhss_change_channel";
        break;

    case _SX1276_DIO_3_CAD_DONE:
        p_str = "sx1276_dio_3_cad_done";
        break;

    case _SX1276_DIO_3_VALID_HEADER:
        p_str = "sx1276_dio_3_valid_header";
        break;

    case _SX1276_DIO_3_PAYLOAD_CRC_ERROR:
        p_str = "sx1276_dio_3_payload_crc_error";
        break;

    case _SX1276_DIO_4_CAD_DETECTED:
        p_str = "sx1276_dio_4_cad_detected";
        break;

    case _SX1276_DIO_4_PLL_LOCK:
        p_str = "sx1276_dio_4_pll_lock";
        break;

    case _SX1276_DIO_5_MODE_READY:
        p_str = "sx1276_dio_5_mode_ready";
        break;

    case _SX1276_DIO_5_CLK_OUT:
        p_str = "sx1276_dio_5_clk_out";
        break;

    default:
        p_str = "sx1276_dio_mapping <unknown>";
        break;
    }

    return p_str;
}


char* sx1276_irq_str(sx1276_irq_t const Value)
{
    static char p_buf[128]; // 109 chars max (including terminator)
    char* p = p_buf;

    p = util_strcpy(p, "sx1276_irq(");

    if (Value & sx1276_irq_cad_detected)
    {
        p = util_strcpy(p, "sx1276_irq_cad_changed, ");
    }

    if (Value & sx1276_irq_fhss_change_channel)
    {
        p = util_strcpy(p, "sx1276_irq_fhss_change_channel, ");
    }

    if (Value & sx1276_irq_cad_done)
    {
        p = util_strcpy(p, "sx1276_irq_cad_done, ");
    }

    if (Value & sx1276_irq_tx_done)
    {
        p = util_strcpy(p, "sx1276_irq_tx_done, ");
    }

    if (Value & sx1276_irq_valid_header)
    {
        p = util_strcpy(p, "sx1276_irq_valid_header, ");
    }

    if (Value & sx1276_irq_payload_crc_error)
    {
        p = util_strcpy(p, "sx1276_irq_payload_crc_error, ");
    }

    if (Value & sx1276_irq_rx_done)
    {
        p = util_strcpy(p, "sx1276_irq_rx_done, ");
    }

    if (Value & sx1276_irq_rx_timeout)
    {
        p = util_strcpy(p, "sx1276_irq_rx_timeout");
    }

    if (*(p - 1) == ' ')
    {
        // remove last ", "
        p -= 2;
    }

    (void) util_strcpy(p, ")");

    return p_buf;
}


const char* sx1276_mode_str(sx1276_mode_t const Value)
{
    const char* p_str;

    switch (Value)
    {
    case sx1276_mode_sleep:         p_str = "sx1276_mode_sleep";         break;
    case sx1276_mode_stdby:         p_str = "sx1276_mode_stdby";         break;
    case sx1276_mode_fstx:          p_str = "sx1276_mode_fstx";          break;
    case sx1276_mode_tx:            p_str = "sx1276_mode_tx";            break;
    case sx1276_mode_fsrx:          p_str = "sx1276_mode_fsrx";          break;
    case sx1276_mode_rx_continuous: p_str = "sx1276_mode_rx_continuous"; break;
    case sx1276_mode_rx_single:     p_str = "sx1276_mode_rx_single";     break;
    case sx1276_mode_cad:           p_str = "sx1276_mode_cad";           break;
    default:                        p_str = "sx1276_mode <unknown>";     break;
    }

    return p_str;
}


uint32_t sx1276_frequency_to_frf(frequency_t const Value)
{
    return (((uint64_t)Value << 19) + ((_SX1276_FOSC - 1) / 2)) /
        _SX1276_FOSC; // Apply rounding
}


frequency_t sx1276_frf_to_frequency(uint32_t const Value)
{
    return (((uint64_t)Value) * _SX1276_FOSC) >> 19;
}


frequency_t sx1276_bw_hz(sx1276_bw_t const Value)
{
    frequency_t value;

    switch (Value)
    {
    case sx1276_bw_7_8kHz:          value = 7800;                        break;
    case sx1276_bw_10_4kHz:         value = 10400;                       break;
    case sx1276_bw_15_6kHz:         value = 15600;                       break;
    case sx1276_bw_20_8kHz:         value = 20800;                       break;
    case sx1276_bw_31_25kHz:        value = 31250;                       break;
    case sx1276_bw_41_7kHz:         value = 41700;                       break;
    case sx1276_bw_62_5kHz:         value = 62500;                       break;
    case sx1276_bw_125kHz:          value = 125000;                      break;
    case sx1276_bw_250kHz:          value = 250000;                      break;
    case sx1276_bw_500kHz:          value = 500000;                      break;
    default:                        value = 0;                           break;
    }

    return value;
}


bool sx1276_test_fifo(bool const DoPrint)
{
    if (DoPrint)
    {
        printf("sx1276_test_fifo:" UTIL_EOL);

        printf("_SX1276_REG_FIFO_RX_BASE_ADDR = %d" UTIL_EOL,
                _sx1276_single_read(_sx1276_reg_fifo_rx_base_addr));

        printf("_SX1276_REG_FIFO_TX_BASE_ADDR = %d" UTIL_EOL,
                _sx1276_single_read(_sx1276_reg_fifo_tx_base_addr));
    }

    uint8_t p_txbuf[_SX1276_FIFO_SIZE];
    for (size_t i = 0; i < _SX1276_FIFO_SIZE; i++)
    {
        p_txbuf[i] = 255 - i;
    }

    _sx1276_single_write(_sx1276_reg_fifo_addr_ptr, 0);
    _sx1276_burst_write(_sx1276_reg_fifo, p_txbuf, sizeof(p_txbuf));

    uint8_t p_rxbuf[_SX1276_FIFO_SIZE] = { 0 };

    _sx1276_single_write(_sx1276_reg_fifo_addr_ptr, 0);
    _sx1276_burst_read(_sx1276_reg_fifo, p_rxbuf, sizeof(p_rxbuf));

    if (DoPrint)
    {
        printf("tx buffer =");
        util_print_uint8_hex(p_txbuf, sizeof(p_txbuf));
        printf(UTIL_EOL);

        printf("rx buffer =");
        util_print_uint8_hex(p_rxbuf, sizeof(p_rxbuf));
        printf(UTIL_EOL);
    }

    bool ok = true;
    for (size_t i = 0; i < _SX1276_FIFO_SIZE; i++)
    {
        if (p_rxbuf[i] != p_txbuf[i])
        {
            ok = false;
            break;
        }
    }

    if (DoPrint)
    {
        printf("Result = %s" UTIL_EOL, ok ? "PASSED" : "FAILED");
    }

    return ok;
}


/*
 * Private functions
 */
void _sx1276_set_low_rate_optimise(void)
{
    uint32_t sf_len = ((uint32_t)1) << (int) _sx1276.sf;

    uint32_t symbol_period = (1000 * sf_len) / (uint32_t)_sx1276.bw_hz;

    int low_rate = (symbol_period > _SX1276_LOW_DATA_SYMBOL_PERIOD_MS) ? 1 : 0;

    uint8_t cfg_val = _sx1276_single_read(_sx1276_reg_modem_config_3);
    cfg_val = _SX1276_LOW_DATA_RATE_OPTIMIZE_SET(cfg_val, low_rate);
    _sx1276_single_write(_sx1276_reg_modem_config_3, cfg_val);
}


void _sx1276_set_500khz_optimise(void)
{
    uint8_t const opt1_val = (_sx1276.bw_hz < 500000) ? 0x02 : 0x03;

    _sx1276_single_write(_sx1276_reg_high_bw_optimize_1, opt1_val);

    if (_sx1276.bw_hz == 500000)    // kHz
    {
        uint8_t opt2_val;

        if (_sx1276.frf < _SX1276_FRF_HF_FREQ)
        {
            opt2_val = 0x64;
        }
        else
        {
            opt2_val = 0x7F;
        }

        // Only set bw_optmize_2 if 500kHz otherwise the chip selects the value
        _sx1276_single_write(_sx1276_reg_high_bw_optimize_2, opt2_val);
    }
}


void _sx1276_set_frf(uint32_t const Value)
{
    _sx1276_uint24_write(_sx1276_reg_frf_msb, Value);

    int low_freq_mode = (Value < _SX1276_FRF_HF_FREQ) ? 1 : 0;
    _sx1276.op_mode_val = _SX1276_LOW_FREQUENCY_MODE_ON_SET(_sx1276.op_mode_val,
                                                            low_freq_mode);
    _sx1276_single_write(_sx1276_reg_op_mode, _sx1276.op_mode_val);

    _sx1276_set_500khz_optimise();
}


static uint8_t _sx1276_single_read(_sx1276_reg_t const RegID)
{
    hal_rf_spi_start();

    (void) hal_rf_spi_transfer(_SX1276_REG_READ(RegID));
    uint8_t const val = hal_rf_spi_transfer(_SX1276_SPI_SPACE);

    hal_rf_spi_end();

    return val;
}


static uint8_t _sx1276_single_write(_sx1276_reg_t const RegID,
                                    uint8_t const Value)
{
    hal_rf_spi_start();

    (void) hal_rf_spi_transfer(_SX1276_REG_WRITE(RegID));
    uint8_t const val = hal_rf_spi_transfer(Value);

    hal_rf_spi_end();

    return val;
}


static void _sx1276_burst_read(_sx1276_reg_t const RegID,
                               uint8_t* const pData, size_t const Size)
{
    hal_rf_spi_start();

    (void) hal_rf_spi_transfer(_SX1276_REG_READ(RegID));

    for (size_t i = 0; i < Size; i++)
    {
        pData[i] = hal_rf_spi_transfer(_SX1276_SPI_SPACE);
    }

    hal_rf_spi_end();
}


static void _sx1276_burst_write(_sx1276_reg_t const RegID,
                                uint8_t* const pData, size_t const Size)
{
    hal_rf_spi_start();

    (void) hal_rf_spi_transfer(_SX1276_REG_WRITE(RegID));

    for (size_t i = 0; i < Size; i++)
    {
        (void) hal_rf_spi_transfer(pData[i]);
    }

    hal_rf_spi_end();
}


uint16_t _sx1276_uint16_read(_sx1276_reg_t const RegID)
{
    uint8_t p_reg[2] = { 0 };

    _sx1276_burst_read(RegID, p_reg, sizeof(p_reg));

    return  (((uint32_t) p_reg[0]) << 8)  |
            (((uint32_t) p_reg[1]) << 0);
}


void _sx1276_uint16_write(_sx1276_reg_t const RegID, uint16_t const Value)
{
    uint8_t p_reg[2] = { Value >> 8, Value >> 0 };

    _sx1276_burst_write(RegID, p_reg, sizeof(p_reg));
}


uint32_t _sx1276_uint24_read(_sx1276_reg_t const RegID)
{
    uint8_t p_reg[3] = { 0 };

    _sx1276_burst_read(RegID, p_reg, sizeof(p_reg));

    return  (((uint32_t) p_reg[0]) << 16) |
            (((uint32_t) p_reg[1]) << 8)  |
            (((uint32_t) p_reg[2]) << 0);
}


void _sx1276_uint24_write(_sx1276_reg_t const RegID, uint32_t const Value)
{
    uint8_t p_reg[3] = { Value >> 16, Value >> 8, Value >> 0 };

    _sx1276_burst_write(RegID, p_reg, sizeof(p_reg));
}
