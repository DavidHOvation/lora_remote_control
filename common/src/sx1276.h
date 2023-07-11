/**
 * Semtech SX1276 LoRa Transceiver Driver
 * Static class!
 *
 * @author  David Hughes
 * @date    30Jun23
 */
#pragma once

#include <stdint.h>     // uint8_t, uint16_t
#include <stdbool.h>    // bool
#include <stddef.h>     // size_t

#include "frequency.h"


/*
 * Public constants and types
 */
#define SX1276_FIFO_SIZE    (256)   // bytes


extern uint8_t const sx1276_version_expected;


/**
 * Use with sx1276_set_header_mode. If this the packet length is this, the radio
 * automatically adds a header to the packets
 */
extern uint8_t const sx1276_header_mode_explicit;


extern frequency_t const sx1276_frequency_delta; // Frequency change resolution


typedef enum
{
    sx1276_clock_source_type_crystal    = 0,
    sx1276_clock_source_type_tcxo       = 1, // clipped_sine
}
sx1276_clock_source_type_t;


typedef enum
{
    sx1276_sf_6             = 6,    // 64 chips / symbol
    sx1276_sf_7             = 7,    // 126 chips / symbol
    sx1276_sf_8             = 8,    // 256 chips / symbol
    sx1276_sf_9             = 9,    // 512 chips / symbol
    sx1276_sf_10            = 10,   // 1024 chips / symbol
    sx1276_sf_11            = 11,   // 2048 chips / symbol
    sx1276_sf_12            = 12,   // 4096 chips / symbol
}
sx1276_sf_t;


typedef enum
{
    sx1276_bw_7_8kHz        = 0,    // 7.81kHz
    sx1276_bw_10_4kHz       = 1,    // 10.42kHz
    sx1276_bw_15_6kHz       = 2,    // 15.62kHz
    sx1276_bw_20_8kHz       = 3,    // 20.83kHz
    sx1276_bw_31_25kHz      = 4,    // 31.25kHz
    sx1276_bw_41_7kHz       = 5,    // 41.67kHz
    sx1276_bw_62_5kHz       = 6,    // 62.5kHz
    sx1276_bw_125kHz        = 7,    // 125kHz
    sx1276_bw_250kHz        = 8,    // 250kHz
    sx1276_bw_500kHz        = 9,    // 500kHz
}
sx1276_bw_t;


typedef enum
{
    sx1276_cr_4_5           = 0,
    sx1276_cr_4_6           = 1,
    sx1276_cr_4_7           = 2,
    sx1276_cr_4_8           = 3,
}
sx1276_cr_t;


typedef enum
{
    sx1276_lna_gain_auto    = 0,
    sx1276_lna_gain_G1      = 1,    // Maximum Gain
    sx1276_lna_gain_G2      = 2,
    sx1276_lna_gain_G3      = 3,
    sx1276_lna_gain_G4      = 4,
    sx1276_lna_gain_G5      = 5,
    sx1276_lna_gain_G6      = 6,    // Minimum Gain
}
sx1276_lna_gain_t;


typedef enum
{
    sx1276_rf_pin_rfo       = 0,
    sx1276_rf_pin_pa_boost  = 1,
}
sx1276_rf_pin_t;


typedef int sx1276_rssi_t;
typedef int sx1276_snr_t;  // fixed-point signed int with 2-bits of decimal


typedef uint8_t sx1276_irq_t;

extern const sx1276_irq_t sx1276_irq_rx_timeout;
extern const sx1276_irq_t sx1276_irq_rx_done;
extern const sx1276_irq_t sx1276_irq_payload_crc_error;
extern const sx1276_irq_t sx1276_irq_valid_header;
extern const sx1276_irq_t sx1276_irq_tx_done;
extern const sx1276_irq_t sx1276_irq_cad_done;
extern const sx1276_irq_t sx1276_irq_fhss_change_channel;
extern const sx1276_irq_t sx1276_irq_cad_detected;


typedef enum
{
    sx1276_dio_0 = 0,
    sx1276_dio_1 = 1,
    sx1276_dio_2 = 2,
    sx1276_dio_3 = 3,
    sx1276_dio_4 = 4,
    sx1276_dio_5 = 5,
}
sx1276_dio_t;


typedef uint8_t sx1276_dio_mapping_t;

extern sx1276_dio_mapping_t const sx1276_dio_0_rx_done;
extern sx1276_dio_mapping_t const sx1276_dio_0_tx_done;
extern sx1276_dio_mapping_t const sx1276_dio_0_cad_done;

extern sx1276_dio_mapping_t const sx1276_dio_1_rx_timeout;
extern sx1276_dio_mapping_t const sx1276_dio_1_fhss_change_channel;
extern sx1276_dio_mapping_t const sx1276_dio_1_cad_detected;

extern sx1276_dio_mapping_t const sx1276_dio_2_fhss_change_channel;

extern sx1276_dio_mapping_t const sx1276_dio_3_cad_done;
extern sx1276_dio_mapping_t const sx1276_dio_3_valid_header;
extern sx1276_dio_mapping_t const sx1276_dio_3_payload_crc_error;

extern sx1276_dio_mapping_t const sx1276_dio_4_cad_detected;
extern sx1276_dio_mapping_t const sx1276_dio_4_pll_lock;

extern sx1276_dio_mapping_t const sx1276_dio_5_mode_ready;
extern sx1276_dio_mapping_t const sx1276_dio_5_clk_out;


typedef enum
{
    sx1276_mode_sleep           = 0,
    sx1276_mode_stdby           = 1,    // Stand-by
    sx1276_mode_fstx            = 2,    // Frequency synthesis TX
    sx1276_mode_tx              = 3,    // Transmit
    sx1276_mode_fsrx            = 4,    // Frequency synthesis RX
    sx1276_mode_rx_continuous   = 5,    // Receive continuous
    sx1276_mode_rx_single       = 6,    // Receive continuous
    sx1276_mode_cad             = 7,    // Channel Activity Detection
}
sx1276_mode_t;


typedef enum
{
    sx1276_timer_resolution_disabled    = 0,
    sx1276_timer_resolution_64us        = 1,
    sx1276_timer_resolution_4_1ms       = 2,
    sx1276_timer_resolution_262ms       = 3,
}
sx1276_timer_resolution_t;


/*
 * Public functions
 */


/**
 * Module initialiser
 * Toggles the reset pin to perform a hardware reset
 * @note The chip comes up in Standby mode
 */
void sx1276_init(void);


/**
 * Disables the module and dependent modules
 */
void sx1276_deinit(void);


/**
 * Retrieves the value of the version register
 *
 * @return The value requested
 */
uint8_t sx1276_get_version(void);


/*
 * Configuration
 */


/**
 * Sets the clock source type
 *
 * @param Source    The source type
 */
void sx1276_set_clock_source_type(sx1276_clock_source_type_t const Source);


/**
 * Retrieves the clock source type
 *
 * @return The value requested
 */
sx1276_clock_source_type_t sx1276_get_clock_source_type(void);


/**
 * Sets the FRF register directly ( = Freq_Hz * 514,288 / 32,000,000)
 * @note Must only be set when in SLEEP or STANDBY mode
 *
 * @param Value The value to set
 */
void sx1276_set_frf(uint32_t const Value);


/**
 * Returns the stored FRF register value
 *
 * @return The value requested
 */
uint32_t sx1276_get_frf(void);


/**
 * Sets the frequency (Hz)
 * @note Must only be set when in SLEEP or STANDBY mode
 *
 * @param Value The value to set
 */
void sx1276_set_frequency(frequency_t const Value);


/**
 * Returns the stored device set frequency (Hz)
 *
 * @return The value requested
 */
frequency_t sx1276_get_frequency(void);


/**
 * Returns the device set frequency (Hz) for transmission, in accordance with
 * the errata note for optimal receiver settings
 *
 * @return The value requested
 */
frequency_t sx1276_get_real_frequency(void);


/**
 * Retrieves the actual frequency from the device (Hz)
 * @note This may be different depending on Rx/Tx mode in accordance with the
 *       errata note for optimal receiver settings
 * @return The value requested
 */
frequency_t sx1276_get_dev_frequency(void);


/**
 * Retrieves the device frequency error measurement (Hz)
 *
 * @return The value requested
 */
int32_t sx1276_get_frequency_error(void);


/**
 * Sets the PPM correction register (ppm)
 * @note Must only be set when in SLEEP or STANDBY mode
 *
 * @param Value The value to set
 */
void sx1276_set_ppm_correction(int8_t const Value);


/**
 * Returns the value of the ppm correction register (ppm)
 *
 * @return The value requested
 */
int8_t sx1276_get_ppm_correction(void);


/**
 * Controls the receiver low-noise amplifier (LNA)
 * @note Depends on frequency. If it jumps over the HF threshold, set_lna needs
 *       to be executed.
 *
 * @param Value         The gain value to set
 * @param BoostEnabled  Lowers the noise of the LNA by increasing its current
 *                      consumption
 */
void sx1276_set_lna(sx1276_lna_gain_t const Value, bool const BoostEnabled);


/**
 * Sets the target transmit power and output pin
 *
 * @param Pout  The output power (dBm)
 * @param Pin   The RF output pin to use
 */
void sx1276_set_tx_power(int const Pout, sx1276_rf_pin_t const Pin);


/**
 * Gets the target transmit power and output pin
 *
 * @param pPout Pointer to the output power (dBm)
 * @param pPin  Pointer to the RF output pin to use
 */
void sx1276_get_tx_power(int* const pPout, sx1276_rf_pin_t* const pPin);


/**
 * Sets the spreading factor
 *
 * @param Value The value to set
 */
void sx1276_set_spreading_factor(sx1276_sf_t const Value);


/**
 * Retrieves the spreading factor
 *
 * @return The value requested
 */
sx1276_sf_t sx1276_get_spreading_factor(void);


/**
 * Sets the bandwidth
 * @note This must be re-set after the frequency band is changed
 *
 * @param Value The value to set
 */
void sx1276_set_bandwidth(sx1276_bw_t const Value);


/**
 * Retrieves the bandwidth
 *
 * @return The value requested
 */
sx1276_bw_t sx1276_get_bandwidth(void);


/**
 * Sets the coding rate
 *
 * @param Value The value to set
 */
void sx1276_set_coding_rate(sx1276_cr_t const Value);


/**
 * Retrieves the coding rate
 *
 * @return The value requested
 */
sx1276_cr_t sx1276_get_coding_rate(void);


/**
 * Sets the symbol timeout (in symbols)
 *
 * @param Value The value to set
 */
void sx1276_set_symbol_timeout(uint16_t const Value);


/**
 * Retrieves the symbol timeout
 *
 * @return The value requested
 */
uint16_t sx1276_get_symbol_timeout(void);


/**
 * Sets the preamble length (in symbols)
 * @note The actual preamble length is Value + 4.25
 *
 * @param Value The value to set
 */
void sx1276_set_preamble_length(uint16_t const Value);


/**
 * Retrieves the preamble length
 *
 * @return The value requested
 */
uint16_t sx1276_get_preamble_length(void);


/**
 * Sets the synchronisation word
 * @note LoRaWAN is 0x34; Lora recommends 0x14 for user applications
 *
 * @param Value The value to set
 */
void sx1276_set_sync_word(uint8_t const Value);


/**
 * Retrieves the synchronisation word
 *
 * @return The value requested
 */
uint8_t sx1276_get_sync_word(void);


/**
 * Controls the implicit header mode
 * @see SX1276_EXPLICIT_HEADER_MODE
 * @note Set PacketSize to 0 for explicit mode
 *       Set 1..255 for the implicit mode packet size
 *
 * @note In explicit mode, the packet is expected to contain a header
 *
 * @param PacketSize    Sets the expected packet length
 */
void sx1276_set_header_mode(uint8_t const PacketSize);


/**
 * Retrieves the implicit mode flag and size
 *
 * @return 0 if explicit mode; otherwise implicit mode
 */
uint8_t sx1276_get_header_mode(void);


/**
 * Queries whether the module is set to implicit header mode
 *
 * @return true if so; false otherwise
 */
bool sx1276_is_header_mode_implicit(void);


/**
 * Controls whether the packet CRC is automatically checked
 *
 * @param Enabled   true to enable; false to disable
 */
void sx1276_set_rx_payload_crc_enabled(bool const Enabled);


/**
 * Retrieves the CRC auto-checking flag
 *
 * @return The value requested
 */
bool sx1276_get_rx_payload_crc_enabled(void);


/**
 * Retrieves whether the packet CRC is automatically checked
 *
 * @return true if so; false otherwise
 */
bool sx1276_is_rx_payload_crc_enabled(void);


/*
 * Sending and Receiving
 */


/**
 * Constructs and sends out a packet
 * @note Assumes the IRQ TX_DONE bit is set
 * @note Make Size larger than max packet length
 *
 * @param pData Pointer to the buffer to receive the packet data
 * @param Size  Number of bytes available in pData
 * @return Number of bytes written to pData
 */
size_t sx1276_send_packet(void* const pData, size_t const Size);


/**
 * Reads out a received packet
 * @note Assumes the IRQ RX_DONE bit is set and PAYLAD_CRC_ERROR is not
 * @note Make Size larger than max packet length
 *
 * @param pData Pointer to the buffer to receive the packet data
 * @param Size  Number of bytes available in pData
 * @return Number of bytes written to pData
 */
size_t sx1276_read_packet(void* const pData, size_t const Size);


/*
 * Information
 */


/**
 * Queries the RSSI of the last packet received
 *
 * @return The value requested
 */
sx1276_rssi_t sx1276_get_packet_rssi(void);


/**
 * Retrieves the wide-band RSSI reading
 * @note Requires the receiver to be active
 *
 * @return The value requested
 */
uint8_t sx1276_get_wideband_rssi(void);


/**
 * Queries the Signal-to-Noise Radio (SNR) of the last packet received
 *
 * @return The value requested
 */
sx1276_snr_t sx1276_get_packet_snr(void);


/**
 * Retrieves the valid packet header counter value
 *
 * @return The value requested
 */
uint16_t sx1276_get_valid_header_count(void);


/**
 * Retrieves the valid packet counter value
 *
 * @note According to the sx1276/7/8 errata note:
 * The valid packet counter (used for debug, and counting only packets whose
 * CRC is correct) presents the following issue:
 *   - In Rx Single mode, it does not increment
 *   - In Rx Continuous mode, it does not count the first valid packet received
 * Workaround
 *   - In Rx Single mode, do not use this counter, but instead increment a
 *     counter variable if PayloadCrcError == 0 on a RxDone interrupt.
 *   - In Rx Continuous mode, the same method can apply.
 *
 * @return The value requested
 */
uint16_t sx1276_get_valid_packet_count(void);


/*
 * Interrupts
 */


/**
 * Sets the value of the DIO pin mapping
 *
 * @param DIOMapping    The DIO mapping value to set
 */
void sx1276_set_dio_mapping(sx1276_dio_mapping_t const DIOMapping);


/**
 * Retrieves the value of the DIO pin mapping
 *
 * @param DIOPin        The DIO pin to set (0..5)
 * @return The value requested
 */
sx1276_dio_mapping_t sx1276_get_dio_mapping(sx1276_dio_t const DIOPin);


/**
 * Sets the chip sensitive to the IRQs
 * @note The IRQ mask register enables irqs for each bit set '0'
 *
 * @param IRQs  The IRQs to set
 */
void sx1276_set_irq_enable(sx1276_irq_t const IRQs);


/**
 * Retrieves the inverted value of the IRQ Mask register
 *
 * @return The value requested
 */
sx1276_irq_t sx1276_get_irq_enable(void);


/**
 * Retrieves the value of the IRQ register
 *
 * @return The value requested
 */
sx1276_irq_t sx1276_get_irq_flags(void);


/**
 * Retrieves the value of the IRQ register and clears all flags
 *
 * @return The value requested
 */
sx1276_irq_t sx1276_get_clear_irq_flags(void);


/*
 * Operating Modes
 */


/**
 * Sets the operating mode
 * @note The expected operation is to transition between TX|RX <-> STANDBY|SLEEP
 *       This is especially important in order for the function to apply
 *       frequency corrections
 *
 * @param Mode  The mode to select
 */
void sx1276_set_mode(sx1276_mode_t const Mode);


/**
 * Queries the device mode
 *
 * @return The value requested
 */
sx1276_mode_t sx1276_get_mode(void);


/**
 * Timer 1 (Receiver on-time)
 *
 * @param Resolution    The timer tick resolution
 * @param Coefficient   The timer period (ticks)
 */
void sx1276_set_timer1(sx1276_timer_resolution_t const Resolution,
                       uint8_t const Coefficient);


/**
 * Timer 2 (Receiver off-time)
 *
 * @param Resolution    The timer tick resolution
 * @param Coefficient   The timer period (ticks)
 */
void sx1276_set_timer2(sx1276_timer_resolution_t const Resolution,
                       uint8_t const Coefficient);


/*
 * Enum to String conversions
 */
const char* sx1276_clock_source_type_str(sx1276_clock_source_type_t const Value);

const char* sx1276_sf_str(sx1276_sf_t const Value);

const char* sx1276_bw_str(sx1276_bw_t const Value);

const char* sx1276_cr_str(sx1276_cr_t const Value);

const char* sx1276_lna_gain_str(sx1276_lna_gain_t const Value);

const char* sx1276_rf_pin_str(sx1276_rf_pin_t const Value);

char*       sx1276_snr_str(sx1276_snr_t const Value);

const char* sx1276_dio_str(sx1276_dio_t const Value);

const char* sx1276_dio_mapping_str(sx1276_dio_mapping_t const Value);

char*       sx1276_irq_str(sx1276_irq_t const Value);

const char* sx1276_mode_str(sx1276_mode_t const Value);


/**
 * Converts frequency (Hz) to FRF
 * @note Includes rounding
 *
 * @param Value The frequency to convert
 * @return The value requested
 */
uint32_t sx1276_frequency_to_frf(frequency_t const Value);


/**
 * Converts frequency (Hz) to FRF
 * @note Includes rounding
 *
 * @param Value The FRF value to convert
 * @return The value requested
 */
frequency_t sx1276_frf_to_frequency(uint32_t const Value);


/**
 * Converts the bandwidth enum value to the corresponding frequency (Hz)
 *
 * @param Value The enum value to convert
 * @return The value requested
 */
frequency_t sx1276_bw_hz(sx1276_bw_t const Value);


/*
 * Testing and development functions
 */
bool sx1276_test_fifo(bool const DoPrint);
