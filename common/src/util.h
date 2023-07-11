/**
 * Utilities
 *
 * @author  David Hughes
 * @date    30Jun23
 */
#pragma once

#include <stdint.h>     // uint8_t, uint16_t, uint32_t, uint64_t
#include <stdbool.h>    // bool
#include <stddef.h>     // size_t


/*
 * Public constants
 */
#define UTIL_EOL    "\r\n"


/*
 * Public macros
 */
#define UTIL_ABS(Value)                 (((Value) < 0) ? -(Value) : (Value))
#define UTIL_MIN(A, B)                  (((A) < (B)) ? (A) : (B))
#define UTIL_MAX(A, B)                  (((A) > (B)) ? (A) : (B))
#define UTIL_LIMIT(Value, Low, High)    ((Value) < (Low) ? (Low) : ((Value) > (High) ? (High) : (Value)))
#define UTIL_ARRAY_LEN(Array)           (sizeof(Array) / sizeof((Array)[0]))


/*
 * Public Functions
 */
const char* util_bool_str(bool const Value);


void util_print_reset_cause(void);


void util_print_memory_usage(void);


void util_print_uint8_hex(uint8_t* const pData, size_t const NumElements);


void util_print_uint16_hex(uint16_t* const pData, size_t const NumElements);


void util_print_uint32_hex(uint32_t* const pData, size_t const NumElements);


void util_print_uint64_hex(uint64_t* const pData, size_t const NumElements);


/**
 * Copies the pSrc to pDest including the NUL terminator, returning the last
 * character of pDest written
 */
char* util_strcpy(char* pDest, const char* pSrc);


/**
 * Clears Length bytes of memory starting at pDest
 *
 * @param pDest     Pointer tot he block of data to clear
 * @param Length    Number of bytes to clear
 */
void util_clear(void* const pDest, size_t const Length);


/**
 * Converts a little-endian byte stream to a unsigned 16-bit integer
 * @note The byte stream must be at least 2 bytes long
 *
 * @param pInput    Pointer to the byte stream
 * @return The value requested
 */
uint16_t util_bytes_to_uint16_le(uint8_t* const pInput);


/**
 * Converts an unsigned 16-bit integer to a little-endian byte stream
 * @note The byte stream must be at least 2 bytes long
 *
 * @param pOutput   Pointer to the byte stream
 * @param Value     Value to encode
 */
void util_uint16_to_bytes_le(uint8_t* const pOutput, uint16_t const Value);


/**
 * Converts a big-endian byte stream to a unsigned 16-bit integer
 * @note The byte stream must be at least 2 bytes long
 *
 * @param pInput    Pointer to the byte stream
 * @return The value requested
 */
uint16_t util_bytes_to_uint16_be(uint8_t* const pInput);


/**
 * Converts an unsigned 16-bit integer to a big-endian byte stream
 * @note The byte stream must be at least 2 bytes long
 *
 * @param pOutput   Pointer to the byte stream
 * @param Value     Value to encode
 */
void util_uint16_to_bytes_be(uint8_t* const pOutput, uint16_t const Value);


/**
 * Converts a little-endian byte stream to a unsigned 24-bit integer
 * @note The byte stream must be at least 3 bytes long
 *
 * @param pInput    Pointer to the byte stream
 * @return The value requested
 */
uint32_t util_bytes_to_uint24_le(uint8_t* const pInput);


/**
 * Converts an unsigned 24-bit integer to a little-endian byte stream
 * @note The byte stream must be at least 3 bytes long
 *
 * @param pOutput   Pointer to the byte stream
 * @param Value     Value to encode
 */
void util_uint24_to_bytes_le(uint8_t* const pOutput, uint32_t const Value);


/**
 * Converts a big-endian byte stream to a unsigned 24-bit integer
 * @note The byte stream must be at least 3 bytes long
 *
 * @param pInput    Pointer to the byte stream
 * @return The value requested
 */
uint32_t util_bytes_to_uint24_be(uint8_t* const pInput);


/**
 * Converts an unsigned 24-bit integer to a big-endian byte stream
 * @note The byte stream must be at least 3 bytes long
 *
 * @param pOutput   Pointer to the byte stream
 * @param Value     Value to encode
 */
void util_uint24_to_bytes_be(uint8_t* const pOutput, uint32_t const Value);


/**
 * Converts a little-endian byte stream to a unsigned 32-bit integer
 * @note The byte stream must be at least 4 bytes long
 *
 * @param pInput    Pointer to the byte stream
 * @return The value requested
 */
uint32_t util_bytes_to_uint32_le(uint8_t* const pInput);


/**
 * Converts an unsigned 32-bit integer to a little-endian byte stream
 * @note The byte stream must be at least 4 bytes long
 *
 * @param pOutput   Pointer to the byte stream
 * @param Value     Value to encode
 */
void util_uint32_to_bytes_le(uint8_t* const pOutput, uint32_t const Value);


/**
 * Converts a big-endian byte stream to a unsigned 32-bit integer
 * @note The byte stream must be at least 4 bytes long
 *
 * @param pInput    Pointer to the byte stream
 * @return The value requested
 */
uint32_t util_bytes_to_uint32_be(uint8_t* const pInput);


/**
 * Converts an unsigned 32-bit integer to a big-endian byte stream
 * @note The byte stream must be at least 4 bytes long
 *
 * @param pOutput   Pointer to the byte stream
 * @param Value     Value to encode
 */
void util_uint32_to_bytes_be(uint8_t* const pOutput, uint32_t const Value);
