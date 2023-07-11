/**
 * Message Encryption / decryption
 *
 * @author  David Hughes
 * @date    30Jun23
 */
#pragma once

#include <stdint.h>     // uint32_t
#include <stddef.h>     // size_t


/*
 * Public types
 */
#define MESSAGE_BLOCK_LENGTH_BYTES  (16)    // 128-bits
#define MESSAGE_BLOCK_LENGTH_U32    (MESSAGE_BLOCK_LENGTH_BYTES / sizeof(uint32_t))


/*
 * Public functions
 */


/**
 * Initialises and enables the hardware modules.
 */
void message_init(void);


/**
 * Shuts down the module
 */
void message_deinit(void);


/**
 * Sets the key for the module.
 * @note pKey must be 32-bit aligned and be able to contain 16-bytes
 * @note This function does not copy the key as it is expected to come from a
 *       static buffer.
 *
 * @param pKey      Pointer to the buffer containing the key
 */
void message_set_key(void* const pKey);


/**
 * Gets 16-bytes of random data for use as an IV or Key
 * @note pOutput must be 32-bit aligned and be able to contain 16-bytes
 *
 * @param pOutput   Pointer to output
 */
void message_get_random_block(void* const pOutput);


/**
 * Encrypts blocks of data. Preceding the message with the IV
 * @note pIV must be 32-bit aligned and MESSAGE_BLOCK_LENGTH_U32 long
 * @note pInput and pOutput must be 32-bit aligned and the same length.
 * @note Length must be a multiple of MESSAGE_BLOCK_LENGTH_U32
 *
 * @param pIV       Pointer to the Initialisation Vector
 * @param pInput    Pointer to the input data block
 * @param pOutput   Pointer to the output data block
 * @param Length    Number of bytes in pInput and pOutput
 */
void message_encrypt(void* const pIV,
                     void* const pInput,
                     void* const pOutput,
                     size_t const Length);


/**
 * Decrypts blocks of data
 * @note pIV must be 32-bit aligned and MESSAGE_BLOCK_LENGTH_U32 long
 * @note pInput and pOutput must be 32-bit aligned and the same length.
 * @note Length must be a multiple of MESSAGE_BLOCK_LENGTH_U32
 *
 * @param pIV       Pointer to the Initialisation Vector
 * @param pInput    Pointer to the input data block
 * @param pOutput   Pointer to the output data block
 * @param Length    Number of bytes in pInput and pOutput
 */
void message_decrypt(void* const pIV,
                     void* const pInput,
                     void* const pOutput,
                     size_t const Length);
