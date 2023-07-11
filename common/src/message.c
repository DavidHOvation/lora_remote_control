#include "message.h"

#include <stdint.h>     // uint32_t
#include <stddef.h>     // size_t
#include <assert.h>

#include <trng.h>
#include <aes.h>

#include "util.h"


/*
 * Private data
 */
static struct
{
    uint32_t* p_aes_key;

    struct aes_config aes_cfg_encrypt;
    struct aes_config aes_cfg_decrypt;
    struct aes_module aes_instance;
}
_message;


/*
 * Private functions
 */
void _message_encrypt_decrypt(void* const pIV,
                              void* const pInput,
                              void* const pOutput,
                              size_t const Length);


/*
 * Public functions
 */
void message_init(void)
{
    util_clear(&_message, sizeof(_message));

    // Configure AES engine
    aes_get_config_defaults(&_message.aes_cfg_encrypt);
    _message.aes_cfg_encrypt.encrypt_mode   = AES_ENCRYPTION;
    _message.aes_cfg_encrypt.key_size       = AES_KEY_SIZE_128;
    _message.aes_cfg_encrypt.start_mode     = AES_MANUAL_START;
    _message.aes_cfg_encrypt.opmode         = AES_CBC_MODE;

    aes_get_config_defaults(&_message.aes_cfg_decrypt);
    _message.aes_cfg_decrypt.encrypt_mode   = AES_DECRYPTION;
    _message.aes_cfg_decrypt.key_size       = AES_KEY_SIZE_128;
    _message.aes_cfg_decrypt.start_mode     = AES_MANUAL_START;
    _message.aes_cfg_decrypt.opmode         = AES_CBC_MODE;

    aes_init(&_message.aes_instance, AES, &_message.aes_cfg_encrypt);
    aes_enable(&_message.aes_instance);

    // AES module has hardware to prevent figuring out an AES key from the
    // power consumption. This uses a PRNG bit-stream seeded from a 32bit
    // random number. We reset it here before every AES operation.
    aes_write_random_seed(&_message.aes_instance, 1);
}


void message_deinit(void)
{
    aes_disable(&_message.aes_instance);
}


void message_set_key(void* const pKey)
{
    _message.p_aes_key = pKey;
}


void message_get_random_block(void* const pOutput)
{
    uint32_t* const p_output = (uint32_t*) pOutput;

    struct trng_config cfg;
    trng_get_config_defaults(&cfg);

    struct trng_module trng;
    trng_init(&trng, TRNG, &cfg);
    trng_enable(&trng);

    for (size_t i = 0; i < MESSAGE_BLOCK_LENGTH_U32; i++)
    {
        while (trng_read(&trng, &p_output[i]) != STATUS_OK);
    }

    trng_disable(&trng);
}


void message_encrypt(void* const pIV,
                     void* const pInput,
                     void* const pOutput,
                     size_t const Length)
{
    aes_set_config(&_message.aes_instance, AES, &_message.aes_cfg_encrypt);

    _message_encrypt_decrypt(pIV, pInput, pOutput, Length);
}


void message_decrypt(void* const pIV,
                     void* const pInput,
                     void* const pOutput,
                     size_t const Length)
{
    aes_set_config(&_message.aes_instance, AES, &_message.aes_cfg_decrypt);

    _message_encrypt_decrypt(pIV, pInput, pOutput, Length);
}


/*
 * Private functions
 */
void _message_encrypt_decrypt(void* const pIV,
                              void* const pInput,
                              void* const pOutput,
                              size_t const Length)
{
    assert((Length > 0) && ((Length % MESSAGE_BLOCK_LENGTH_BYTES) == 0));
    
    // All operations are 32-bit
    size_t const length = Length / sizeof(uint32_t);
    uint32_t* p_input = (uint32_t*)pInput;
    uint32_t* p_output = (uint32_t*)pOutput;

    // Flag the first block so the IV gets used
    aes_write_key(&_message.aes_instance, _message.p_aes_key);
    aes_write_init_vector(&_message.aes_instance, pIV);
    aes_set_new_message(&_message.aes_instance);

    for (size_t i = 0; i < length; i += MESSAGE_BLOCK_LENGTH_U32)
    {
        aes_write_input_data(&_message.aes_instance, &p_input[i]);
        aes_start(&_message.aes_instance);

        // Wait for the end of the encryption process
        while (!(aes_get_status(&_message.aes_instance) &
                 AES_ENCRYPTION_COMPLETE));

        aes_read_output_data(&_message.aes_instance, &p_output[i]);
        aes_clear_new_message(&_message.aes_instance);
    }
}
