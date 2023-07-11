#include "util.h"

#include <stdint.h>
#include <inttypes.h>   // PRI printf macros

#include <stdbool.h>
#include <stdio.h>

#include "linker.h"
#include <system.h>

/*
 * Public Functions
 */
const char* util_bool_str(bool const Value)
{
    return Value ? "true" : "false";
}


void util_print_reset_cause(void)
{
    char const* p_str;

    switch (system_get_reset_cause())
    {
    /* The system was last reset by a backup reset */
    case SYSTEM_RESET_CAUSE_BACKUP:
        p_str = "SYSTEM_RESET_CAUSE_BACKUP";
        break;

    /* The system was last reset by a software reset */
    case SYSTEM_RESET_CAUSE_SOFTWARE:
        p_str = "SYSTEM_RESET_CAUSE_SOFTWARE";
        break;

    /* The system was last reset by the watchdog timer */
    case SYSTEM_RESET_CAUSE_WDT:
        p_str = "SYSTEM_RESET_CAUSE_WDT";
        break;

    /* The system was last reset because the external reset line was pulled low */
    case SYSTEM_RESET_CAUSE_EXTERNAL_RESET:
        p_str = "SYSTEM_RESET_CAUSE_EXTERNAL_RESET";
        break;

    /* The system was last reset by the BOD33 */
    case SYSTEM_RESET_CAUSE_BOD33:
        p_str = "SYSTEM_RESET_CAUSE_BOD33";
        break;

    /* The system was last reset by the BOD12 */
    case SYSTEM_RESET_CAUSE_BOD12:
        p_str = "SYSTEM_RESET_CAUSE_BOD12";
        break;

    /* The system was last reset by the POR (Power on reset). */
    case SYSTEM_RESET_CAUSE_POR:
        p_str = "SYSTEM_RESET_CAUSE_POR";
        break;

    default:
        p_str = "<Unknown>";
        break;
    }

    printf("Last reset cause: %s" UTIL_EOL, p_str);
}


void util_print_memory_usage(void)
{
    uint32_t const rom_used     = (uint32_t)&_efixed - (uint32_t)&_sfixed;
    uint32_t const rom_size     = 256 * 1024;

    uint32_t const ram_used     = (uint32_t)&_end - (uint32_t)&_srelocate;
    uint32_t const ram_size     = 40 * 1024;

    uint32_t const stack_size   = (uint32_t)&_estack - (uint32_t)&_sstack;

    printf("Memory usage:" UTIL_EOL
           "    Program Memory Usage: %" PRIu32 " bytes %0.1f %% Full" UTIL_EOL
           "    Data Memory Usage   : %" PRIu32 " bytes %0.1f %% Full" UTIL_EOL
           "    Stack Size          : %" PRIu32 UTIL_EOL,
           rom_used, 100.0f * (float)rom_used / rom_size,
           ram_used, 100.0f * (float)ram_used / ram_size,
           stack_size);
}


void util_print_uint8_hex(uint8_t* const pData, size_t const NumElements)
{
    for (size_t i = 0; i < NumElements; i++)
    {
        printf(" %02" PRIX8, pData[i]);
    }
}


void util_print_uint16_hex(uint16_t* const pData, size_t const NumElements)
{
    for (size_t i = 0; i < NumElements; i++)
    {
        printf(" %04" PRIX16, pData[i]);
    }
}


void util_print_uint32_hex(uint32_t* const pData, size_t const NumElements)
{
    for (size_t i = 0; i < NumElements; i++)
    {
        printf(" %08" PRIX32, pData[i]);
    }
}


void util_print_uint64_hex(uint64_t* const pData, size_t const NumElements)
{
    for (size_t i = 0; i < NumElements; i++)
    {
        printf(" %016" PRIX64, pData[i]);
    }
}


char* util_strcpy(char* pDest, const char* pSrc)
{
    char c;

    do
    {
        c = *pSrc++;
        *pDest++ = c;
    }
    while (c != '\0');

    return pDest - 1;
}


void util_clear(void* const pDest, size_t const Length)
{
    size_t const start  = (size_t) pDest;
    size_t const end    = start + Length;
    uint8_t* p_8        = (uint8_t*) start;

    if ((start & 3) > 0)  // Misaligned start
    {
        // End is the next word boundary or the end of the array
        size_t const first_word_end = (start + 4) & ~3;
        uint8_t* p_8e = (uint8_t*) UTIL_MIN(first_word_end, end);
        while (p_8 < p_8e)
        {
            *p_8++ = 0;
        }
    }

    if (p_8 < (uint8_t*)end)
    {
        // Fast clear as many elements as possible
        uint32_t* p_32  = (uint32_t*) ((void*)p_8);
        uint32_t* p_32e = (uint32_t*) (end & ~3);
        while (p_32 < p_32e)
        {
            *p_32++ = 0UL;
        }

        if ((Length & 3) > 0)   // Length wasn't a multiple of words
        {
            p_8 = (uint8_t*) p_32e;
            uint8_t* p_8e = (uint8_t*) end;
            while (p_8 < p_8e)
            {
                *p_8++ = 0;
            }
        }
    }
}


uint16_t util_bytes_to_uint16_le(uint8_t* const pInput)
{
    return (((uint16_t)pInput[0]) << 0) |
           (((uint16_t)pInput[1]) << 8);
}


void util_uint16_to_bytes_le(uint8_t* const pOutput, uint16_t const Value)
{
    pOutput[0] = (Value >> 0) & 0xFF;
    pOutput[1] = (Value >> 8) & 0xFF;
}


uint16_t util_bytes_to_uint16_be(uint8_t* const pInput)
{
    return (((uint16_t)pInput[0]) << 8) |
           (((uint16_t)pInput[1]) << 0);
}


void util_uint16_to_bytes_be(uint8_t* const pOutput, uint16_t const Value)
{
    pOutput[0] = (Value >> 8) & 0xFF;
    pOutput[1] = (Value >> 0) & 0xFF;
}


uint32_t util_bytes_to_uint24_le(uint8_t* const pInput)
{
    return (((uint32_t)pInput[0]) << 0) |
           (((uint32_t)pInput[1]) << 8) |
           (((uint32_t)pInput[2]) << 16);
}


void util_uint24_to_bytes_le(uint8_t* const pOutput, uint32_t const Value)
{
    pOutput[0] = (Value >> 0) & 0xFF;
    pOutput[1] = (Value >> 8) & 0xFF;
    pOutput[2] = (Value >> 16) & 0xFF;
}


uint32_t util_bytes_to_uint24_be(uint8_t* const pInput)
{
    return (((uint32_t)pInput[0]) << 16) |
           (((uint32_t)pInput[1]) << 8) |
           (((uint32_t)pInput[2]) << 0);
}


void util_uint24_to_bytes_be(uint8_t* const pOutput, uint32_t const Value)
{
    pOutput[0] = (Value >> 16) & 0xFF;
    pOutput[1] = (Value >> 8) & 0xFF;
    pOutput[2] = (Value >> 0) & 0xFF;
}


uint32_t util_bytes_to_uint32_le(uint8_t* const pInput)
{
    return (((uint32_t)pInput[0]) << 0) |
           (((uint32_t)pInput[1]) << 8) |
           (((uint32_t)pInput[2]) << 16) |
           (((uint32_t)pInput[3]) << 24);
}


void util_uint32_to_bytes_le(uint8_t* const pOutput, uint32_t const Value)
{
    pOutput[0] = (Value >> 0) & 0xFF;
    pOutput[1] = (Value >> 8) & 0xFF;
    pOutput[2] = (Value >> 16) & 0xFF;
    pOutput[3] = (Value >> 24) & 0xFF;
}


uint32_t util_bytes_to_uint32_be(uint8_t* const pInput)
{
    return (((uint32_t)pInput[0]) << 24) |
           (((uint32_t)pInput[1]) << 16) |
           (((uint32_t)pInput[2]) << 8) |
           (((uint32_t)pInput[3]) << 0);
}


void util_uint32_to_bytes_be(uint8_t* const pOutput, uint32_t const Value)
{
    pOutput[0] = (Value >> 24) & 0xFF;
    pOutput[1] = (Value >> 16) & 0xFF;
    pOutput[2] = (Value >> 8) & 0xFF;
    pOutput[3] = (Value >> 0) & 0xFF;
}
