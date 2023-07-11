/**
 * Get access to variables defined in the linker script
 * \src\ASF\sam0\utils\linker_scripts\samr34\gcc\samr34j18b_flash.ld
 *
 * @author  David Hughes
 * @date    30Jun23
 */
#pragma once

#include <stdint.h>  // uint32_t


extern uint32_t _sfixed;
extern uint32_t _efixed;
extern uint32_t _etext;
extern uint32_t _srelocate;
extern uint32_t _erelocate;
extern uint32_t _szero;
extern uint32_t _ezero;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _slpram;
extern uint32_t _elpram;
extern uint32_t _sstack;
extern uint32_t _estack;
extern uint32_t _end;
