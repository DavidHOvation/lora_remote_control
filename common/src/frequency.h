/**
 * Frequency Type
 *
 * @author  David Hughes
 * @date    30Jun23
 */
#pragma once


typedef unsigned long frequency_t;       // Hz
typedef signed long long frequency_diff_t;  // Hz


#define FREQUENCY_MHZ(Value)        ((frequency_t)(Value * 1e6))
