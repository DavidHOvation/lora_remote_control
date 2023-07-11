/**
 * LoRa-specific
 *
 * @author  David Hughes
 * @date    30Jun23
 */
#pragma once

#include "frequency.h"


/*
 * Public constants
 */
#define LORA_MAX_PACKET_SIZE    255


/// @ref https://www.rfwireless-world.com/Tutorials/LoRa-channels-list.html

#define LORA_CH_10_868 	        FREQUENCY_MHZ(865.2)
#define LORA_CH_11_868 	        FREQUENCY_MHZ(865.5)
#define LORA_CH_12_868 	        FREQUENCY_MHZ(865.8)
#define LORA_CH_13_868 	        FREQUENCY_MHZ(866.1)
#define LORA_CH_14_868 	        FREQUENCY_MHZ(866.4)
#define LORA_CH_15_868 	        FREQUENCY_MHZ(866.7)
#define LORA_CH_16_868 	        FREQUENCY_MHZ(867)
#define LORA_CH_17_868 	        FREQUENCY_MHZ(868)

#define LORA_CH_00_900          FREQUENCY_MHZ(903.08)
#define LORA_CH_01_900          FREQUENCY_MHZ(905.24)
#define LORA_CH_02_900          FREQUENCY_MHZ(907.40)
#define LORA_CH_03_900          FREQUENCY_MHZ(909.56)
#define LORA_CH_04_900          FREQUENCY_MHZ(911.72)
#define LORA_CH_05_900          FREQUENCY_MHZ(913.88)
#define LORA_CH_06_900          FREQUENCY_MHZ(916.04)
#define LORA_CH_07_900          FREQUENCY_MHZ(918.20)
#define LORA_CH_08_900          FREQUENCY_MHZ(920.36)
#define LORA_CH_09_900          FREQUENCY_MHZ(922.52)
#define LORA_CH_10_900          FREQUENCY_MHZ(924.68)
#define LORA_CH_11_900          FREQUENCY_MHZ(926.84)
#define LORA_CH_12_900          FREQUENCY_MHZ(915)
