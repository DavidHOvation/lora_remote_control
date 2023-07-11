/**
 * Ovation PCB 1350A definitions
 *
 * Symbols that describe features and capabilities of the board.
 *
 * @author  David Hughes
 * @date    30Jun23
 */
#pragma once


#include <conf_board.h>
#include <compiler.h>

#ifdef __cplusplus
extern "C" {
#endif


/** Name string macro */
#define BOARD_NAME                          "OVATION_PCB1350A"


/** \name Resonator definitions
 *  @{ */
#define BOARD_FREQ_SLCK_XTAL                (32768U)
#define BOARD_FREQ_SLCK_BYPASS              (32768U)
#define BOARD_FREQ_MAINCK_XTAL              0 /* Not Mounted */
#define BOARD_FREQ_MAINCK_BYPASS            0 /* Not Mounted */
#define BOARD_MCK                           CHIP_FREQ_CPU_MAX
#define BOARD_OSC_STARTUP_US                15625
/** @} */

#define BOARD_PIN_LOW                       (false)
#define BOARD_PIN_HIGH                      (true)


#define BOARD_USB_TARGET_DP_PIN             PIN_PA25G_USB_DP
#define BOARD_USB_TARGET_DP_MUX             MUX_PA25G_USB_DP

#define BOARD_USB_TARGET_DM_PIN             PIN_PA24G_USB_DM
#define BOARD_USB_TARGET_DM_MUX             MUX_PA24G_USB_DM

#define BOARD_USB_VBUS_PIN                  PIN_PA07
#define BOARD_USB_VBUS_EIC_PIN              PIN_PA07A_EIC_EXTINT7
#define BOARD_USB_VBUS_EIC_MUX              MUX_PA07A_EIC_EXTINT7
#define BOARD_USB_VBUS_EIC_LINE             7

#define BOARD_USB_ID_PIN                    PIN_PA15    // with pull-up!
#define BOARD_USB_ID_EIC_PIN                PIN_PA15A_EIC_EXTINT15
#define BOARD_USB_ID_EIC_MUX                MUX_PA15A_EIC_EXTINT15
#define BOARD_USB_ID_EIC_LINE               15


// Inter-die connections
#define BOARD_RF_SPI                        SERCOM4

#define BOARD_RF_SPI_MOSI_PIN               PIN_PB30
#define BOARD_RF_SPI_MOSI_ACTIVE            BOARD_PIN_LOW

#define BOARD_RF_SPI_SCK_PIN                PIN_PC18
#define BOARD_RF_SPI_SCK_ACTIVE             BOARD_PIN_LOW

#define BOARD_RF_SPI_MISO_PIN               PIN_PC19

#define BOARD_RF_SPI_CS_PIN                 PIN_PB31
#define BOARD_RF_SPI_CS_ACTIVE              BOARD_PIN_LOW

#define BOARD_RF_SPI_SERCOM_MUX_SETTING     SPI_SIGNAL_MUX_SETTING_E
#define BOARD_RF_SPI_SERCOM_PINMUX_PAD0     PINMUX_PC19F_SERCOM4_PAD0   // MISO
#define BOARD_RF_SPI_SERCOM_PINMUX_PAD1     PINMUX_UNUSED
#define BOARD_RF_SPI_SERCOM_PINMUX_PAD2     PINMUX_PB30F_SERCOM4_PAD2   // MOSI
#define BOARD_RF_SPI_SERCOM_PINMUX_PAD3     PINMUX_PC18F_SERCOM4_PAD3   // SCK

#define BOARD_RF_DIO0_PIN                   PIN_PB16
#define BOARD_RF_DIO0_ACTIVE                BOARD_PIN_HIGH
#define BOARD_RF_DIO0_EIC_PIN               PIN_PB16A_EIC_EXTINT0
#define BOARD_RF_DIO0_EIC_MUX               MUX_PB16A_EIC_EXTINT0
#define BOARD_RF_DIO0_EIC_LINE              0

#define BOARD_RF_DIO1_PIN                   PIN_PA11
#define BOARD_RF_DIO1_ACTIVE                BOARD_PIN_HIGH
#define BOARD_RF_DIO1_EIC_PIN               PIN_PA11A_EIC_EXTINT11
#define BOARD_RF_DIO1_EIC_MUX               MUX_PA11A_EIC_EXTINT11
#define BOARD_RF_DIO1_EIC_LINE              11

#define BOARD_RF_DIO2_PIN                   PIN_PA12
#define BOARD_RF_DIO2_ACTIVE                BOARD_PIN_HIGH
#define BOARD_RF_DIO2_EIC_PIN               PIN_PA12A_EIC_EXTINT12
#define BOARD_RF_DIO2_EIC_MUX               MUX_PA12A_EIC_EXTINT12
#define BOARD_RF_DIO2_EIC_LINE              12

#define BOARD_RF_DIO3_PIN                   PIN_PB17
#define BOARD_RF_DIO3_ACTIVE                BOARD_PIN_HIGH
#define BOARD_RF_DIO3_EIC_PIN               PIN_PB17A_EIC_EXTINT1
#define BOARD_RF_DIO3_EIC_MUX               MUX_PB17A_EIC_EXTINT1
#define BOARD_RF_DIO3_EIC_LINE              1

#define BOARD_RF_DIO4_PIN                   PIN_PA10
#define BOARD_RF_DIO4_ACTIVE                BOARD_PIN_HIGH
#define BOARD_RF_DIO4_EIC_PIN               PIN_PA10A_EIC_EXTINT10
#define BOARD_RF_DIO4_EIC_MUX               MUX_PA10A_EIC_EXTINT10
#define BOARD_RF_DIO4_EIC_LINE              10

#define BOARD_RF_DIO5_PIN                   PIN_PB00
#define BOARD_RF_DIO5_ACTIVE                BOARD_PIN_HIGH
#define BOARD_RF_DIO5_EIC_PIN               PIN_PB00A_EIC_EXTINT0
#define BOARD_RF_DIO5_EIC_MUX               MUX_PB00A_EIC_EXTINT0
#define BOARD_RF_DIO5_EIC_LINE              0

#define BOARD_RF_RESET_PIN                  PIN_PB15
#define BOARD_RF_RESET_ACTIVE               BOARD_PIN_LOW

#define BOARD_RF_SWITCH_PIN                 PIN_PA13
#define BOARD_RF_SWITCH_ACTIVE              BOARD_PIN_HIGH

#define BOARD_TCXO_PWR_PIN                  PIN_PA09
#define BOARD_TCXO_PWR_ACTIVE               BOARD_PIN_HIGH

#define BOARD_SWITCH1_PIN                   PIN_PA18
#define BOARD_SWITCH1_ACTIVE                BOARD_PIN_LOW
#define BOARD_SWITCH1_EIC_PIN               PIN_PA18A_EIC_EXTINT2
#define BOARD_SWITCH1_EIC_MUX               MUX_PA18A_EIC_EXTINT2
#define BOARD_SWITCH1_EIC_LINE              2

#define BOARD_SWITCH2_PIN                   PIN_PA19
#define BOARD_SWITCH2_ACTIVE                BOARD_PIN_LOW
#define BOARD_SWITCH2_EIC_PIN               PIN_PA19A_EIC_EXTINT3
#define BOARD_SWITCH2_EIC_MUX               MUX_PA19A_EIC_EXTINT3
#define BOARD_SWITCH2_EIC_LINE              3

#define BOARD_SWITCH3_PIN                   PIN_PA22
#define BOARD_SWITCH3_ACTIVE                BOARD_PIN_LOW
#define BOARD_SWITCH3_EIC_PIN               PIN_PA22A_EIC_EXTINT6
#define BOARD_SWITCH3_EIC_MUX               MUX_PA22A_EIC_EXTINT6
#define BOARD_SWITCH3_EIC_LINE              6

#define BOARD_SWITCH4_PIN                   PIN_PA23
#define BOARD_SWITCH4_ACTIVE                BOARD_PIN_LOW
#define BOARD_SWITCH4_EIC_PIN               PIN_PA23A_EIC_EXTINT7
#define BOARD_SWITCH4_EIC_MUX               MUX_PA23A_EIC_EXTINT7
#define BOARD_SWITCH4_EIC_LINE              7

#define BOARD_BUTTON_PIN                    PIN_PA28
#define BOARD_BUTTON_ACTIVE                 BOARD_PIN_LOW
#define BOARD_BUTTON_EIC_PIN                PIN_PA28A_EIC_EXTINT8
#define BOARD_BUTTON_EIC_MUX                MUX_PA28A_EIC_EXTINT8
#define BOARD_BUTTON_EIC_LINE               8


#define BOARD_S0_UART_MODULE                SERCOM0
#define BOARD_S0_UART_SERCOM_MUX_SETTING    USART_RX_1_TX_0_XCK_1
#define BOARD_S0_UART_SERCOM_PINMUX_PAD0    PINMUX_PA04D_SERCOM0_PAD0   // Data to Device
#define BOARD_S0_UART_SERCOM_PINMUX_PAD1    PINMUX_PA05D_SERCOM0_PAD1   // Data from Device
#define BOARD_S0_UART_SERCOM_PINMUX_PAD2    PINMUX_UNUSED
#define BOARD_S0_UART_SERCOM_PINMUX_PAD3    PINMUX_UNUSED


#define BOARD_S0_UART_TX_PIN                PIN_PA04
#define BOARD_S0_UART_TX_ACTIVE             BOARD_PIN_HIGH

#define BOARD_S0_UART_RX_PIN                PIN_PA05
#define BOARD_S0_UART_RX_ACTIVE             BOARD_PIN_LOW

#define BOARD_GPIO_OUT2_PIN                 PIN_PA06
#define BOARD_GPIO_OUT2_ACTIVE              BOARD_PIN_HIGH

#define BOARD_GPIO_INPUT1_PIN               PIN_PA08
#define BOARD_GPIO_INPUT1_ACTIVE            BOARD_PIN_LOW


#define BOARD_BT_UART_MODULE                SERCOM5
#define BOARD_BT_UART_SERCOM_MUX_SETTING    USART_RX_3_TX_2_XCK_3
#define BOARD_BT_UART_SERCOM_PINMUX_PAD0    PINMUX_UNUSED
#define BOARD_BT_UART_SERCOM_PINMUX_PAD1    PINMUX_UNUSED
#define BOARD_BT_UART_SERCOM_PINMUX_PAD2    PINMUX_PB22D_SERCOM5_PAD2   // Data to BT
#define BOARD_BT_UART_SERCOM_PINMUX_PAD3    PINMUX_PB23D_SERCOM5_PAD3   // Data from BT


#define BOARD_I2C_MODULE                    SERCOM1
#define BOARD_I2C_SERCOM_PINMUX_PAD0        PINMUX_PA16C_SERCOM1_PAD0   // SDA
#define BOARD_I2C_SERCOM_PINMUX_PAD1        PINMUX_PA17C_SERCOM1_PAD1   // SCL


// IO expanded Pins
#define BOARD_LED_RGB_RED_PIN               IOEXPANDER_IO0_0
#define BOARD_LED_RGB_RED_ACTIVE            BOARD_PIN_LOW

#define BOARD_LED_RGB_GREEN_PIN             IOEXPANDER_IO0_1
#define BOARD_LED_RGB_GREEN_ACTIVE          BOARD_PIN_LOW

#define BOARD_LED_RGB_BLUE_PIN              IOEXPANDER_IO0_2
#define BOARD_LED_RGB_BLUE_ACTIVE           BOARD_PIN_LOW

#define BOARD_LED_STAT_RED_PIN              IOEXPANDER_IO0_3
#define BOARD_LED_STAT_RED_ACTIVE           BOARD_PIN_LOW

#define BOARD_LED_STAT_GREEN_PIN            IOEXPANDER_IO0_4
#define BOARD_LED_STAT_GREEN_ACTIVE         BOARD_PIN_LOW

#define BOARD_OPTION_0_PIN                  IOEXPANDER_IO1_0
#define BOARD_OPTION_0_ACTIVE               BOARD_PIN_LOW

#define BOARD_OPTION_1_PIN                  IOEXPANDER_IO1_1
#define BOARD_OPTION_1_ACTIVE               BOARD_PIN_LOW

#define BOARD_OPTION_2_PIN                  IOEXPANDER_IO1_2
#define BOARD_OPTION_2_ACTIVE               BOARD_PIN_LOW

#define BOARD_USB_MAXCHARGE_PIN             IOEXPANDER_IO1_3
#define BOARD_USB_MAXCHARGE_ACTIVE          BOARD_PIN_HIGH

#define BOARD_PWR_ON_PIN                    IOEXPANDER_IO1_4
#define BOARD_PWR_ON_ACTIVE                 BOARD_PIN_LOW

#define BOARD_PWROFF_PIN                    IOEXPANDER_IO1_5
#define BOARD_PWROFF_ACTIVE                 BOARD_PIN_LOW

#define BOARD_CHARGING_PIN                  IOEXPANDER_IO1_6
#define BOARD_CHARGING_ACTIVE               BOARD_PIN_LOW

#define BOARD_PWRIN_GOOD_PIN                IOEXPANDER_IO1_7
#define BOARD_PWRIN_GOOD_ACTIVE             BOARD_PIN_LOW


/*
 * Global symbols for Microchip ASF framework
 */
#define USB_TARGET_DP_PIN                   BOARD_USB_TARGET_DP_PIN
#define USB_TARGET_DP_MUX                   BOARD_USB_TARGET_DP_MUX

#define USB_TARGET_DM_PIN                   BOARD_USB_TARGET_DM_PIN
#define USB_TARGET_DM_MUX                   BOARD_USB_TARGET_DM_MUX

#define USB_VBUS_PIN                        BOARD_USB_VBUS_PIN
#define USB_VBUS_EIC_PIN                    BOARD_USB_VBUS_EIC_PIN
#define USB_VBUS_EIC_MUX                    BOARD_USB_VBUS_EIC_MUX
#define USB_VBUS_EIC_LINE                   BOARD_USB_VBUS_EIC_LINE

#define USB_ID_PIN                          BOARD_USB_ID_PIN
#define USB_ID_EIC_MUX                      BOARD_USB_ID_EIC_MUX
#define USB_ID_EIC_LINE                     BOARD_USB_ID_EIC_LINE


#ifdef __cplusplus
}
#endif
