#include "hal-pcb1350a.h"

#include <stdbool.h>    // bool
#include <assert.h>

// Microchip
#include <system.h>

#include <extint.h>
#include <delay.h>
#include <spi.h>
#include <i2c_master.h>
#include <port.h>
#include <interrupt.h>
#include <tc.h>
#include <tc_interrupt.h>



#include "hal_cdc.h"


#include "util.h"       // util_clear
#include "pca9539a.h"   // port expander device
#include "ioexpander.h" // port expander function


/*
 * Private constants
 */
#define BOARD_RF_SPI_BAUDRATE   (10000000)


/*
 * Private data
 */
static struct
{
    uint32_t interrupt_level;

    struct spi_module           rf_spi;
    struct spi_slave_inst       rf_spi_dev;

    struct i2c_master_module    i2c;
    pca9539a_t                  pca9539a;

    struct tc_module            tc_periodic;
    hal_interrupt_callback_t*   p_tc_periodic_overflow_callback;
}
_hal;


/*
 * Private functions
 */
void _hal_tc_periodic_overflow_callback(struct tc_module* const pModule);


/*
 * Public functions
 */
void hal_init(void)
{
    util_clear(&_hal, sizeof(_hal));

    system_init();

    /* Initialize the delay driver */
    delay_init();

    hal_cdc_init();


    {
        // Initializes the Radio Digital IO's, Reset Pins
        struct port_config cfg;

        port_get_config_defaults(&cfg);
        cfg.direction = PORT_PIN_DIR_OUTPUT;

        port_pin_set_config(BOARD_RF_SPI_MOSI_PIN,      &cfg);
        port_pin_set_output_level(BOARD_RF_SPI_MOSI_PIN,!BOARD_RF_SPI_MOSI_ACTIVE);

        port_pin_set_config(BOARD_RF_SPI_SCK_PIN,       &cfg);
        port_pin_set_output_level(BOARD_RF_SPI_SCK_PIN, !BOARD_RF_SPI_SCK_ACTIVE);

        port_pin_set_config(BOARD_RF_SPI_CS_PIN,        &cfg);
        port_pin_set_output_level(BOARD_RF_SPI_CS_PIN,  !BOARD_RF_SPI_CS_ACTIVE);

        port_pin_set_config(BOARD_RF_RESET_PIN,         &cfg);
        port_pin_set_output_level(BOARD_RF_RESET_PIN,   BOARD_RF_RESET_ACTIVE);

        port_pin_set_config(BOARD_RF_SWITCH_PIN,        &cfg);
        port_pin_set_output_level(BOARD_RF_SWITCH_PIN,  !BOARD_RF_SWITCH_ACTIVE);

        port_pin_set_config(BOARD_TCXO_PWR_PIN,         &cfg);
        port_pin_set_output_level(BOARD_TCXO_PWR_PIN,   BOARD_TCXO_PWR_ACTIVE);

        port_pin_set_config(BOARD_S0_UART_TX_PIN,       &cfg);
        port_pin_set_output_level(BOARD_S0_UART_TX_PIN, !BOARD_S0_UART_TX_ACTIVE);

        port_pin_set_config(BOARD_GPIO_OUT2_PIN,        &cfg);
        port_pin_set_output_level(BOARD_GPIO_OUT2_PIN,  !BOARD_GPIO_OUT2_ACTIVE);


        cfg.direction = PORT_PIN_DIR_INPUT;
        port_pin_set_config(BOARD_USB_VBUS_PIN,     &cfg);
        
        port_pin_set_config(BOARD_RF_SPI_MISO_PIN,  &cfg);
        port_pin_set_config(BOARD_RF_DIO0_PIN,      &cfg);
        port_pin_set_config(BOARD_RF_DIO1_PIN,      &cfg);
        port_pin_set_config(BOARD_RF_DIO2_PIN,      &cfg);
        port_pin_set_config(BOARD_RF_DIO3_PIN,      &cfg);
        port_pin_set_config(BOARD_RF_DIO4_PIN,      &cfg);

        port_pin_set_config(BOARD_SWITCH1_PIN,      &cfg);
        port_pin_set_config(BOARD_SWITCH2_PIN,      &cfg);
        port_pin_set_config(BOARD_SWITCH3_PIN,      &cfg);
        port_pin_set_config(BOARD_SWITCH4_PIN,      &cfg);
        port_pin_set_config(BOARD_BUTTON_PIN,       &cfg);

        port_pin_set_config(BOARD_GPIO_INPUT1_PIN,  &cfg);

        cfg.input_pull = PORT_PIN_PULL_UP;
        port_pin_set_config(BOARD_USB_ID_PIN,       &cfg);
    }


    // Enable TCXO supply
    hal_rf_set_tcxo(true);


    {
        // Initializes the Radio SPI Interface (From radio_driver_hal.c)
        struct spi_slave_inst_config cfg;

        spi_slave_inst_get_config_defaults(&cfg);

        cfg.ss_pin  = BOARD_RF_SPI_CS_PIN;
        spi_attach_slave(&_hal.rf_spi_dev, &cfg);
    }


    {
        struct spi_config cfg;

        spi_get_config_defaults(&cfg);

        cfg.mode_specific.master.baudrate = BOARD_RF_SPI_BAUDRATE;
        cfg.mux_setting = BOARD_RF_SPI_SERCOM_MUX_SETTING;
        cfg.pinmux_pad0 = BOARD_RF_SPI_SERCOM_PINMUX_PAD0;
        cfg.pinmux_pad1 = BOARD_RF_SPI_SERCOM_PINMUX_PAD1;
        cfg.pinmux_pad2 = BOARD_RF_SPI_SERCOM_PINMUX_PAD2;
        cfg.pinmux_pad3 = BOARD_RF_SPI_SERCOM_PINMUX_PAD3;

        spi_init(&_hal.rf_spi, BOARD_RF_SPI, &cfg);
        spi_enable(&_hal.rf_spi);
    }


    {
        struct i2c_master_config cfg;

        i2c_master_get_config_defaults(&cfg);

        cfg.buffer_timeout  = 10000;
        cfg.baud_rate       = I2C_MASTER_BAUD_RATE_400KHZ;	// Gives clean edges with 4K7 pull-ups on 3v3
        cfg.pinmux_pad0     = BOARD_I2C_SERCOM_PINMUX_PAD0;
        cfg.pinmux_pad1     = BOARD_I2C_SERCOM_PINMUX_PAD1;

        i2c_master_init(&_hal.i2c, BOARD_I2C_MODULE, &cfg);
        i2c_master_enable(&_hal.i2c);
    }


    pca9539a_init(&_hal.pca9539a, hal_i2c_rw, PCA9539A_ADDRESS);
    ioexpander_init(&_hal.pca9539a);

    ioexpander_set_direction_output(BOARD_LED_RGB_RED_PIN);
    ioexpander_set_direction_output(BOARD_LED_RGB_GREEN_PIN);
    ioexpander_set_direction_output(BOARD_LED_RGB_BLUE_PIN);
    
    ioexpander_set_direction_output(BOARD_LED_STAT_RED_PIN);
    ioexpander_set_direction_output(BOARD_LED_STAT_GREEN_PIN);

    ioexpander_set_direction_output(BOARD_PWR_ON_PIN);
    ioexpander_set_direction_output(BOARD_PWROFF_PIN);

    ioexpander_set_output_level(BOARD_LED_RGB_RED_PIN,      !BOARD_LED_RGB_RED_ACTIVE);
    ioexpander_set_output_level(BOARD_LED_RGB_GREEN_PIN,    !BOARD_LED_RGB_GREEN_ACTIVE);
    ioexpander_set_output_level(BOARD_LED_RGB_BLUE_PIN,     !BOARD_LED_RGB_BLUE_ACTIVE);
    
    ioexpander_set_output_level(BOARD_LED_STAT_RED_PIN,     !BOARD_LED_STAT_RED_ACTIVE);
    ioexpander_set_output_level(BOARD_LED_STAT_GREEN_PIN,   !BOARD_LED_STAT_GREEN_ACTIVE);

    ioexpander_set_output_level(BOARD_PWR_ON_PIN,           !BOARD_PWR_ON_ACTIVE);
    ioexpander_set_output_level(BOARD_PWROFF_PIN,           !BOARD_PWROFF_ACTIVE);

    ioexpander_sync_hardware();
}


void hal_shutdown(void)
{
#if 0   // don't have a power switch...    
    {
        // Data sheet of SX1276 says RESET pin should be left floating during
        // POR sequence
        struct port_config cfg;

        port_get_config_defaults(&cfg);
        cfg.direction = PORT_PIN_DIR_INPUT;
        port_pin_set_config(BOARD_RF_RESET_PIN, &cfg);
    }

    i2c_master_disable(&_hal.i2c);
    spi_disable(&_hal.rf_spi);
    tc_disable(&_hal.tc_periodic);
    hal_cdc_deinit();

    hal_rf_set_tcxo(false);
#endif

    ioexpander_set_output_level(BOARD_PWROFF_PIN, BOARD_PWROFF_ACTIVE);
    ioexpander_sync_hardware();

    while (1) __NOP();  // Wait to power down
}


bool hal_button_is_pressed(hal_button_t const Identifier)
{
    bool yes;

    switch (Identifier)
    {
    case hal_button_switch1:
        yes = (port_pin_get_input_level(BOARD_SWITCH1_PIN) == BOARD_SWITCH1_ACTIVE);
        break;

    case hal_button_switch2:
        yes = (port_pin_get_input_level(BOARD_SWITCH2_PIN) == BOARD_SWITCH2_ACTIVE);
        break;

    case hal_button_switch3:
        yes = (port_pin_get_input_level(BOARD_SWITCH3_PIN) == BOARD_SWITCH3_ACTIVE);
        break;

    case hal_button_switch4:
        yes = (port_pin_get_input_level(BOARD_SWITCH4_PIN) == BOARD_SWITCH4_ACTIVE);
        break;

    case hal_button_button:
        yes = (port_pin_get_input_level(BOARD_BUTTON_PIN) == BOARD_BUTTON_ACTIVE);
        break;

    default:
        yes = false;
    }

    return yes;
}


void hal_led_rgb_set(bool const RedOn, bool const GreenOn, bool const BlueOn)
{
    ioexpander_set_output_level(BOARD_LED_RGB_RED_PIN,
        RedOn ? BOARD_LED_RGB_RED_ACTIVE : !BOARD_LED_RGB_RED_ACTIVE);

    ioexpander_set_output_level(BOARD_LED_RGB_GREEN_PIN,
        GreenOn ? BOARD_LED_RGB_GREEN_ACTIVE : !BOARD_LED_RGB_GREEN_ACTIVE);

    ioexpander_set_output_level(BOARD_LED_RGB_BLUE_PIN,
        BlueOn ? BOARD_LED_RGB_BLUE_ACTIVE : !BOARD_LED_RGB_BLUE_ACTIVE);

    ioexpander_sync_hardware();
}


void hal_led_stat_set(bool const RedOn, bool const GreenOn)
{
    ioexpander_set_output_level(BOARD_LED_STAT_RED_PIN,
        RedOn ? BOARD_LED_STAT_RED_ACTIVE : !BOARD_LED_STAT_RED_ACTIVE);

    ioexpander_set_output_level(BOARD_LED_STAT_GREEN_PIN,
        GreenOn ? BOARD_LED_STAT_GREEN_ACTIVE : !BOARD_LED_STAT_GREEN_ACTIVE);

    ioexpander_sync_hardware();
}


void hal_output_set(hal_output_t const Identifier, bool const Active)
{
    bool is_on_expander;
    uint8_t pin;
    bool is_high;

    switch (Identifier)
    {
    case hal_output_power:
        is_on_expander = true;
        pin     = BOARD_PWR_ON_PIN;
        is_high = Active ? BOARD_PWR_ON_ACTIVE : !BOARD_PWR_ON_ACTIVE;
        break;

    case hal_output_discrete_1:
        is_on_expander = false;
        pin     = BOARD_S0_UART_TX_PIN;
        is_high = Active ? BOARD_S0_UART_TX_ACTIVE : !BOARD_S0_UART_TX_ACTIVE;
        break;

    case hal_output_discrete_2:
        is_on_expander = false;
        pin     = BOARD_GPIO_OUT2_PIN;
        is_high = Active ? BOARD_GPIO_OUT2_ACTIVE : !BOARD_GPIO_OUT2_ACTIVE;
        break;

    default:
        return;
    }

    if (is_on_expander)
    {
        ioexpander_set_output_level(pin, is_high);
        ioexpander_sync_hardware();
    }
    else
    {
        port_pin_set_output_level(pin, is_high);
    }
}


bool hal_output_get(hal_output_t const Identifier)
{
    bool is_on_expander;
    uint8_t pin;
    bool active_level;
    bool is_high;

    switch (Identifier)
    {
    case hal_output_power:
        is_on_expander  = true;
        pin             = BOARD_PWR_ON_PIN;
        active_level    = BOARD_PWR_ON_ACTIVE;
        break;

    case hal_output_discrete_1:
        is_on_expander  = false;
        pin             = BOARD_S0_UART_TX_PIN;
        active_level    = BOARD_S0_UART_TX_ACTIVE;
        break;

    case hal_output_discrete_2:
        is_on_expander  = false;
        pin             = BOARD_GPIO_OUT2_PIN;
        active_level    = BOARD_GPIO_OUT2_ACTIVE;
        break;

    default:
        return false;
    }

    if (is_on_expander)
    {
        ioexpander_sync_hardware();
        is_high = ioexpander_get_input_level(pin);
    }
    else
    {
        is_high = port_pin_get_output_level(pin);
    }

    return is_high == active_level;
}


bool hal_i2c_rw(unsigned const DeviceAddress,
                void* pWData, size_t const WDataSize,
                void* pRData, size_t const RDataSize)
{
    int result = STATUS_OK;

    uint16_t timeout       = 0;
    uint16_t timeout_max   = 15;

    // Common to both write and read operations
    struct i2c_master_packet packet =
    {
        .address         = (uint16_t)DeviceAddress,
        .ten_bit_address = false,
        .high_speed      = false,
        .hs_master_code  = 0x0,
    };


    if ( (pWData != NULL) && (WDataSize > 0) )  // Doing a write
    {
        // Build packet to send
        packet.data        = pWData;
        packet.data_length = WDataSize;

        // Write and send STOP bit if there's no following read
        if (RDataSize == 0)
        {
            // Write buffer to device until success.
            timeout = timeout_max;
            do
            {
                result = i2c_master_write_packet_wait(&_hal.i2c, &packet);
                if (result == STATUS_OK)
                {
                    break;
                }
            }
            while (--timeout);
        }
        else   // If read follows, then don't send STOP bit
        {
            // Write buffer to device until success.
            timeout = timeout_max;
            do
            {
                result = i2c_master_write_packet_wait_no_stop(&_hal.i2c, &packet);
                if (result == STATUS_OK)
                {
                    break;
                }
            }
            while (--timeout);
        }
    }

    // Reads might follow writes
    if ( (pRData != NULL) && (RDataSize > 0) && (result == STATUS_OK) )
    {
        // Build packet to send
        packet.data        = pRData;
        packet.data_length = RDataSize;

        // Read buffer from device until success.
        timeout = timeout_max;
        do
        {
            result = i2c_master_read_packet_wait(&_hal.i2c, &packet);
            if (result == STATUS_OK)
            {
                break;
            }
        }
        while (--timeout);
    }

    // A failed transaction may well freeze the bus, so reset the controller to recover
    if (result != STATUS_OK)
    {
        i2c_master_reset(&_hal.i2c);
    }

    return result == STATUS_OK;
}


void hal_rf_spi_start(void)
{
    spi_select_slave(&_hal.rf_spi, &_hal.rf_spi_dev, true);
}


void hal_rf_spi_end(void)
{
    spi_select_slave(&_hal.rf_spi, &_hal.rf_spi_dev, false);
}


uint8_t hal_rf_spi_transfer(uint8_t const MOSI)
{
    uint16_t miso = 0;

    while (spi_write(&_hal.rf_spi, MOSI) != STATUS_OK);
    while (spi_read(&_hal.rf_spi, &miso) == STATUS_ERR_IO);
    // spi_read has three errors: STATUS_ERR_IO, STATUS_ERR_OVERFLOW, STATUS_OK
    // STATUS_ERR_IO means not ready; others return latest data read

    return (uint8_t)miso;
}


void hal_rf_set_tcxo(bool const Active)
{
    port_pin_set_output_level(BOARD_TCXO_PWR_PIN,
        Active ? BOARD_TCXO_PWR_ACTIVE : !BOARD_TCXO_PWR_ACTIVE);
    if (Active)
    {
        delay_ms(2);    // Stabilisation time
    }
}


void hal_rf_set_switch(bool const Active)
{
    port_pin_set_output_level(BOARD_RF_SWITCH_PIN,
        Active ? BOARD_RF_SWITCH_ACTIVE : !BOARD_RF_SWITCH_ACTIVE);
}


void hal_rf_set_reset(bool const Active)
{
    port_pin_set_output_level(BOARD_RF_RESET_PIN,
        Active ? BOARD_RF_RESET_ACTIVE : !BOARD_RF_RESET_ACTIVE);
}


void hal_delay_ms(uint32_t const Time)
{
    delay_ms(Time);
}


void hal_delay_us(uint32_t const Time)
{
    delay_us(Time);
}


void hal_interrupt_set_callback(hal_interrupt_source_t const Source,
                                hal_interrupt_callback_t Callback)
{
    uint8_t channel;
    struct extint_chan_conf config;
    extint_chan_get_config_defaults(&config);

    switch (Source)
    {
    case hal_interrupt_source_switch1:
        channel                     = BOARD_SWITCH1_EIC_LINE;
        config.gpio_pin             = BOARD_SWITCH1_EIC_PIN;
        config.gpio_pin_mux         = BOARD_SWITCH1_EIC_MUX;
        config.gpio_pin_pull        = EXTINT_PULL_NONE;
        config.detection_criteria   = EXTINT_DETECT_FALLING;
        break;

    case hal_interrupt_source_switch2:
        channel                     = BOARD_SWITCH2_EIC_LINE;
        config.gpio_pin             = BOARD_SWITCH2_EIC_PIN;
        config.gpio_pin_mux         = BOARD_SWITCH2_EIC_MUX;
        config.gpio_pin_pull        = EXTINT_PULL_NONE;
        config.detection_criteria   = EXTINT_DETECT_FALLING;
        break;

    case hal_interrupt_source_switch3:
        channel                     = BOARD_SWITCH3_EIC_LINE;
        config.gpio_pin             = BOARD_SWITCH3_EIC_PIN;
        config.gpio_pin_mux         = BOARD_SWITCH3_EIC_MUX;
        config.gpio_pin_pull        = EXTINT_PULL_NONE;
        config.detection_criteria   = EXTINT_DETECT_FALLING;
        break;

    case hal_interrupt_source_switch4:
        channel                     = BOARD_SWITCH4_EIC_LINE;
        config.gpio_pin             = BOARD_SWITCH4_EIC_PIN;
        config.gpio_pin_mux         = BOARD_SWITCH4_EIC_MUX;
        config.gpio_pin_pull        = EXTINT_PULL_NONE;
        config.detection_criteria   = EXTINT_DETECT_FALLING;
        break;

    case hal_interrupt_source_button:
        channel                     = BOARD_BUTTON_EIC_LINE;
        config.gpio_pin             = BOARD_BUTTON_EIC_PIN;
        config.gpio_pin_mux         = BOARD_BUTTON_EIC_MUX;
        config.gpio_pin_pull        = EXTINT_PULL_NONE;
        config.detection_criteria   = EXTINT_DETECT_BOTH;
        break;

    case hal_interrupt_source_vbus:
        channel                     = BOARD_USB_VBUS_EIC_LINE;
        config.gpio_pin             = BOARD_USB_VBUS_EIC_PIN;
        config.gpio_pin_mux         = BOARD_USB_VBUS_EIC_MUX;
        config.gpio_pin_pull        = EXTINT_PULL_NONE;
        config.detection_criteria   = EXTINT_DETECT_BOTH;
        break;

    case hal_interrupt_source_usb_id:
        channel                     = BOARD_USB_ID_EIC_LINE;
        config.gpio_pin             = BOARD_USB_ID_EIC_PIN;
        config.gpio_pin_mux         = BOARD_USB_ID_EIC_MUX;
        config.gpio_pin_pull        = EXTINT_PULL_UP;
        config.detection_criteria   = EXTINT_DETECT_FALLING;
        break;

    case hal_interrupt_source_dio0:
        channel                     = BOARD_RF_DIO0_EIC_LINE;
        config.gpio_pin             = BOARD_RF_DIO0_EIC_PIN;
        config.gpio_pin_mux         = BOARD_RF_DIO0_EIC_MUX;
        config.gpio_pin_pull        = EXTINT_PULL_NONE;
        config.detection_criteria   = EXTINT_DETECT_RISING;
        break;

    case hal_interrupt_source_dio1:
        channel                     = BOARD_RF_DIO1_EIC_LINE;
        config.gpio_pin             = BOARD_RF_DIO1_EIC_PIN;
        config.gpio_pin_mux         = BOARD_RF_DIO1_EIC_MUX;
        config.gpio_pin_pull        = EXTINT_PULL_NONE;
        config.detection_criteria   = EXTINT_DETECT_RISING;
        break;

    case hal_interrupt_source_dio2:
        channel                     = BOARD_RF_DIO2_EIC_LINE;
        config.gpio_pin             = BOARD_RF_DIO2_EIC_PIN;
        config.gpio_pin_mux         = BOARD_RF_DIO2_EIC_MUX;
        config.gpio_pin_pull        = EXTINT_PULL_NONE;
        config.detection_criteria   = EXTINT_DETECT_RISING;
        break;

    case hal_interrupt_source_dio3:
        channel                     = BOARD_RF_DIO3_EIC_LINE;
        config.gpio_pin             = BOARD_RF_DIO3_EIC_PIN;
        config.gpio_pin_mux         = BOARD_RF_DIO3_EIC_MUX;
        config.gpio_pin_pull        = EXTINT_PULL_NONE;
        config.detection_criteria   = EXTINT_DETECT_RISING;
        break;

    case hal_interrupt_source_dio4:
        channel                     = BOARD_RF_DIO4_EIC_LINE;
        config.gpio_pin             = BOARD_RF_DIO4_EIC_PIN;
        config.gpio_pin_mux         = BOARD_RF_DIO4_EIC_MUX;
        config.gpio_pin_pull        = EXTINT_PULL_NONE;
        config.detection_criteria   = EXTINT_DETECT_RISING;
        break;

    case hal_interrupt_source_dio5:
        channel                     = BOARD_RF_DIO5_EIC_LINE;
        config.gpio_pin             = BOARD_RF_DIO5_EIC_PIN;
        config.gpio_pin_mux         = BOARD_RF_DIO5_EIC_MUX;
        config.gpio_pin_pull        = EXTINT_PULL_NONE;
        config.detection_criteria   = EXTINT_DETECT_RISING;
        break;

    default:
        return;
    }

    extint_chan_set_config(channel, &config);
    extint_register_callback(Callback, channel, EXTINT_CALLBACK_TYPE_DETECT);
}


void hal_interrupt_enable_callback(hal_interrupt_source_t const Source,
                                   bool const Enabled)
{
    uint8_t channel;

    switch (Source)
    {
    case hal_interrupt_source_switch1:
        channel = BOARD_SWITCH1_EIC_LINE;
        break;

    case hal_interrupt_source_switch2:
        channel = BOARD_SWITCH2_EIC_LINE;
        break;

    case hal_interrupt_source_switch3:
        channel = BOARD_SWITCH3_EIC_LINE;
        break;

    case hal_interrupt_source_switch4:
        channel = BOARD_SWITCH4_EIC_LINE;
        break;

    case hal_interrupt_source_button:
        channel = BOARD_BUTTON_EIC_LINE;
        break;

    case hal_interrupt_source_vbus:
        channel = BOARD_USB_VBUS_EIC_LINE;
        break;

    case hal_interrupt_source_usb_id:
        channel = BOARD_USB_ID_EIC_LINE;
        break;

    case hal_interrupt_source_dio0:
        channel = BOARD_RF_DIO0_EIC_LINE;
        break;

    case hal_interrupt_source_dio1:
        channel = BOARD_RF_DIO1_EIC_LINE;
        break;

    case hal_interrupt_source_dio2:
        channel = BOARD_RF_DIO2_EIC_LINE;
        break;

    case hal_interrupt_source_dio3:
        channel = BOARD_RF_DIO3_EIC_LINE;
        break;

    case hal_interrupt_source_dio4:
        channel = BOARD_RF_DIO4_EIC_LINE;
        break;

    case hal_interrupt_source_dio5:
        channel = BOARD_RF_DIO5_EIC_LINE;
        break;

    default:
        return;
    }

    if (Enabled)
    {
        extint_chan_enable_callback(channel, EXTINT_CALLBACK_TYPE_DETECT);
    }
    else
    {
        extint_chan_disable_callback(channel, EXTINT_CALLBACK_TYPE_DETECT);
    }
}


void hal_interrupt_enable_global(bool const Enabled)
{
    if (Enabled)
    {
        if (_hal.interrupt_level > 0)
        {
            _hal.interrupt_level--;
        }

        if (_hal.interrupt_level == 0)
        {
            Enable_global_interrupt();
        }
    }
    else
    {
        _hal.interrupt_level++;
        Disable_global_interrupt();
    }
}


void hal_wait_for_interrupt(void)
{
    __WFI();
}


bool hal_usb_vbus_is_present(void)
{
    return port_pin_get_input_level(USB_VBUS_PIN);
}


void hal_usb_stdio_enable(void)
{
    hal_cdc_start();
}


void hal_usb_stdio_disable(void)
{
    hal_cdc_deinit();
}


bool hal_usb_stdio_is_running(void)
{
    return hal_cdc_is_open();
}


void hal_periodic_set_period(uint32_t const PeriodMSec)
{
    assert(PeriodMSec > 0);
    assert(PeriodMSec < 32);

    struct tc_config cfg;

    tc_get_config_defaults(&cfg);

    /* Alter any TC configuration settings here if required */
    cfg.clock_source     = GCLK_GENERATOR_2;    // 16MHz
    cfg.counter_size     = TC_COUNTER_SIZE_16BIT;
    cfg.clock_prescaler  = TC_CLOCK_PRESCALER_DIV16;
    cfg.wave_generation  = TC_WAVE_GENERATION_MATCH_FREQ;

    /* Initialize timer with the user settings */
    tc_init(&_hal.tc_periodic, TC0, &cfg);
    tc_enable(&_hal.tc_periodic);

    uint32_t const clk_hz =  system_gclk_gen_get_hz(GCLK_GENERATOR_2) / 16;
    tc_set_top_value(&_hal.tc_periodic, PeriodMSec * clk_hz / 1000);
}


void hal_periodic_set_callback(hal_interrupt_callback_t Callback)
{
    _hal.p_tc_periodic_overflow_callback = Callback;

    tc_register_callback(&_hal.tc_periodic,
                         _hal_tc_periodic_overflow_callback,
                         TC_CALLBACK_OVERFLOW);
}


void hal_periodic_enable(bool const Enabled)
{
    if (Enabled)
    {
        tc_enable_callback(&_hal.tc_periodic, TC_CALLBACK_OVERFLOW);
    }
    else
    {
        tc_disable_callback(&_hal.tc_periodic, TC_CALLBACK_OVERFLOW);
    }
}


/*
 * Private functions
 */
void _hal_tc_periodic_overflow_callback(struct tc_module* const pModule)
{
    (void) pModule;

    if (_hal.p_tc_periodic_overflow_callback != 0)
    {
        _hal.p_tc_periodic_overflow_callback();
    }
}
