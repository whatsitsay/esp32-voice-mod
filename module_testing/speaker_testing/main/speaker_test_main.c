/*
 * Speaker test using MCP4728 DAC and 4 ohm/3 watt speaker
 */

#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

// I2C characteristics
#define DAC_I2C_ADDR        (0x60) // Default
#define DAC_I2C_PORT        (I2C_NUM_0)
#define DAC_I2C_SDA_IO      (GPIO_NUM_21)
#define DAC_I2C_SCL_IO      (GPIO_NUM_22)
#define DAC_I2C_CLK_FREQ_HZ (400000) // "Fast" clock, default is 100 kHz

// DAC addressing and characteristics
// Based off of MCP4728 datasheet
// TODO: this should really be in a separate driver to abstract away
// operation
#define DAC_VOLTAGE_MAX (3.3)
#define DAC_MAX_VAL_RAW (4095) // 12-bit DAC
#define DAC_NUM_CH (4)
#define DAC_SPEAKER_CH  (0x3) // Channel D
// General call commands
#define DAC_GENERAL_CALL_CMD (0x00)
#define DAC_RESET_CMD (0x06)
#define DAC_WAKEUP_CMD (0x09)
// Write commands
#define DAC_CMD_OFFSET (5)
#define DAC_FAST_WRITE_CMD     (0x0)

#define DAC_INPUT_WRITE_CMD    (0x2)
#define DAC_WR_FUNCTION_OFFSET (3) // Only used for input write command
#define DAC_MULTI_WR_FUNCTION  (0x0) // EEPROM *not* updated
#define DAC_SEQ_WR_FUNCTION    (0x2) // EEPROM updated
#define DAC_SINGLE_WR_FUNCTION (0x3) // EEPROM updated

#define DAC_WR_VREF_SEL        (0x4)
#define DAC_WR_GAIN_SEL        (0x6)
#define DAC_WR_PD_SEL          (0x5)

// Wave characteristics
#define WAVE_FREQ (440)

// According to datasheet, max size would be updating EEPROM for all channels
// => 3B per channel X 4 channels = 12B total
static uint8_t wr_buffer[12];
/**
 * Helper function for I2C writes to DAC
*/
void dac_i2c_wr(size_t wr_size)
{
    ESP_ERROR_CHECK(i2c_master_write_to_device(
        DAC_I2C_PORT, DAC_I2C_ADDR, wr_buffer, wr_size, portMAX_DELAY
    ));
}

void app_main(void)
{
    printf("Initializing I2C interface to quad DAC\n");

    // Set config
    i2c_config_t dac_i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = DAC_I2C_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = DAC_I2C_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = DAC_I2C_CLK_FREQ_HZ,
        .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL
    };
    i2c_param_config(DAC_I2C_PORT, &dac_i2c_conf);

    // Install driver
    ESP_ERROR_CHECK(i2c_driver_install(DAC_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    printf("I2C bus initialized, performing DAC initial setup...\n");
    // Set all channels to 0 V, with PD in normal mode and Vref enabled
    wr_buffer[0] = (DAC_INPUT_WRITE_CMD << DAC_CMD_OFFSET) |
                   (DAC_SEQ_WR_FUNCTION << DAC_WR_FUNCTION_OFFSET) |
                   (0 << 1); // Start with channel A. 0 in LSB => immediate write
    for (int i = 0; i < DAC_NUM_CH; i++) {
        // Set all bits except MSB to 0. MSB = 1 => use Vref
        wr_buffer[1 + 2*i] = 1 << 7;
        wr_buffer[2 + 2*i] = 0;
    }
    dac_i2c_wr(9); // 9 bytes due to 1B command + 2B/ch

    printf("All DAC channels set to 0V. Driving 440 Hz wave on Ch D\n");

    fflush(stdout);
    
    // Main loop
    while (1) {
        // Get timestamp in S
        // esp_timer_get_time() retrieves time in uS
        float t = esp_timer_get_time() / 1.0e6;
        // Calculate current sinusoid value, as if timer started from 0
        // Includes offset to have all values be positive
        // NOTE: should this be changed to fully use differential input?
        int16_t dac_val_raw = (DAC_MAX_VAL_RAW / 2) + (int16_t)(
            (DAC_MAX_VAL_RAW / 2) * sinf(2 * M_PI * WAVE_FREQ * t));
        // Sign extend for 12 bits
        uint16_t dac_val = (dac_val_raw & 0x7ff) | ((dac_val_raw < 0) ? 0xf8 : 0x0);
        // Write value to channel D
        wr_buffer[0] = (DAC_INPUT_WRITE_CMD << DAC_CMD_OFFSET) |
                       (DAC_MULTI_WR_FUNCTION << DAC_WR_FUNCTION_OFFSET) |
                       (DAC_SPEAKER_CH << 1); // implied 0 lSB => immediate update
        wr_buffer[1] = (1 << 8) | // Use Vref, no PD, Gx of 1x
                       ((dac_val >> 8) & 0xF); // Upper nibble
        wr_buffer[2] = dac_val & 0xFF; // Lower byte
        dac_i2c_wr(3); // Multi-write, but only need last channel (no EEPROM)
    }

    // If this point is hit, wait some time then restart
    printf("Exited main loop! Restarting ESP in 5 seconds...\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    esp_restart();
}
