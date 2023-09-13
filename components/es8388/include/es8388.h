#ifndef _ES8388_H_
#define _ES8388_H_

/**
 * @file es8388_lib.h
 * @author Gabe Kaufman
 * @brief Module for configuring and interfacing with ES8388 module
 *        on the AI ESP32 Audio Kit. Based on code from the repo
 *        thaaraak/ESP32-ES8388
 * @date 2023-09-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <inttypes.h>
#include "codec_es8388.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"

#define ACK_CHECK_EN 0x1
#define ACK_CHECK_DIS 0x0
#define ACK_VAL I2C_MASTER_ACK
#define NACK_VAL I2C_MASTER_NACK

#define VOL_DEFAULT (90)


#define SAMPLE_RATE     (44100)
#define ES_I2S_NUM         (0)

#define ES_I2S_BCK_IO    (GPIO_NUM_27)
#define ES_I2S_WS_IO     (GPIO_NUM_25)
#define ES_I2S_DO_IO     (GPIO_NUM_26)
#define ES_I2S_DI_IO     (GPIO_NUM_35)
#define ES_I2S_MCLK_PIN	(GPIO_NUM_0)

#define I2C_MASTER_NUM (I2C_NUM_0)
#define I2C_MASTER_SDA_IO (GPIO_NUM_33)
#define I2C_MASTER_SCL_IO (GPIO_NUM_32)
#define ES8388_ADDR (0x10)

#define HEADPHONE_DETECT_GPIO (GPIO_NUM_39)
#define POWER_AMP_EN_GPIO (GPIO_NUM_21)

/*
#define ES_I2S_BCK_IO      (GPIO_NUM_5)
#define ES_I2S_WS_IO       (GPIO_NUM_25)
#define ES_I2S_DO_IO       (GPIO_NUM_26)
#define ES_I2S_DI_IO       (GPIO_NUM_35)
#define I2C_MASTER_SDA_IO 18
#define I2C_MASTER_SCL_IO 23
#define IS2_MCLK_PIN	(GPIO_NUM_0)
*/

uint8_t es_i2c_write_bulk( uint8_t i2c_bus_addr, uint8_t reg, uint8_t bytes, uint8_t *data);

uint8_t es_i2c_write( uint8_t i2c_bus_addr, uint8_t reg, uint8_t value);

uint8_t es_i2c_read( uint8_t i2c_bus_addr, uint8_t reg);

esp_err_t es_i2c_master_init(void);

esp_err_t es_write_reg(uint8_t slave_addr, uint8_t reg_add, uint8_t data);

esp_err_t es_read_reg(uint8_t reg_add, uint8_t *p_data);

void es8388_read_all();

int es8388_set_adc_dac_volume(int mode, int volume, int dot);

esp_err_t es8388_init( es_dac_output_t output, es_adc_input_t input );

// This function sets the I2S format which can be one of
//		ES_I2S_NORMAL
//		ES_I2S_LEFT		Left Justified
//		ES_I2S_RIGHT,      Right Justified
//		ES_I2S_DSP,        dsp/pcm format
//
// and the bits per sample which must be one of
//		BIT_LENGTH_16BITS
//		BIT_LENGTH_18BITS
//		BIT_LENGTH_20BITS
//		BIT_LENGTH_24BITS
//		BIT_LENGTH_32BITS
//
// Note the above must match the ESP-IDF I2S configuration which is set separately

esp_err_t es8388_config_i2s( es_bits_length_t bits_length, es_module_t mode, es_format_t fmt );

esp_err_t es8388_set_voice_mute(bool enable);

esp_err_t es8388_start(es_module_t mode);

esp_err_t es8388_set_voice_volume(int volume);

void es8388_config();

esp_err_t es_i2s_mclk_gpio_select(i2s_port_t i2s_num, gpio_num_t gpio_num);

esp_err_t es_toggle_power_amp();

void es_i2s_init();

#endif // _ES8388_H_