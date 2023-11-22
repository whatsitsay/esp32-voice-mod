#include <string.h>
#include "include/es8388.h"
#include "include/codec_es8388.h"
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "soc/gpio_sig_map.h"
#include "math.h"

static const char *ES_TAG = "ES8388_DRIVER";
#define DAC_VOLUME_DEFAULT_DB (0)
#define ADC_VOLUME_DEFAULT_DB (-30)

uint8_t es_i2c_write_bulk( uint8_t i2c_bus_addr, uint8_t reg, uint8_t bytes, uint8_t *data)
{

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2c_bus_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(cmd, &reg, 1, ACK_CHECK_EN);
    i2c_master_write(cmd, data, bytes, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin( I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }

    return 0;
}

uint8_t es_i2c_write( uint8_t i2c_bus_addr, uint8_t reg, uint8_t value)
{
    return es_i2c_write_bulk( i2c_bus_addr, reg, 1, &value );
}



uint8_t es_i2c_read( uint8_t i2c_bus_addr, uint8_t reg)
{
	   	uint8_t buffer[2];
	    //ESP_LOGI(ES_TAG,  "Addr: [%d] Reading register: [%d]\n", i2c_bus_addr, reg );

	    buffer[0] = reg;

	    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	    // Write the register address to be read
	    i2c_master_start(cmd);
	    i2c_master_write_byte(cmd, i2c_bus_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
	    i2c_master_write_byte(cmd, buffer[0], ACK_CHECK_EN);

	    // Read the data for the register from the slave
	    i2c_master_start(cmd);
	    i2c_master_write_byte(cmd, i2c_bus_addr << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
	    i2c_master_read_byte(cmd, &buffer[0], NACK_VAL);
	    i2c_master_stop(cmd);

	    ESP_ERROR_CHECK(i2c_master_cmd_begin( I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS));
	    i2c_cmd_link_delete(cmd);

	    //ESP_LOGI(ES_TAG,  "Read: [%02x]=[%02x]\n", reg, buffer[0] );

	    return (buffer[0]);
}


esp_err_t es_i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    memset(&conf, 0, sizeof(i2c_config_t));
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000; // FIXME Should be macro, not magic num

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

esp_err_t es_write_reg(uint8_t slave_addr, uint8_t reg_add, uint8_t data)
{
    return es_i2c_write( ES8388_ADDR, reg_add, data );
}

esp_err_t es_read_reg(uint8_t reg_add, uint8_t *p_data)
{
    *p_data = es_i2c_read( ES8388_ADDR, reg_add );
    return ESP_OK;
}

void es8388_read_all()
{
	//ESP_LOGI(ES_TAG,  "\n\n===================\n\n");
    for (int i = 0; i < 50; i++) {
        uint8_t reg = 0;
        es_read_reg(i, &reg);
    }
	//ESP_LOGI(ES_TAG,  "\n\n===================\n\n");
}



int es8388_set_adc_dac_volume(int mode, int volume, int dot)
{
    int res = 0;
    if ( volume < -96 || volume > 0 ) {
        ESP_LOGW(ES_TAG, "Warning: volume < -96! or > 0!\n");
        if (volume < -96)
            volume = -96;
        else
            volume = 0;
    }
    dot = (dot >= 5 ? 1 : 0);
    volume = (-volume << 1) + dot;
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL8, volume);
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL9, volume);  //ADC Right Volume=0db
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL5, volume);
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL4, volume);
    }
    return res;
}

esp_err_t es8388_init( es_dac_output_t output, es_adc_input_t input )
{
    int res = 0;

    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x04);  // 0x04 mute/0x00 unmute&ramp;DAC unmute and  disabled digital volume control soft ramp

    res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x50);
    res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00); //normal all and power up all
    res |= es_write_reg(ES8388_ADDR, ES8388_MASTERMODE, ES_MODE_SLAVE ); //CODEC IN I2S SLAVE MODE

    res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0xC0);  //disable DAC and disable Lout/Rout/1/2
    res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x32);  //SameFs, Enfr=0,Play&Record Mode,(0x17-both of mic&paly)
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, 0x22); // 32-bit audio, left justify
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL2, 0x02);  //DACFsMode,SINGLE SPEED; DACFsRatio,256
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL6, 0x08);  // Clickfree disable
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x00); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x90); // only left DAC to left mixer enable 0db
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x90); // only right DAC to right mixer enable 0db
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80); //set internal ADC and DAC use the same LRCK clock, ADC LRCK as internal LRCK
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL23, 0x00);   //vroi=0
    res |= es8388_set_adc_dac_volume(ES_MODULE_DAC, DAC_VOLUME_DEFAULT_DB, 0);

    ESP_LOGW(ES_TAG, "Setting DAC Output: %02x", output );
    res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, output );
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0xFF);
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL1, 0xbb); // MIC Left and Right channel PGA gain


    ESP_LOGW(ES_TAG, "Setting ADC Input: %02x", input );
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL2, input);

    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL3, 0x08); // Mono-mix to ADC left, use LINPUT2
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, 0x11); // 32-bit audio, left justify 
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL5, 0x02);  //ADCFsMode,singel SPEED,RATIO=256
    //ALC for Microphone
    res |= es8388_set_adc_dac_volume(ES_MODULE_ADC, ADC_VOLUME_DEFAULT_DB, 0); 
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x00); // Power up all bits, disable low-power mode

    return res;
}

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

esp_err_t es8388_config_i2s( es_bits_length_t bits_length, es_module_t mode, es_format_t fmt )
{
    esp_err_t res = ESP_OK;
    uint8_t reg = 0;

    // Set the Format
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        ESP_LOGW(ES_TAG,  "Setting I2S ADC Format %d", fmt);
        res = es_read_reg(ES8388_ADCCONTROL4, &reg);
        reg = reg & 0xfc;
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, reg | fmt);
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        ESP_LOGW(ES_TAG,  "Setting I2S DAC Format: %d", fmt);
        res = es_read_reg(ES8388_DACCONTROL1, &reg);
        reg = reg & 0xf9;
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, reg | (fmt << 1));
    }


    // Set the Sample bits length
    int bits = (int)bits_length;
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        ESP_LOGW(ES_TAG,  "Setting I2S ADC Bits: %d", bits);
        res = es_read_reg(ES8388_ADCCONTROL4, &reg);
        reg = reg & 0xe3;
        res |=  es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, reg | (bits << 2));
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        ESP_LOGW(ES_TAG, "Setting I2S DAC Bits: %d", bits);
        res = es_read_reg(ES8388_DACCONTROL1, &reg);
        reg = reg & 0xc7;
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, reg | (bits << 3));
    }
    return res;
}


esp_err_t es8388_set_voice_mute(bool enable)
{
    esp_err_t res = ESP_OK;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_DACCONTROL3, &reg);
    reg = reg & 0xFB;
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, reg | (((int)enable) << 2));
    return res;
}

esp_err_t es8388_start(es_module_t mode)
{
    esp_err_t res = ESP_OK;
    uint8_t prev_data = 0, data = 0;
    es_read_reg(ES8388_DACCONTROL21, &prev_data);
    if (mode == ES_MODULE_LINE) {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x09); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2 by pass enable
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x50); // left DAC to left mixer enable  and  LIN signal to left mixer enable 0db  : bupass enable
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x50); // right DAC to right mixer enable  and  LIN signal to right mixer enable 0db : bupass enable
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0xC0); //enable adc
    } else {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80);   //enable dac
    }
    es_read_reg(ES8388_DACCONTROL21, &data);
    if (prev_data != data) {
    	ESP_LOGI(ES_TAG,  "Resetting State Machine\n");

        res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0xF0);   //start state machine
        // res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x16);
        // res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x50);
        res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00);   //start state machine
    }
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC || mode == ES_MODULE_LINE) {
    	ESP_LOGI(ES_TAG,  "Powering up ADC\n");
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x00);   //power up adc and line in
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC || mode == ES_MODULE_LINE) {
    	ESP_LOGI(ES_TAG,  "Powering up DAC\n");
        res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x3c);   //power up dac and line out
        res |= es8388_set_voice_mute(false);
    }

    return res;
}


esp_err_t es8388_set_voice_volume(int volume)
{
    esp_err_t res = ESP_OK;
    if (volume < 0)
        volume = 0;
    else if (volume > 100)
        volume = 100;
    volume /= 3;
    res = es_write_reg(ES8388_ADDR, ES8388_DACCONTROL24, volume);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL25, volume);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL26, volume);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL27, volume);
    return res;
}


void es8388_config()
{
    // Instantiate I2C master first
    ESP_ERROR_CHECK(es_i2c_master_init());
    // Input/Output Modes
    //
    //	es_dac_output_t output = DAC_OUTPUT_LOUT1 | DAC_OUTPUT_LOUT2 | DAC_OUTPUT_ROUT1 | DAC_OUTPUT_ROUT2;
    //	es_adc_input_t input = ADC_INPUT_LINPUT1_RINPUT1;
    // 	es_adc_input_t input = ADC_INPUT_LINPUT2_RINPUT2;

    es_dac_output_t output = DAC_OUTPUT_LOUT1 | DAC_OUTPUT_LOUT2 | DAC_OUTPUT_ROUT1 | DAC_OUTPUT_ROUT2;
	//es_dac_output_t output = DAC_OUTPUT_LOUT1  | DAC_OUTPUT_ROUT1;
	//es_dac_output_t output = DAC_OUTPUT_LOUT2  | DAC_OUTPUT_ROUT2;

    //es_dac_output_t output = 0;
	es_adc_input_t input = ADC_INPUT_LINPUT2_RINPUT2 | ADC_INPUT_MIC1 | ADC_INPUT_MIC2;


    es8388_init( output, input );

    // Modes Available
    //
    //	es_mode_t  = ES_MODULE_ADC;
    //	es_mode_t  = ES_MODULE_LINE;
    //	es_mode_t  = ES_MODULE_DAC;
    //	es_mode_t  = ES_MODULE_ADC_DAC;

    es_bits_length_t bits_length = BIT_LENGTH_32BITS;
#ifdef INMP441_MIC
    es_module_t module = ES_MODULE_DAC; // Use INMP441 for microphone instead of built-in mics
#else
    es_module_t module = ES_MODULE_ADC_DAC;
#endif
    es_format_t fmt = ES_I2S_LEFT;

    es8388_config_i2s( bits_length, module, fmt );
    es8388_set_voice_volume( VOL_DEFAULT );

    // Setup pins for headphone detect, power amp
    ESP_ERROR_CHECK(gpio_set_direction(HEADPHONE_DETECT_GPIO, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(HEADPHONE_DETECT_GPIO, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_set_direction(POWER_AMP_EN_GPIO, GPIO_MODE_OUTPUT));
    es_toggle_power_amp(); // Will be set based on headphone detect

    es8388_start( module );

}

esp_err_t es_toggle_power_amp()
{
    // FIXME This should really be in some sort of Audio Kit lib, not here
    // should do for now however
    bool headphone_detect = !gpio_get_level(HEADPHONE_DETECT_GPIO);
    // Power amp should be set opposite to headphone detect
    return es_set_power_amp(!headphone_detect);
}

esp_err_t es_set_power_amp(bool en)
{
    return gpio_set_level(POWER_AMP_EN_GPIO, (int)en);
}

void es_i2s_init(i2s_chan_handle_t* tx_handle, i2s_chan_handle_t* rx_handle, int i2s_sample_rate)
{
    i2s_chan_config_t i2s_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_chan_cfg.dma_desc_num  = 3;
    i2s_chan_cfg.dma_frame_num = 511; // Same as buffer length?
#ifdef INMP441_MIC
    i2s_new_channel(&i2s_chan_cfg, tx_handle, NULL);
#else
    // Use internal mics
    i2s_new_channel(&i2s_chan_cfg, tx_handle, rx_handle);
#endif

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(i2s_sample_rate),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = ES_I2S_MCLK_PIN,
            .bclk = ES_I2S_BCK_IO,
            .ws   = ES_I2S_WS_IO,
            .dout = ES_I2S_DO_IO,
            .din  = ES_I2S_DI_IO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            }
        }
    };
    i2s_channel_init_std_mode(*tx_handle, &std_cfg);
#ifndef INMP441_MIC
    i2s_channel_init_std_mode(*rx_handle, &std_cfg);
#endif

#ifdef INMP441_MIC
    // Initialize INMP441 microphone as RX channel
    // Set L/R low to mimic ground (set channel as "L")
    gpio_set_direction(GPIO_NUM_19, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_19, 0);
    // Set I2S config
    i2s_chan_config_t mic_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    i2s_chan_cfg.dma_desc_num  = 3;
    i2s_chan_cfg.dma_frame_num = 511; // Same as buffer length?
    i2s_new_channel(&mic_chan_cfg, NULL, rx_handle);
    i2s_std_config_t mic_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(i2s_sample_rate),
        // Use Philips as there is one clock cycle delay before data clocks out
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_NUM_22,
            .ws   = GPIO_NUM_21,
            .dout = I2S_GPIO_UNUSED,
            .din  = GPIO_NUM_23,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            }
        }
    };
    i2s_channel_init_std_mode(*rx_handle, &mic_cfg);
#endif
}