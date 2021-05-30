/* I2S Example

    This example code will output 100Hz sine wave and triangle wave to 2-channel of I2S driver
    Every 5 seconds, it will change bits_per_sample [16, 24, 32] for i2s data

    This example code is in the Public Domain (or CC0 licensed, at your option.)

    Unless required by applicable law or agreed to in writing, this
    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
    CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include <math.h>
#include "soc/gpio_sig_map.h"
#include "soc/io_mux_reg.h"

#define SAMPLE_RATE     (44100)
#define I2S_NUM         (0)
#define WAVE_FREQ_HZ    (1700)
#define PI              (3.14159265)
#define I2S_BCK_IO      (GPIO_NUM_5)
#define I2S_WS_IO       (GPIO_NUM_18)
#define I2S_DO_IO       (GPIO_NUM_19)
#define I2S_DI_IO       (-1)

#define SAMPLE_PER_CYCLE (SAMPLE_RATE/WAVE_FREQ_HZ)

#define SAMPLES			SAMPLE_PER_CYCLE	// Total number of samples left and right
#define	BUF_SAMPLES		SAMPLES * 4			// Size of DMA tx/rx buffer samples * left/right * 2 for 32 bit samples

// DMA Buffers
uint16_t rxBuf[BUF_SAMPLES];
uint16_t txBuf[BUF_SAMPLES];



static void setup_sine_waves16()
{
	double sin_float;

    size_t i2s_bytes_write = 0;

    printf("\r\nFree mem=%d, written data=%d\n", esp_get_free_heap_size(), BUF_SAMPLES*2 );

    for( int pos = 0; pos < BUF_SAMPLES; pos += 2 )
    {
        sin_float = 10000 * sin( pos/2 * 2 * PI / SAMPLE_PER_CYCLE);

        int lval = sin_float;
        int rval = sin_float;

        txBuf[pos] = lval&0xFFFF;
        txBuf[pos+1] = 0;

        printf( "%d  %04x:%04x\n", lval, txBuf[pos],txBuf[pos+1] );

    }

    //i2s_write(I2S_NUM, txBuf, BUF_SAMPLES*2, &i2s_bytes_write, -1);
    printf ( "Written: %d\n", i2s_bytes_write );
}

static void setup_sine_waves24( int amplitude )
{
	double sin_float;

    size_t i2s_bytes_write = 0;

    //printf("\r\nFree mem=%d, written data=%d\n", esp_get_free_heap_size(), BUF_SAMPLES*2 );

    for( int pos = 0; pos < BUF_SAMPLES; pos += 4 )
    {
        sin_float = amplitude * sin( pos/4 * 2 * PI / SAMPLE_PER_CYCLE);

        int lval = sin_float;
        int rval = sin_float;

        lval = lval << 8;

        txBuf[pos+1] = (lval>>16)&0xFFFF;
        txBuf[pos] = lval&0xFFFF;
        txBuf[pos+2] = 0;
        txBuf[pos+3] = 0;

        //printf( "%d  %04x:%04x:%04x:%04x\n", lval, txBuf[pos],txBuf[pos+1],txBuf[pos+2],txBuf[pos+3] );

    }

}

esp_err_t i2s_mclk_gpio_select(i2s_port_t i2s_num, gpio_num_t gpio_num)
{
    if (i2s_num >= I2S_NUM_MAX) {
        printf( "Does not support i2s number(%d)", i2s_num);
        return ESP_ERR_INVALID_ARG;
    }
    if (gpio_num != GPIO_NUM_0 && gpio_num != GPIO_NUM_1 && gpio_num != GPIO_NUM_3) {
        printf("Only support GPIO0/GPIO1/GPIO3, gpio_num:%d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    printf("I2S%d, MCLK output by GPIO%d", i2s_num, gpio_num);
    if (i2s_num == I2S_NUM_0) {
        if (gpio_num == GPIO_NUM_0) {
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
            WRITE_PERI_REG(PIN_CTRL, 0xFFF0);
        } else if (gpio_num == GPIO_NUM_1) {
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD_CLK_OUT3);
            WRITE_PERI_REG(PIN_CTRL, 0xF0F0);
        } else {
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD_CLK_OUT2);
            WRITE_PERI_REG(PIN_CTRL, 0xFF00);
        }
    } else if (i2s_num == I2S_NUM_1) {
        if (gpio_num == GPIO_NUM_0) {
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
            WRITE_PERI_REG(PIN_CTRL, 0xFFFF);
        } else if (gpio_num == GPIO_NUM_1) {
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD_CLK_OUT3);
            WRITE_PERI_REG(PIN_CTRL, 0xF0FF);
        } else {
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD_CLK_OUT2);
            WRITE_PERI_REG(PIN_CTRL, 0xFF0F);
        }
    }
    return ESP_OK;
}



void app_main(void)
{
    //for 36Khz sample rates, we create 100Hz sine wave, every cycle need 36000/100 = 360 samples (4-bytes or 8-bytes each sample)
    //depend on bits_per_sample
    //using 6 buffers, we need 60-samples per buffer
    //if 2-channels, 16-bit each channel, total buffer is 360*4 = 1440 bytes
    //if 2-channels, 24/32-bit each channel, total buffer is 360*8 = 2880 bytes
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,                                  // Only TX
        .sample_rate = SAMPLE_RATE,
//        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,                           //2-channels
        .communication_format = I2S_COMM_FORMAT_STAND_MSB,
        .dma_buf_count = 3,
        .dma_buf_len = 300,
        .use_apll = true,
	    .tx_desc_auto_clear = true,
	    .fixed_mclk = 0,
        .intr_alloc_flags = 0                                //Interrupt level 1
    };
    /*


    .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX,                    \
    .sample_rate = 44100,                                                   \
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,                           \
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,                           \
    .communication_format = I2S_COMM_FORMAT_I2S,                            \
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2 | ESP_INTR_FLAG_IRAM,          \
    .dma_buf_count = 3,                                                     \
    .dma_buf_len = 300,                                                     \
    .use_apll = true,                                                       \
    .tx_desc_auto_clear = true,                                             \
    .fixed_mclk = 0

	*/
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_DO_IO,
        .data_in_num = I2S_DI_IO                                               //Not used
    };
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);
    i2s_mclk_gpio_select(I2S_NUM, (gpio_num_t)GPIO_NUM_3 );

    //i2s_set_clk(I2S_NUM, SAMPLE_RATE, 16, 2);
    i2s_set_clk(I2S_NUM, SAMPLE_RATE, 32, 2);

    size_t i2s_bytes_write = 0;

    int cnt = 0;
    int amplitude = 1000000;

    setup_sine_waves24( amplitude );
    int vol = 50000;

    while (1) {

    	if ( cnt++ > 10 )
    	{
    		cnt = 0;
    		amplitude += vol;
    		if ( amplitude > 3000000 || amplitude < 1000000 )
    			vol = -vol;

    		printf( "Volume: %d\n", amplitude );
    	    setup_sine_waves24( amplitude );
    	}

    	//i2s_write(I2S_NUM, txBuf, BUF_SAMPLES*2, &i2s_bytes_write, -1);
    	i2s_write(I2S_NUM, txBuf, BUF_SAMPLES*2, &i2s_bytes_write, -1);
    	//vTaskDelay(10/portTICK_RATE_MS);

    }

}
