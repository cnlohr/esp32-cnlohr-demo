//Almost entirelly lifted directly from https://github.com/igrr/esp32-cam-demo
//Just clocked a little differently and has chained buffers.
//This totes works with the I2S bus on the ESP32 for READING 16 wires simultaneously.
//Can be clocked off of I2S's internal controller or an external clock.

#define I2S_D0 4
#define I2S_D1 5
#define I2S_D2 18
#define I2S_D3 19
#define I2S_D4 36
#define I2S_D5 39
#define I2S_D6 34
#define I2S_D7 35

#define I2S_D8 32
#define I2S_D9 33
#define I2S_D10 25
#define I2S_D11 26
#define I2S_D12 27
#define I2S_D13 14
#define I2S_D14 12
#define I2S_D15 13

#define I2S_CLK 22
//#define I2S_CLKO 13
//#define I2S_CLKOWS 2

#define I2S_HZ 10000000
//#define I2S_HZ 1000000



#include "esp_intr.h"
#include "i2s_stream_in.h"
#include "driver/gpio.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/io_mux_reg.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "rom/lldesc.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include <stdlib.h>

#define ETS_I2S0_INUM 13

static void IRAM_ATTR i2s_isr(void* arg);
static esp_err_t dma_desc_init();
static void i2s_init();
static void i2s_run();
static void enable_out_clock();


uint16_t * i2sbufferIn[2] __attribute__((aligned(128)));
static lldesc_t s_dma_desc[2];
static int i2s_running;
volatile unsigned isr_countIn;


void SetupI2SIn()
{
	enable_out_clock();
    i2s_init();
	dma_desc_init();
	i2s_run();
}


static esp_err_t dma_desc_init()
{
    for (int i = 0; i < 2; ++i) {
		i2sbufferIn[i] = malloc( BUFF_SIZE_BYTES );
        s_dma_desc[i].length = BUFF_SIZE_BYTES;     // size of a single DMA buf
        s_dma_desc[i].size = BUFF_SIZE_BYTES;       // total size of the chain
        s_dma_desc[i].owner = 1;
        s_dma_desc[i].sosf = 1;
        s_dma_desc[i].buf = (uint8_t*) i2sbufferIn[i];
        s_dma_desc[i].offset = i;
        s_dma_desc[i].empty = 0;
        s_dma_desc[i].eof = 1;
        s_dma_desc[i].qe.stqe_next = &s_dma_desc[(i+1)&1];
    }
    return ESP_OK;
}


static void enable_out_clock() {
    periph_module_enable(PERIPH_LEDC_MODULE);

    ledc_timer_config_t timer_conf;
    timer_conf.bit_num = 3;
    timer_conf.freq_hz = I2S_HZ;
    timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    timer_conf.timer_num = LEDC_TIMER_0;
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        //ESP_LOGE(TAG, "ledc_timer_config failed, rc=%x", err);
    }

    ledc_channel_config_t ch_conf;
    ch_conf.channel = LEDC_CHANNEL_0;
    ch_conf.timer_sel = LEDC_TIMER_0;
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    ch_conf.duty = 4;
    ch_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    ch_conf.gpio_num = 17; //s_config.pin_xclk; 
    err = ledc_channel_config(&ch_conf);
    if (err != ESP_OK) {
        //ESP_LOGE(TAG, "ledc_channel_config failed, rc=%x", err);
    }

}


static void i2s_init()
{
    xt_set_interrupt_handler(ETS_I2S0_INUM, &i2s_isr, NULL);
    intr_matrix_set(0, ETS_I2S0_INTR_SOURCE, ETS_I2S0_INUM);

    gpio_num_t pins[] = {
            I2S_D15,
            I2S_D14,
            I2S_D13,
            I2S_D12,
            I2S_D11,
            I2S_D10,
            I2S_D9,
            I2S_D8,

            I2S_D7,
            I2S_D6,
            I2S_D5,
            I2S_D4,
            I2S_D3,
            I2S_D2,
            I2S_D1,
            I2S_D0,
			I2S_CLK,
    };

    gpio_config_t conf = {
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
    };
    for (int i = 0; i < sizeof(pins)/sizeof(gpio_num_t); ++i) {
        conf.pin_bit_mask = 1LL << pins[i];
        gpio_config(&conf);
    }

    gpio_matrix_in(I2S_D0,    I2S0I_DATA_IN0_IDX, false);
    gpio_matrix_in(I2S_D1,    I2S0I_DATA_IN1_IDX, false);
    gpio_matrix_in(I2S_D2,    I2S0I_DATA_IN2_IDX, false);
    gpio_matrix_in(I2S_D3,    I2S0I_DATA_IN3_IDX, false);
    gpio_matrix_in(I2S_D4,    I2S0I_DATA_IN4_IDX, false);
    gpio_matrix_in(I2S_D5,    I2S0I_DATA_IN5_IDX, false);
    gpio_matrix_in(I2S_D6,    I2S0I_DATA_IN6_IDX, false);
    gpio_matrix_in(I2S_D7,    I2S0I_DATA_IN7_IDX, false);

    gpio_matrix_in(I2S_D8,    I2S0I_DATA_IN8_IDX, false);
    gpio_matrix_in(I2S_D9,    I2S0I_DATA_IN9_IDX, false);
    gpio_matrix_in(I2S_D10,    I2S0I_DATA_IN10_IDX, false);
    gpio_matrix_in(I2S_D11,    I2S0I_DATA_IN11_IDX, false);
    gpio_matrix_in(I2S_D12,    I2S0I_DATA_IN12_IDX, false);
    gpio_matrix_in(I2S_D13,    I2S0I_DATA_IN13_IDX, false);
    gpio_matrix_in(I2S_D14,    I2S0I_DATA_IN14_IDX, false);
    gpio_matrix_in(I2S_D15,    I2S0I_DATA_IN15_IDX, false);

	//This magic is needed to enable the bus!
    gpio_matrix_in(0x38,    I2S0I_V_SYNC_IDX, false);
    gpio_matrix_in(0x38, 	I2S0I_H_SYNC_IDX, false);
	gpio_matrix_in(0x38,    I2S0I_H_ENABLE_IDX, false);

//Whether the bus is self-clocking, or accepting an external clock.
#define SLAVE_MOD 0

#if SLAVE_MOD
    gpio_matrix_in(I2S_CLK,  I2S0I_WS_IN_IDX, false);

//    gpio_matrix_in(I2S_CLKO,  I2S0O_BCK_OUT_IDX, false);
//    gpio_matrix_in(I2S_CLKOWS,  I2S0O_WS_OUT_IDX, false);

#else

	conf.mode = GPIO_MODE_OUTPUT;
    conf.pin_bit_mask = 1LL << I2S_CLK;
    gpio_config(&conf);
    gpio_matrix_out(I2S_CLK,  I2S0I_BCK_OUT_IDX, false, 0 );

#endif
    periph_module_enable(PERIPH_I2S0_MODULE);



    SET_PERI_REG_BITS(I2S_LC_CONF_REG(0), 0x1, 1, I2S_IN_RST_S);
    SET_PERI_REG_BITS(I2S_LC_CONF_REG(0), 0x1, 0, I2S_IN_RST_S);
    SET_PERI_REG_BITS(I2S_LC_CONF_REG(0), 0x1, 1, I2S_AHBM_RST_S);
    SET_PERI_REG_BITS(I2S_LC_CONF_REG(0), 0x1, 0, I2S_AHBM_RST_S);
    SET_PERI_REG_BITS(I2S_LC_CONF_REG(0), 0x1, 1, I2S_AHBM_FIFO_RST_S);
    SET_PERI_REG_BITS(I2S_LC_CONF_REG(0), 0x1, 0, I2S_AHBM_FIFO_RST_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 1, I2S_RX_RESET_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 0, I2S_RX_RESET_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 1, I2S_RX_FIFO_RESET_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 0, I2S_RX_FIFO_RESET_S);


#if SLAVE_MOD
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 1, I2S_RX_SLAVE_MOD_S);
#else
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 0, I2S_RX_SLAVE_MOD_S);
#endif
    SET_PERI_REG_BITS(I2S_CONF2_REG(0), 0x1, 1, I2S_LCD_EN_S);
    SET_PERI_REG_BITS(I2S_CONF2_REG(0), 0x1, 1, I2S_CAMERA_EN_S);

    SET_PERI_REG_BITS(I2S_CLKM_CONF_REG(0), I2S_CLKM_DIV_A, 0, I2S_CLKM_DIV_A_S);
    SET_PERI_REG_BITS(I2S_CLKM_CONF_REG(0), I2S_CLKM_DIV_B, 0, I2S_CLKM_DIV_B_S);
    SET_PERI_REG_BITS(I2S_CLKM_CONF_REG(0), I2S_CLKM_DIV_NUM, 1, I2S_CLKM_DIV_NUM_S);  //Setting to 0 wrecks it up.



//This seems ignored in slave mode.
//	WRITE_PERI_REG( I2S_TIMING_REG(0), 0xffffffff );
//	WRITE_PERI_REG( I2S_CONF_SIGLE_DATA_REG(0), 0xffffffff );
//    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 1, I2S_RX_SHORT_SYNC_S); //
//	SET_PERI_REG_BITS(I2S_SAMPLE_RATE_CONF_REG(0), I2S_RX_BCK_DIV_NUM, 1, I2S_RX_BCK_DIV_NUM_S);


    SET_PERI_REG_BITS(I2S_FIFO_CONF_REG(0), 0x1, 1, I2S_DSCR_EN_S);

    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 0, I2S_RX_RIGHT_FIRST_S);	//Seem ignored in parallel mode
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 0, I2S_RX_MSB_RIGHT_S);//Seem ignored in parallel mode
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 0, I2S_RX_MSB_SHIFT_S);//Seem ignored in parallel mode
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 1, I2S_RX_MONO_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 1, I2S_RX_SHORT_SYNC_S); //Seem ignored in parallel mode
    SET_PERI_REG_BITS(I2S_FIFO_CONF_REG(0), I2S_RX_FIFO_MOD, 1, I2S_RX_FIFO_MOD_S);
    SET_PERI_REG_BITS(I2S_FIFO_CONF_REG(0), 0x1, 1, I2S_RX_FIFO_MOD_FORCE_EN_S);
    SET_PERI_REG_BITS(I2S_CONF_CHAN_REG(0), I2S_RX_CHAN_MOD, 1, I2S_RX_CHAN_MOD_S);
	SET_PERI_REG_BITS(I2S_SAMPLE_RATE_CONF_REG(0), I2S_RX_BITS_MOD, 8, I2S_RX_BITS_MOD_S);

	//If you don't do this, BCK will be limited to 13.3333 MHz.

	SET_PERI_REG_BITS(I2S_SAMPLE_RATE_CONF_REG(0), I2S_RX_BCK_DIV_NUM, 1, I2S_RX_BCK_DIV_NUM_S);  
	//Once set, 1 = you can read all 8 bits at 40 MHz.
	//Once set, 2 = you can read all 8 bits at 20 MHz.
	//			3 = 13.33333 MHz
	//			4 = 10.0MHz


	//When self-clocked, seem to make BCK rate ~13.3333 MHz.
	

}

static void i2s_fill_buf() {
    ESP_INTR_DISABLE(ETS_I2S0_INUM);

    SET_PERI_REG_BITS(I2S_RXEOF_NUM_REG(0), I2S_RX_EOF_NUM, (BUFF_SIZE_BYTES - 8) / 2, I2S_RX_EOF_NUM_S);
    SET_PERI_REG_BITS(I2S_IN_LINK_REG(0), I2S_INLINK_ADDR, ((uint32_t) &s_dma_desc), I2S_INLINK_ADDR_S);
    SET_PERI_REG_BITS(I2S_IN_LINK_REG(0), 0x1, 1, I2S_INLINK_START_S);

    REG_WRITE(I2S_INT_CLR_REG(0), (REG_READ(I2S_INT_RAW_REG(0)) & 0xffffffc0) | 0x3f);

    REG_WRITE(I2S_CONF_REG(0), REG_READ(I2S_CONF_REG(0)) & 0xfffffff0);
    (void) REG_READ(I2S_CONF_REG(0));
    REG_WRITE(I2S_CONF_REG(0), (REG_READ(I2S_CONF_REG(0)) & 0xfffffff0) | 0xf);
    (void) REG_READ(I2S_CONF_REG(0));
    REG_WRITE(I2S_CONF_REG(0), REG_READ(I2S_CONF_REG(0)) & 0xfffffff0);
    while (GET_PERI_REG_BITS2(I2S_STATE_REG(0), 0x1, I2S_TX_FIFO_RESET_BACK_S));

    SET_PERI_REG_BITS(I2S_INT_ENA_REG(0), 0x1, 1, I2S_IN_DONE_INT_ENA_S);
    ESP_INTR_ENABLE(ETS_I2S0_INUM);
	SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 1, I2S_RX_START_S);
}

/*
static void i2s_stop() {
    ESP_INTR_DISABLE(ETS_I2S0_INUM);

    REG_WRITE(I2S_CONF_REG(0), REG_READ(I2S_CONF_REG(0)) & 0xfffffff0);
    (void) REG_READ(I2S_CONF_REG(0));
    REG_WRITE(I2S_CONF_REG(0), (REG_READ(I2S_CONF_REG(0)) & 0xfffffff0) | 0xf);
    (void) REG_READ(I2S_CONF_REG(0));
    REG_WRITE(I2S_CONF_REG(0), REG_READ(I2S_CONF_REG(0)) & 0xfffffff0);

    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 0, I2S_RX_START_S);
    i2s_running = false;
}
*/

void TickI2SIn()
{
	i2s_run();
}

static void i2s_run()
{
    // wait for vsync
    //ESP_LOGD(TAG, "Waiting for VSYNC");
    //while(gpio_get_level(s_config.pin_vsync) != 0);
    //while(gpio_get_level(s_config.pin_vsync) == 0);
    ///ESP_LOGD(TAG, "Got VSYNC");
    // wait a bit
    //delay(2);

    // start RX
//    line_count = 0;
    isr_countIn = 0;
    i2s_running = true;
    i2s_fill_buf(0);
}

static void IRAM_ATTR i2s_isr(void* arg) {
    REG_WRITE(I2S_INT_CLR_REG(0), (REG_READ(I2S_INT_RAW_REG(0)) & 0xffffffc0) | 0x3f);
    ++isr_countIn;
}

