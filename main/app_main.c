
// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"

#include "esp_err.h"
#include "esp_log.h"

#include "i2s_stream_out.h"


#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"

#include "i2s_stream_fast.h"

#include <soc/gpio_struct.h>

void asmtest();
int test_fast_gpio();

#define XT_INTEXC_HOOK_NUM  (1 + XCHAL_NUM_INTLEVELS + XCHAL_HAVE_NMI)
extern  volatile void * _xt_intexc_hooks[XT_INTEXC_HOOK_NUM];

void _my_xt_nmi();
uint32_t mydata;


esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

void app_main()
{
//#define DO_WIFI
#ifdef DO_WIFI

	nvs_flash_init();
	tcpip_adapter_init();
	ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
	ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
	ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
	wifi_config_t sta_config = {
		.sta = {
		    .ssid = "charles",
		    .password = "thisiswifi",
		    .bssid_set = false
		}
	};
	ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
	ESP_ERROR_CHECK( esp_wifi_start() );
	ESP_ERROR_CHECK( esp_wifi_connect() );
#endif



//#define GPIO_ASM_TEST
#ifdef GPIO_ASM_TEST
	GPIO.enable_w1ts = 1<<17;
	GPIO.out_w1ts = 1<<17;
	GPIO.out_w1tc = 1<<17;
	GPIO.out_w1ts = 1<<17;
	GPIO.out_w1tc = 1<<17;
	int rr = test_fast_gpio();
	//printf( "TFGPIO: %08x\n",  );
	while(1)
	{
		GPIO.out_w1ts = 1<<17;
		GPIO.out_w1tc = 1<<17;
	}
#endif

#define NMI_TEST
#ifdef NMI_TEST
	GPIO.func_out_sel_cfg[16].func_sel = 256;
	WRITE_PERI_REG( DR_REG_IO_MUX_BASE +0x4c,  0xa00 ); //GPIO 16 GPIO.
	printf( "%08x %08x %d %d\n", READ_PERI_REG( DR_REG_IO_MUX_BASE +0x4c ), READ_PERI_REG( DR_REG_IO_MUX_BASE +0x50 ), GPIO.func_out_sel_cfg[16].val, GPIO.func_out_sel_cfg[17].val  );
	GPIO.enable_w1ts = 1<<17;
	GPIO.enable_w1ts = 1<<16;

	GPIO.out_w1ts = 1<<16;
	GPIO.out_w1tc = 1<<16;
	GPIO.out_w1ts = 1<<16;
	GPIO.out_w1tc = 1<<16;

	_xt_intexc_hooks[XCHAL_NMILEVEL] = &_my_xt_nmi;

	GPIO.pin[0].int_ena = GPIO_PRO_CPU_NMI_INTR_ENA; ///Why can't I set the app CPU?
	GPIO.pin[0].int_type = 3; //Level-change trigger.

	#define ETS_GPIO_INUM 14 //Actually NMI vector.
	intr_matrix_set( 0, ETS_GPIO_NMI_SOURCE, ETS_GPIO_INUM );
	ESP_INTR_ENABLE( ETS_GPIO_INUM );


	while(1)
	{
		GPIO.out_w1ts = 1<<17;
		GPIO.out_w1tc = 1<<17;

	}
#endif


#define I2SOTEST_FAST


#ifdef I2SOTEST_FAST
	printf( "!!!!!!!!!!!!!!!!!!!!!!1\n" );
	SetupI2SOut_fast();
	printf( "!!!!!!!!!!!!!!!!!!!!!!2\n" );
	TickI2SOut_fast();
	printf( "!!!!!!!!!!!!!!!!!!!!!!3\n" );

	while(true)
	{
		printf( "%d %08x\n", isr_countOut_fast, i2sbufferOut_fast[0][3] );
        vTaskDelay(1000 / portTICK_RATE_MS);
	}

#endif
#ifdef I2SOTEST
	SetupI2SOut();
	TickI2SOut();

	while(true)
	{
		printf( "%d %08x\n", isr_countOut, i2sbufferOut[0][3] );
        vTaskDelay(1000 / portTICK_RATE_MS);
	}

#endif

#ifdef GPIOASMTEST

    gpio_config_t conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
			.pin_bit_mask = 1LL << 18
    };
    gpio_config(&conf);

	while(true)
	{
		//gpio_set_level( 18, 0 );
		//gpio_set_level( 18, 1 );
        GPIO.out_w1ts = (1 << 18);
		asmtest();
		GPIO.out_w1tc = (1 << 18);
		asmtest();
//            GPIO.out1_w1ts.data = (1 << (gpio_num - 32));


//		printf( "%d %08x\n", isr_countOut, i2sbufferOut[0][3] );
//        vTaskDelay(1000 / portTICK_RATE_MS);
	}

#endif

}

