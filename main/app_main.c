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

void asmtest();

uint32_t mydata;

void app_main()
{

#define I2SOTEST

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

