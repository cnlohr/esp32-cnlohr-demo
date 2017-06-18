#ifndef _I2S_STREAM_FAST_H
#define _I2S_STREAM_FAST_H


#define BUFF_SIZE_BYTES 2048

extern volatile unsigned isr_countOut_fast;
extern uint32_t * i2sbufferOut_fast[3];

void SetupI2SOut_fast();
void TickI2SOut_fast();

#endif

