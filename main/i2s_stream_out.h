#ifndef _I2S_STREAM_H
#define _I2S_STREAM_H


#define BUFF_SIZE_BYTES 2048

extern volatile unsigned isr_countOut;
extern uint16_t * i2sbufferOut[3] __attribute__((aligned(128)));

void SetupI2SOut();
void TickI2SOut();

#endif

