#ifndef _I2S_STREAM_H
#define _I2S_STREAM_H


#define BUFF_SIZE_BYTES 2048

extern volatile unsigned isr_countIn;
extern uint16_t * i2sbufferIn[2] __attribute__((aligned(128)));

void SetupI2SIn();
void TickI2SIn();

#endif

