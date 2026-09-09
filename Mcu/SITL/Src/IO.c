/*
  IO.c - SITL signal IO. DShot/servo input arrives as UDP packets handled
  by sitl_input.c, which emulates the input capture timer + DMA
 */

#include "IO.h"

#include "sitl.h"
#include "targets.h"

uint32_t dma_buffer[64];
volatile char out_put;
char ic_timer_prescaler = CPU_FREQUENCY_MHZ / 6;
uint8_t buffer_padding;

void changeToOutput(void) { }
void changeToInput(void) { }

void receiveDshotDma(void)
{
    out_put = 0;
    sitl_input_arm();
}

void sendDshotDma(void)
{
    out_put = 1;
    sitl_input_send_reply();
}

uint8_t getInputPinState(void)
{
    return sitl_input_pin_state();
}

void setInputPolarityRising(void) { }
void setInputPullDown(void) { }
void setInputPullUp(void) { }
void setInputPullNone(void) { }
void enableHalfTransferInt(void) { }
