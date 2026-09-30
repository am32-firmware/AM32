/*
 * IO.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "IO.h"

#include "common.h"
#include "dshot.h"
#include "functions.h"
#include "serial_telemetry.h"
#include "signal.h"
#include "targets.h"
#include "ultra.h"

char ic_timer_prescaler = CPU_FREQUENCY_MHZ / 7;
uint32_t dma_buffer[64] = { 0 };
volatile char out_put = 0;
uint8_t buffer_padding = 7;

#ifdef ULTRA_DEDICATED
extern void processDshot(void);

// ultra mode: the input capture DMA runs without a transfer complete
// interrupt and is drained here from the 20kHz loop; a packet is complete
// once the line has been idle for more than 2x the expected bit time.
// DMA_start_bit stays valid until the next accepted packet: the software
// EXTI that decodes it only preempts this routine on some MCUs
void runDshotCheck()
{
    // one snapshot of the DMA counter per pass, and the timer is read
    // after the last captured edge: an edge arriving in between must not
    // make the line look idle or change the packet being judged
    const uint16_t remaining = INPUT_DMA_CHANNEL->dtcnt;
    if (remaining < 63) {
        if (armed) {
            const uint32_t last_edge = dma_buffer[63 - remaining];
            if ((IC_TIMER_REGISTER->cval - last_edge) > (uint32_t)(valid_packet_high << 1)) {
                if (ultraPacketStart(dma_buffer, remaining, valid_packet_high, &DMA_start_bit)) {
                    transfercomplete();
                    EXINT->swtrg = EXINT_LINE_15;
                } else {
                    packet_length_badcounts++;
                }
                INPUT_DMA_CHANNEL->ctrl_bit.chen = FALSE;
                INPUT_DMA_CHANNEL->dtcnt = 64;
                INPUT_DMA_CHANNEL->ctrl_bit.chen = TRUE;
                IC_TIMER_REGISTER->cval = 0;
            }
        } else {
            if (remaining <= 32) {
                DMA_start_bit = 0;
                transfercomplete();
                processDshot();
                INPUT_DMA_CHANNEL->ctrl_bit.chen = FALSE;
                INPUT_DMA_CHANNEL->dtcnt = 64;
                INPUT_DMA_CHANNEL->ctrl_bit.chen = TRUE;
                IC_TIMER_REGISTER->cval = 0;
            }
        }
    }
}
#endif // ULTRA_DEDICATED

void changeToOutput()
{
    INPUT_DMA_CHANNEL->ctrl |= DMA_DIR_MEMORY_TO_PERIPHERAL;
    tmr_reset(IC_TIMER_REGISTER);
    IC_TIMER_REGISTER->cm1 = 0x60; // oc mode pwm
    IC_TIMER_REGISTER->cctrl = 0x3; //
    IC_TIMER_REGISTER->div = output_timer_prescaler;
    IC_TIMER_REGISTER->pr = 76; // 76 to start

    out_put = 1;
    IC_TIMER_REGISTER->swevt_bit.ovfswtr = TRUE;
}

void changeToInput()
{
    INPUT_DMA_CHANNEL->ctrl |= DMA_DIR_PERIPHERAL_TO_MEMORY;
    tmr_reset(IC_TIMER_REGISTER);
    IC_TIMER_REGISTER->cm1 = 0x41;
    IC_TIMER_REGISTER->cctrl = 0xB;
    IC_TIMER_REGISTER->div = ic_timer_prescaler;
    IC_TIMER_REGISTER->pr = 0xFFFF;
    IC_TIMER_REGISTER->swevt_bit.ovfswtr = TRUE;
    out_put = 0;
}
void receiveDshotDma()
{
    changeToInput();
#ifdef ULTRA_DEDICATED
    // polled mode: 64 deep capture, no transfer complete interrupt,
    // timer counter free-runs for the idle gap detection in
    // runDshotCheck(); lighter input filter, noise is handled in
    // software instead
    IC_TIMER_REGISTER->cm1 = 0x21;
    INPUT_DMA_CHANNEL->paddr = (uint32_t)&IC_TIMER_REGISTER->c1dt;
    INPUT_DMA_CHANNEL->maddr = (uint32_t)&dma_buffer;
    INPUT_DMA_CHANNEL->dtcnt = 64;
    IC_TIMER_REGISTER->iden |= TMR_C1_DMA_REQUEST;
    IC_TIMER_REGISTER->ctrl1_bit.tmren = TRUE;
    INPUT_DMA_CHANNEL->ctrl = 0x00000989;
#else
    IC_TIMER_REGISTER->cval = 0;
    INPUT_DMA_CHANNEL->paddr = (uint32_t)&IC_TIMER_REGISTER->c1dt;
    INPUT_DMA_CHANNEL->maddr = (uint32_t)&dma_buffer;
    INPUT_DMA_CHANNEL->dtcnt = buffersize;
    IC_TIMER_REGISTER->iden |= TMR_C1_DMA_REQUEST;
    IC_TIMER_REGISTER->ctrl1_bit.tmren = TRUE;
    INPUT_DMA_CHANNEL->ctrl = 0x0000098b;
#endif
}

void sendDshotDma()
{
    changeToOutput();
    INPUT_DMA_CHANNEL->paddr = (uint32_t)&IC_TIMER_REGISTER->c1dt;
    INPUT_DMA_CHANNEL->maddr = (uint32_t)&gcr;
    INPUT_DMA_CHANNEL->dtcnt = 23 + buffer_padding;
    INPUT_DMA_CHANNEL->ctrl |= DMA_FDT_INT;
    INPUT_DMA_CHANNEL->ctrl |= DMA_DTERR_INT;
    INPUT_DMA_CHANNEL->ctrl_bit.chen = TRUE;
    IC_TIMER_REGISTER->iden |= TMR_C1_DMA_REQUEST;
    IC_TIMER_REGISTER->brk_bit.oen = TRUE;
    IC_TIMER_REGISTER->ctrl1_bit.tmren = TRUE;
}

uint8_t getInputPinState()
{
    uint8_t state = INPUT_PIN_PORT->idt & INPUT_PIN;
    return state;
}

void setInputPolarityRising()
{
    IC_TIMER_REGISTER->cctrl_bit.c1p = TMR_INPUT_RISING_EDGE;
}

void setInputPullDown()
{
    gpio_mode_set(INPUT_PIN_PORT, GPIO_MODE_MUX, GPIO_PULL_DOWN, INPUT_PIN);
}

void setInputPullUp()
{
    gpio_mode_set(INPUT_PIN_PORT, GPIO_MODE_MUX, GPIO_PULL_UP, INPUT_PIN);
}

void enableHalfTransferInt() { INPUT_DMA_CHANNEL->ctrl |= DMA_HDT_INT; }
void setInputPullNone()
{
    gpio_mode_set(INPUT_PIN_PORT, GPIO_MODE_MUX, GPIO_PULL_NONE, INPUT_PIN);
}
