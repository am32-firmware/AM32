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

char ic_timer_prescaler = CPU_FREQUENCY_MHZ / 5 - 2;
uint32_t dma_buffer[64] = { 0 };
volatile char out_put = 0;
uint8_t buffer_padding = 0;
uint8_t buffer_size = 32;
uint16_t change_time = 0;

#ifdef ULTRA_DEDICATED
extern void processDshot(void);

// ultra mode: the input capture DMA runs without a transfer complete
// interrupt and is drained here from the 20kHz loop; a packet is complete
// once the line has been idle for more than 2x the expected bit time.
// DMA_start_bit stays valid until the next accepted packet: the software
// EXTI that decodes it only preempts this routine on some MCUs
void runDshotCheck()
{
    if (DMA_CHCNT(INPUT_DMA_CHANNEL) < 63) {
        if (armed) {
            if ((TIMER_CNT(IC_TIMER_REGISTER) - dma_buffer[63 - DMA_CHCNT(INPUT_DMA_CHANNEL)]) > (uint32_t)(valid_packet_high << 1)) {
                if (ultraPacketStart(dma_buffer, DMA_CHCNT(INPUT_DMA_CHANNEL), valid_packet_high, &DMA_start_bit)) {
                    transfercomplete();
                    EXTI_SWIEV |= (uint32_t)EXTI_15;
                } else {
                    packet_length_badcounts++;
                }
                dma_channel_disable(INPUT_DMA_CHANNEL);
                DMA_CHCNT(INPUT_DMA_CHANNEL) = 64;
                dma_channel_enable(INPUT_DMA_CHANNEL);
                TIMER_CNT(IC_TIMER_REGISTER) = 0;
            }
        } else {
            if (DMA_CHCNT(INPUT_DMA_CHANNEL) <= 32) {
                DMA_start_bit = 0;
                transfercomplete();
                processDshot();
                dma_channel_disable(INPUT_DMA_CHANNEL);
                DMA_CHCNT(INPUT_DMA_CHANNEL) = 64;
                dma_channel_enable(INPUT_DMA_CHANNEL);
                TIMER_CNT(IC_TIMER_REGISTER) = 0;
            }
        }
    }
}
#endif // ULTRA_DEDICATED

void receiveDshotDma()
{
    #ifdef USE_TIMER_2_CHANNEL_0
    RCU_REG_VAL(RCU_TIMER2RST) |= BIT(RCU_BIT_POS(RCU_TIMER2RST));
    RCU_REG_VAL(RCU_TIMER2RST) &= ~BIT(RCU_BIT_POS(RCU_TIMER2RST));
    #endif
    #ifdef USE_TIMER_14_CHANNEL_0
    RCU_REG_VAL(RCU_TIMER14RST) |= BIT(RCU_BIT_POS(RCU_TIMER14RST));
    RCU_REG_VAL(RCU_TIMER14RST) &= ~BIT(RCU_BIT_POS(RCU_TIMER14RST));
    #endif

#ifdef ULTRA_DEDICATED
    // lighter input filter, noise is handled in runDshotCheck()
    TIMER_CHCTL0(IC_TIMER_REGISTER) = 0x21;
#else
    TIMER_CHCTL0(IC_TIMER_REGISTER) = 0x41;
#endif
    TIMER_CHCTL2(IC_TIMER_REGISTER) = 0xa;
    TIMER_PSC(IC_TIMER_REGISTER) = ic_timer_prescaler;
    TIMER_CAR(IC_TIMER_REGISTER) = 0xFFFF;
    TIMER_SWEVG(IC_TIMER_REGISTER) |= (uint32_t)TIMER_EVENT_SRC_UPG;
    out_put = 0;
    TIMER_CNT(IC_TIMER_REGISTER) = 0;
    DMA_CHMADDR(INPUT_DMA_CHANNEL) = (uint32_t)&dma_buffer;
#ifdef ULTRA_DEDICATED
    // polled mode: 64 deep capture, no transfer complete interrupt
    DMA_CHCNT(INPUT_DMA_CHANNEL) = 64;
#else
    DMA_CHCNT(INPUT_DMA_CHANNEL) = (buffersize & DMA_CHANNEL_CNT_MASK);
#endif
    TIMER_DMAINTEN(IC_TIMER_REGISTER) |= (uint32_t)TIMER_DMA_CH0D;
    TIMER_CHCTL2(IC_TIMER_REGISTER) |= (uint32_t)TIMER_CCX_ENABLE;
    TIMER_CTL0(IC_TIMER_REGISTER) |= (uint32_t)TIMER_CTL0_CEN;
#ifdef ULTRA_DEDICATED
    DMA_CHCTL(INPUT_DMA_CHANNEL) = 0x00000989; // enable without transfer complete interrupt
#else
    DMA_CHCTL(INPUT_DMA_CHANNEL) = 0x0000098b; // just set the whole reg in one go to enable
#endif
}

void sendDshotDma()
{
    #ifdef USE_TIMER_2_CHANNEL_0
    RCU_REG_VAL(RCU_TIMER2RST) |= BIT(RCU_BIT_POS(RCU_TIMER2RST));
    RCU_REG_VAL(RCU_TIMER2RST) &= ~BIT(RCU_BIT_POS(RCU_TIMER2RST));
    #endif
    #ifdef USE_TIMER_14_CHANNEL_0
    RCU_REG_VAL(RCU_TIMER14RST) |= BIT(RCU_BIT_POS(RCU_TIMER14RST));
    RCU_REG_VAL(RCU_TIMER14RST) &= ~BIT(RCU_BIT_POS(RCU_TIMER14RST));
    #endif
    
    TIMER_CHCTL0(IC_TIMER_REGISTER) = 0x60;
    TIMER_CHCTL2(IC_TIMER_REGISTER) = 0x3;
    TIMER_PSC(IC_TIMER_REGISTER) = output_timer_prescaler;
    TIMER_CAR(IC_TIMER_REGISTER) = 100;
    out_put = 1;
    TIMER_SWEVG(IC_TIMER_REGISTER) |= (uint32_t)TIMER_EVENT_SRC_UPG;
    DMA_CHMADDR(INPUT_DMA_CHANNEL) = (uint32_t)&gcr;
    DMA_CHCNT(INPUT_DMA_CHANNEL) = ((23 + buffer_padding) & DMA_CHANNEL_CNT_MASK);
    DMA_CHCTL(INPUT_DMA_CHANNEL) = 0x0000099b;
    TIMER_DMAINTEN(IC_TIMER_REGISTER) |= (uint32_t)TIMER_DMA_CH0D;
    TIMER_CHCTL2(IC_TIMER_REGISTER) |= (uint32_t)TIMER_CCX_ENABLE;
    TIMER_CCHP(IC_TIMER_REGISTER) |= (uint32_t)TIMER_CCHP_POEN;
    TIMER_CTL0(IC_TIMER_REGISTER) |= (uint32_t)TIMER_CTL0_CEN;
}

uint8_t getInputPinState() { return GPIO_ISTAT(INPUT_PIN_PORT) & (INPUT_PIN); }

void setInputPolarityRising()
{
    TIMER_CHCTL2(IC_TIMER_REGISTER) |= (uint32_t)(TIMER_IC_POLARITY_RISING);
}

void setInputPullDown()
{
    gpio_mode_set(INPUT_PIN_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, INPUT_PIN);
}

void setInputPullUp()
{
    gpio_mode_set(INPUT_PIN_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, INPUT_PIN);
}

void enableHalfTransferInt() { DMA_CHCTL(INPUT_DMA_CHANNEL) |= DMA_INT_HTF; }
void setInputPullNone()
{
    gpio_mode_set(INPUT_PIN_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, INPUT_PIN);
}
