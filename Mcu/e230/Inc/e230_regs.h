/*
 * e230_regs.h
 *
 * Small register helpers that replace the GD32E23x standard peripheral
 * library sources. The vendor headers are still used for the register and
 * bit definitions, they cost no flash.
 */

#ifndef E230_REGS_H_
#define E230_REGS_H_

#include "gd32e23x.h"

// clock enable / disable / reset pulse, takes the rcu_periph_enum values from gd32e23x_rcu.h
#define RCU_CLOCK_ENABLE(periph) (RCU_REG_VAL(periph) |= BIT(RCU_BIT_POS(periph)))
#define RCU_CLOCK_DISABLE(periph) (RCU_REG_VAL(periph) &= ~BIT(RCU_BIT_POS(periph)))
#define RCU_RESET_PULSE(periph_rst) \
    (RCU_REG_VAL(periph_rst) |= BIT(RCU_BIT_POS(periph_rst)), RCU_REG_VAL(periph_rst) &= ~BIT(RCU_BIT_POS(periph_rst)))

#define FWDGT_RELOAD() (FWDGT_CTL = FWDGT_KEY_RELOAD)

// spread the 16 pin bits so pin n lands on bit 2n, one bit per 2 bit field
static inline uint32_t pinsTo2BitFields(uint32_t pins)
{
    pins &= 0xFFFFU;
    pins = (pins | (pins << 8)) & 0x00FF00FFU;
    pins = (pins | (pins << 4)) & 0x0F0F0F0FU;
    pins = (pins | (pins << 2)) & 0x33333333U;
    pins = (pins | (pins << 1)) & 0x55555555U;
    return pins;
}

// spread 8 pin bits so pin n lands on bit 4n, one bit per 4 bit field
static inline uint32_t pinsTo4BitFields(uint32_t pins)
{
    pins &= 0xFFU;
    pins = (pins | (pins << 12)) & 0x000F000FU;
    pins = (pins | (pins << 6)) & 0x03030303U;
    pins = (pins | (pins << 3)) & 0x11111111U;
    return pins;
}

// same result as gpio_mode_set()
static inline void gpioSetMode(uint32_t port, uint32_t mode, uint32_t pull_up_down, uint32_t pins)
{
    const uint32_t fields = pinsTo2BitFields(pins);
    GPIO_CTL(port) = (GPIO_CTL(port) & ~(fields * 3U)) | (fields * mode);
    GPIO_PUD(port) = (GPIO_PUD(port) & ~(fields * 3U)) | (fields * pull_up_down);
}

// same result as gpio_output_options_set()
static inline void gpioSetOutputOptions(uint32_t port, uint8_t otype, uint32_t speed, uint32_t pins)
{
    const uint32_t fields = pinsTo2BitFields(pins);
    if (otype == GPIO_OTYPE_OD) {
        GPIO_OMODE(port) |= pins;
    } else {
        GPIO_OMODE(port) &= ~pins;
    }
    GPIO_OSPD(port) = (GPIO_OSPD(port) & ~(fields * 3U)) | (fields * speed);
}

// same result as gpio_af_set()
static inline void gpioSetAf(uint32_t port, uint32_t af, uint32_t pins)
{
    const uint32_t low = pinsTo4BitFields(pins);
    const uint32_t high = pinsTo4BitFields(pins >> 8);
    if (low) {
        GPIO_AFSEL0(port) = (GPIO_AFSEL0(port) & ~(low * 0xFU)) | (low * af);
    }
    if (high) {
        GPIO_AFSEL1(port) = (GPIO_AFSEL1(port) & ~(high * 0xFU)) | (high * af);
    }
}

// same result as dma_deinit()
static inline void dmaDeinit(dma_channel_enum channel)
{
    DMA_CHCTL(channel) &= ~DMA_CHXCTL_CHEN;
    DMA_CHCTL(channel) = DMA_CHCTL_RESET_VALUE;
    DMA_CHCNT(channel) = DMA_CHCNT_RESET_VALUE;
    DMA_CHPADDR(channel) = DMA_CHPADDR_RESET_VALUE;
    DMA_CHMADDR(channel) = DMA_CHMADDR_RESET_VALUE;
    DMA_INTC |= DMA_FLAG_ADD(DMA_CHINTF_RESET_VALUE, channel);
}

// dma_interrupt_flag_get(): the flag only counts when its interrupt is enabled
#define DMA_INT_FLAG_ACTIVE(channel, flag, enable_bit) \
    ((DMA_INTF & DMA_FLAG_ADD(flag, channel)) && (DMA_CHCTL(channel) & (enable_bit)))
#define DMA_INT_FLAG_CLEAR(channel, flag) (DMA_INTC |= DMA_FLAG_ADD(flag, channel))

#endif /* E230_REGS_H_ */
