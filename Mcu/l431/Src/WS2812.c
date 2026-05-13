#include "WS2812.h"
#include "targets.h"

#ifdef USE_LED_STRIP

#include "stm32l4xx_ll_spi.h"

#if !defined(WS2812_PORT) || !defined(WS2812_PIN) || !defined(WS2812_AF)
#error "WS2812_PORT / WS2812_PIN / WS2812_AF must be defined by the hardware group"
#endif

/* SPI1 MOSI driven via DMA1 channel 3 (the only request mapping for
 * SPI1_TX on L431). The pin/AF come from the hardware group, but every
 * L4 group points the same place because PB5 (AF5) is the only SPI1_MOSI
 * pin this part exposes. Clocked at PCLK2 / 32 = 2.5 MHz, each WS2812
 * bit becomes 3 SPI bits at 400 ns/bit:
 *   '0' -> 0b100  (400 ns high, 800 ns low)
 *   '1' -> 0b110  (800 ns high, 400 ns low)
 * 24 colour bits per LED * 3 = 72 SPI bits = 9 bytes.
 * 16 trailing zero bytes hold MOSI low for ~51 us, satisfying the
 * WS2812 reset latch so back-to-back sends still latch correctly. */
#define WS2812_BYTES_PER_LED 9
#define WS2812_RESET_BYTES   16
#define WS2812_BUFFER_BYTES  (LED_COUNT * WS2812_BYTES_PER_LED + WS2812_RESET_BYTES)

static uint8_t led_dma_buffer[WS2812_BUFFER_BYTES];

static void encode_byte(uint8_t in, uint8_t *out)
{
    uint32_t acc = 0;
    for (int b = 7; b >= 0; b--) {
        acc <<= 3;
        acc |= ((in >> b) & 1u) ? 0x6u : 0x4u;
    }
    out[0] = (uint8_t)(acc >> 16);
    out[1] = (uint8_t)(acc >> 8);
    out[2] = (uint8_t)acc;
}

void WS2812_Init(void)
{
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_GPIO_SetPinMode(WS2812_PORT, WS2812_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(WS2812_PORT, WS2812_PIN, WS2812_AF);
    LL_GPIO_SetPinSpeed(WS2812_PORT, WS2812_PIN, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinOutputType(WS2812_PORT, WS2812_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(WS2812_PORT, WS2812_PIN, LL_GPIO_PULL_NO);

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

    /* Master, SSM+SSI (NSS managed in software), BR=/32, CPOL=0/CPHA=0,
     * MSB-first, half-duplex transmit-only on MOSI so MISO pin is free. */
    SPI1->CR1 = SPI_CR1_MSTR
              | SPI_CR1_SSM | SPI_CR1_SSI
              | SPI_CR1_BR_2
              | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE;
    /* 8-bit data, RX FIFO threshold = 1/4 (byte access), TX DMA enable. */
    SPI1->CR2 = (7u << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH | SPI_CR2_TXDMAEN;

    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMA_REQUEST_1);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3,
                                    LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);

    LL_SPI_Enable(SPI1);
}

void send_LED_RGB(uint8_t red, uint8_t green, uint8_t blue)
{
    /* Skip if a transfer is still in flight. The caller's state machine
     * re-sends on every transition, so the next call will succeed. */
    if ((DMA1_Channel3->CCR & DMA_CCR_EN) && DMA1_Channel3->CNDTR != 0) {
        return;
    }
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);

    uint8_t *p = led_dma_buffer;
    for (uint8_t led = 0; led < LED_COUNT; led++) {
        encode_byte(green, p); p += 3;
        encode_byte(red,   p); p += 3;
        encode_byte(blue,  p); p += 3;
    }
    /* Trailing reset window stays zero-initialised in BSS but write it
     * here too so the buffer survives a warm-restart with leftover data. */
    for (uint32_t i = 0; i < WS2812_RESET_BYTES; i++) {
        p[i] = 0;
    }

    DMA1_Channel3->CMAR  = (uint32_t)led_dma_buffer;
    DMA1_Channel3->CPAR  = (uint32_t)&SPI1->DR;
    DMA1_Channel3->CNDTR = WS2812_BUFFER_BYTES;
    DMA1->IFCR = DMA_IFCR_CGIF3;

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
}

#endif
