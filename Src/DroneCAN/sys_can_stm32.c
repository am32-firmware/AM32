/*
  sys_can.c - MCU specific CAN code for STM32 bxCAN
 */

#include "targets.h"

#if DRONECAN_SUPPORT && defined(MCU_L431)

#include "sys_can.h"
#include <canard_stm32.h>
#include <_internal_bxcan.h>
#include "functions.h"

#define BXCAN CANARD_STM32_CAN1

// #pragma GCC optimize("O0")

/*
  usleep is needed by canard_stm32.c startup code
 */
void usleep(uint32_t usec)
{
    delayMicros(usec);
}

/*
  get a 16 byte unique ID for this node
*/
void sys_can_getUniqueID(uint8_t id[16])
{
    const uint8_t *uidbase = (const uint8_t *)UID_BASE;
    memcpy(id, uidbase, 12);

    // put CPU ID in last 4 bytes, handy for knowing the exact MCU we are on
    const uint32_t cpuid = SCB->CPUID;
    memcpy(&id[12], &cpuid, 4);
}

/*
  canned when a mailbox has a pending incoming frame
 */
static void handleTxMailboxInterrupt(uint8_t mbox, bool txok)
{
    DroneCAN_processTxQueue();
}

/*
  check for error state
 */
static void pollErrorFlagsFromISR()
{
    const uint8_t lec = (uint8_t)((BXCAN->ESR & CANARD_STM32_CAN_ESR_LEC_MASK) >> CANARD_STM32_CAN_ESR_LEC_SHIFT);
    if (lec != 0) {
	canstats.esr = BXCAN->ESR; // Record error status
	BXCAN->ESR = 0;
    }
}

/*
  handle all TX interrupts
 */
static void handleTxInterrupt(void)
{
    canstats.num_tx_interrupts++;

    // TXOK == false means that there was a hardware failure
    if (BXCAN->TSR & CANARD_STM32_CAN_TSR_RQCP0) {
        const bool txok = BXCAN->TSR & CANARD_STM32_CAN_TSR_TXOK0;
        BXCAN->TSR = CANARD_STM32_CAN_TSR_RQCP0;
	handleTxMailboxInterrupt(0, txok);
    }
    if (BXCAN->TSR & CANARD_STM32_CAN_TSR_RQCP1) {
        const bool txok = BXCAN->TSR & CANARD_STM32_CAN_TSR_TXOK1;
        BXCAN->TSR = CANARD_STM32_CAN_TSR_RQCP1;
	handleTxMailboxInterrupt(1, txok);
    }
    if (BXCAN->TSR & CANARD_STM32_CAN_TSR_RQCP2) {
        const bool txok = BXCAN->TSR & CANARD_STM32_CAN_TSR_TXOK2;
        BXCAN->TSR = CANARD_STM32_CAN_TSR_RQCP2;
	handleTxMailboxInterrupt(2, txok);
    }

    pollErrorFlagsFromISR();
}

/*
  handle all RX interrupts
 */
static void handleRxInterrupt(uint8_t fifo_index)
{
    canstats.num_rx_interrupts++;

    volatile uint32_t* const rfr_reg = (fifo_index == 0) ? &BXCAN->RF0R : &BXCAN->RF1R;
    if ((*rfr_reg & CANARD_STM32_CAN_RFR_FMP_MASK) == 0) {
        return;
    }

    /*
     * Register overflow as a hardware error
     */
    if ((*rfr_reg & CANARD_STM32_CAN_RFR_FOVR) != 0) {
	canstats.rx_errors++;
    }

    DroneCAN_receiveFrame();

    pollErrorFlagsFromISR();
}

/*
  interrupt handlers for CAN1
*/
void CAN1_RX0_IRQHandler(void)
{
    handleRxInterrupt(0);
}

void CAN1_RX1_IRQHandler(void)
{
    handleRxInterrupt(1);
}

void CAN1_TX_IRQHandler(void)
{
    handleTxInterrupt();
}

/*
  try to transmit a frame.
  return 1 for success, 0 for no space, -ve for failure
 */
int16_t sys_can_transmit(const CanardCANFrame* txf)
{
    return canardSTM32Transmit(txf);
}

/*
  check for an incoming frame
  return 1 on new frame, 0 for no frame, -ve for erro
 */
int16_t sys_can_receive(CanardCANFrame *rx_frame)
{
    return canardSTM32Receive(rx_frame);
}

/*
  disable CAN IRQs
 */
void sys_can_disable_IRQ(void)
{
    NVIC_DisableIRQ(CAN1_RX0_IRQn);
    NVIC_DisableIRQ(CAN1_RX1_IRQn);
    NVIC_DisableIRQ(CAN1_TX_IRQn);
}

/*
  enable CAN IRQs
 */
void sys_can_enable_IRQ(void)
{
    NVIC_EnableIRQ(CAN1_RX0_IRQn);
    NVIC_EnableIRQ(CAN1_RX1_IRQn);
    NVIC_EnableIRQ(CAN1_TX_IRQn);
}

/*
  init code should be small, not fast
 */
#pragma GCC optimize("Os")

/*
  initialise CAN hardware
 */
void sys_can_init(void)
{
    /*
      setup CAN RX and TX pins
     */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CAN1);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

    // assume PA11/PA12 for now
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LL_GPIO_PIN_11 | LL_GPIO_PIN_12;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = 9; // AF9==CAN
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    LL_RCC_ClocksTypeDef Clocks;
    LL_RCC_GetSystemClocksFreq(&Clocks);

    /*
      work out timing for 1MBps CAN
     */
    CanardSTM32CANTimings timings;

#if 0
    // use this to get timings for a new board
    canardSTM32ComputeCANTimings(Clocks.PCLK1_Frequency, 1000000, &timings);
#else
    switch (Clocks.PCLK1_Frequency) {
        case 80000000UL:
            timings.bit_rate_prescaler = 8;
            timings.bit_segment_1 = 8;
            timings.bit_segment_2 = 1;
            timings.max_resynchronization_jump_width = 1;
            break;
        default:
            // need to port to this board
            while (true) {
                delayMicros(1000);
            }
            break;
    }
#endif

    canardSTM32Init(&timings, CanardSTM32IfaceModeNormal);

    /*
      enable interrupt for CAN receive and transmit
    */
    NVIC_SetPriority(CAN1_RX0_IRQn, 5);
    NVIC_SetPriority(CAN1_RX1_IRQn, 5);
    NVIC_SetPriority(CAN1_TX_IRQn, 5);
    BXCAN->IER = CANARD_STM32_CAN_IER_FMPIE0 | CANARD_STM32_CAN_IER_FMPIE1 | CANARD_STM32_CAN_IER_TMEIE;
}

uint32_t get_rtc_backup_register(uint8_t idx)
{
    const volatile uint32_t *bkp = &RTC->BKP0R;
    return bkp[idx];
}

void set_rtc_backup_register(uint8_t idx, uint32_t value)
{
    volatile uint32_t *bkp = &RTC->BKP0R;
    bkp[idx] = value;
}

/*
  setup a static port/pin
 */
void setup_portpin(uint16_t portpin, bool enable)
{
    const uint8_t port = portpin >> 8;
    const uint8_t pin = portpin & 0xff;
    const uint32_t pinshift = 1U<<pin;
    GPIO_TypeDef *pport = port==0?GPIOA:GPIOB;

    if (enable) {
        LL_GPIO_SetOutputPin(pport, pinshift);
    } else {
        LL_GPIO_ResetOutputPin(pport, pinshift);
    }
    
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pinshift;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    LL_GPIO_Init(pport, &GPIO_InitStruct);
}

#endif // DRONECAN_SUPPORT && defined(MCU_L431)

