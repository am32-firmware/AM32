/*
  sys_can_stm32_CANFD.c - MCU specific CAN FD code for STM32 FDCAN

  based on ArduPilot CANFDIface.cpp driver
 */

#include "targets.h"

#if DRONECAN_SUPPORT && defined(MCU_G431)

// #pragma GCC optimize("O0")

#include "sys_can.h"
#include "functions.h"
#include <string.h>

// FDCAN Frame buffer - 18 words per frame
#define FDCAN_FRAME_BUFFER_SIZE 18

// Message RAM Allocations for STM32G4
#define MAX_FILTER_LIST_SIZE 80U
#define FDCAN_NUM_RXFIFO0_SIZE 104U
#define FDCAN_TX_FIFO_BUFFER_SIZE 128U
#define FDCAN_MESSAGERAM_STRIDE 0x350
#define FDCAN_EXFILTER_OFFSET 0x70
#define FDCAN_RXFIFO0_OFFSET 0xB0
#define FDCAN_RXFIFO1_OFFSET 0x188
#define FDCAN_TXFIFO_OFFSET 0x278

#define FDCAN_RXF0S_F0GI_SHIFT 8
#define FDCAN_RXF1S_F1GI_SHIFT 8
#define FDCAN_TXFQS_TFGI_SHIFT 8

// CAN ID masks (used for frame parsing)
#define MaskStdID 0x7FFU
#define MaskExtID 0x1FFFFFFFU

// Message RAM element structures
typedef struct {
  uint32_t id_flags;
  uint32_t dlc_timestamp;
  uint32_t data[16];  // Up to 64 bytes for CANFD
} RxMessageRAM;

typedef struct {
  uint32_t id_flags;
  uint32_t dlc_flags;
  uint32_t data[16];  // Up to 64 bytes for CANFD
} TxMessageRAM;

volatile struct {
  uint32_t rx_overflow;
} can_stats;

// Message RAM addresses (for STM32G4, these are at fixed offsets)
static uint32_t MessageRam_StandardFilterSA;
static uint32_t MessageRam_ExtendedFilterSA;
static uint32_t MessageRam_RxFIFO0SA;
static uint32_t MessageRam_RxFIFO1SA;
static uint32_t MessageRam_TxFIFOQSA;

/*
  setup message RAM for FDCAN1 on STM32G4
*/
static void setupMessageRam(void)
{
  // For FDCAN1, self_index = 0, so base = SRAMCAN_BASE + 0
  const uint32_t base = SRAMCAN_BASE + FDCAN_MESSAGERAM_STRIDE * 0;
  
  // Clear message RAM for this interface
  memset((void*)base, 0, FDCAN_MESSAGERAM_STRIDE);
  
  // Store message RAM addresses at fixed offsets
  MessageRam_StandardFilterSA = base;
  MessageRam_ExtendedFilterSA = base + FDCAN_EXFILTER_OFFSET;
  MessageRam_RxFIFO0SA = base + FDCAN_RXFIFO0_OFFSET;
  MessageRam_RxFIFO1SA = base + FDCAN_RXFIFO1_OFFSET;
  MessageRam_TxFIFOQSA = base + FDCAN_TXFIFO_OFFSET;
  
  // Set TXBC to 0 for FIFO mode (STM32G4 specific)
  FDCAN1->TXBC = 0;
}

/*
  send a CAN frame
*/
static bool can_send(const CanardCANFrame *frame)
{
  // Check if Tx FIFO is full
  if ((FDCAN1->TXFQS & FDCAN_TXFQS_TFQF) != 0) {
    return false;  // No space
  }
  
  // Get put index
  uint32_t put_index = (FDCAN1->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;
  
  // Calculate address in message RAM using stored base address
  volatile TxMessageRAM *tx_mailbox = (volatile TxMessageRAM *)(MessageRam_TxFIFOQSA + (put_index * FDCAN_FRAME_BUFFER_SIZE * 4));
  
  // Setup ID and flags
  if (frame->id & CANARD_CAN_FRAME_EFF) {
    tx_mailbox->id_flags = (frame->id & MaskExtID) | (1U << 30);
  } else {
    // Standard ID
    tx_mailbox->id_flags = (frame->id & MaskStdID) << 18;
  }
  
  if (frame->id & CANARD_CAN_FRAME_RTR) {
    tx_mailbox->id_flags |= (1U << 29);
  }
  
  // Setup DLC
  tx_mailbox->dlc_flags = (frame->data_len << 16);
  
  // Copy data
  const uint32_t *data_ptr = (const uint32_t *)frame->data;
  for (int i = 0; i < 2; i++) {
    tx_mailbox->data[i] = data_ptr[i];
  }
  
  // Request transmission
  FDCAN1->TXBAR = (1U << put_index);
  
  return true;
}

static void handleRxInterrupt(uint8_t fifo_index)
{
  volatile uint32_t *fifo_status_reg = (fifo_index == 0) ? &FDCAN1->RXF0S : &FDCAN1->RXF1S;
  volatile uint32_t *fifo_ack_reg = (fifo_index == 0) ? &FDCAN1->RXF0A : &FDCAN1->RXF1A;
  
  uint32_t fifo_level_mask = (fifo_index == 0) ? FDCAN_RXF0S_F0FL : FDCAN_RXF1S_F1FL;
  uint32_t get_index_mask = (fifo_index == 0) ? FDCAN_RXF0S_F0GI : FDCAN_RXF1S_F1GI;
  uint32_t get_index_shift = (fifo_index == 0) ? FDCAN_RXF0S_F0GI_SHIFT : FDCAN_RXF1S_F1GI_SHIFT;
  
  // Check if FIFO has messages
  if ((*fifo_status_reg & fifo_level_mask) == 0) {
    return;
  }
  
  // Get the get index
  uint32_t get_index = (*fifo_status_reg & get_index_mask) >> get_index_shift;
  
  // Calculate address in message RAM
  uint32_t rx_fifo_addr = (fifo_index == 0) ? MessageRam_RxFIFO0SA : MessageRam_RxFIFO1SA;
  volatile RxMessageRAM *rx_mailbox = (volatile RxMessageRAM *)(rx_fifo_addr + (get_index * FDCAN_FRAME_BUFFER_SIZE * 4));
  
  // Read the frame
  CanardCANFrame frame = {};
  
  uint32_t id_flags = rx_mailbox->id_flags;
  if (id_flags & (1U << 30)) {
    frame.id = (id_flags & MaskExtID) | CANARD_CAN_FRAME_EFF;
  } else {
    frame.id = (id_flags >> 18) & MaskStdID;
  }
  
  if (id_flags & (1U << 29)) {
    frame.id |= CANARD_CAN_FRAME_RTR;
  }
  
  // Get DLC
  uint32_t dlc = (rx_mailbox->dlc_timestamp >> 16) & 0xF;
  frame.data_len = dlc;
  
  // Copy data
  uint32_t *data_ptr = (uint32_t *)frame.data;
  for (int i = 0; i < 2; i++) {
    data_ptr[i] = rx_mailbox->data[i];
  }
  
  // Acknowledge the read
  *fifo_ack_reg = get_index;
  
  // Process the frame
  DroneCAN_handleFrame(&frame);
}

static void handleTxCompleteInterrupt(void)
{
  // Nothing to do in simple bootloader mode
}

static void pollErrorFlagsFromISR(void)
{
  // Simple error handling
  uint32_t ecr = FDCAN1->ECR;
  (void)ecr;
}

/*
  get a 16 byte unique ID for this node
*/
void sys_can_getUniqueID(uint8_t id[16])
{
  // Use CMSIS UID_BASE definition
  const uint8_t *uidbase = (const uint8_t *)UID_BASE;
  memcpy(id, uidbase, 12);

  // put CPU ID in last 4 bytes
  const uint32_t cpuid = SCB->CPUID;
  memcpy(&id[12], &cpuid, 4);
}

/*
  interrupt handlers for FDCAN1
*/
void FDCAN1_IT0_IRQHandler(void)
{
  // Rx FIFO interrupts
  if ((FDCAN1->IR & FDCAN_IR_RF0N) || (FDCAN1->IR & FDCAN_IR_RF0F)) {
    FDCAN1->IR = FDCAN_IR_RF0N | FDCAN_IR_RF0F;
    handleRxInterrupt(0);
  }
  
  if ((FDCAN1->IR & FDCAN_IR_RF1N) || (FDCAN1->IR & FDCAN_IR_RF1F)) {
    FDCAN1->IR = FDCAN_IR_RF1N | FDCAN_IR_RF1F;
    handleRxInterrupt(1);
  }
  
  pollErrorFlagsFromISR();
}

void FDCAN1_IT1_IRQHandler(void)
{
  // Tx complete interrupt
  if (FDCAN1->IR & FDCAN_IR_TC) {
    FDCAN1->IR = FDCAN_IR_TC;
    handleTxCompleteInterrupt();
  }
  
  // Bus off interrupt
  if (FDCAN1->IR & FDCAN_IR_BO) {
    FDCAN1->IR = FDCAN_IR_BO;
    // Try to recover from bus off
    FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;
  }
  
  pollErrorFlagsFromISR();
}

/*
  try to transmit a frame.
  return 1 for success, 0 for no space, -ve for failure
 */
int16_t sys_can_transmit(const CanardCANFrame* txf)
{
  return can_send(txf) ? 1 : 0;
}

/*
  check for an incoming frame
  return 1 on new frame, 0 for no frame, -ve for error
 */
int16_t sys_can_receive(CanardCANFrame *rx_frame)
{
  // not used on STM32 with interrupt-driven receive
  return -1;
}

/*
  disable CAN IRQs
 */
void sys_can_disable_IRQ(void)
{
  NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
}

/*
  enable CAN IRQs
 */
void sys_can_enable_IRQ(void)
{
  NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
}

/*
  wait for register bit to change state
*/
static bool waitForBitState(volatile uint32_t *reg, uint32_t mask, bool target_state)
{
  while (true) {
    bool current_state = ((*reg) & mask) != 0;
    if (current_state == target_state) {
      return true;
    }
  }
  return false;
}

/*
  init code should be small, not fast
 */
#pragma GCC optimize("Os")

static void can_init(void)
{
  // Enable FDCAN clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;
  
  // Wait for clock to stabilize
  for (volatile int i = 0; i < 10000; i++) {
    __NOP();
  }
  
  // Perform reset
  RCC->APB1RSTR1 |= RCC_APB1RSTR1_FDCANRST;
  RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_FDCANRST;
  
  // Wait after reset
  for (volatile int i = 0; i < 100; i++) {
    __NOP();
  }

  // Note: PLL_Q must be configured to output 80 MHz
  // This should be done in bl_clock_config() by enabling PLL Q domain
  // with Q divider = 4 (VCO 320 MHz / 4 = 80 MHz)
  
  // Exit sleep mode
  FDCAN1->CCCR &= ~FDCAN_CCCR_CSR;
  
  // Wait for sleep mode to exit
  if (!waitForBitState(&FDCAN1->CCCR, FDCAN_CCCR_CSA, false)) {
    return; // Failed to exit sleep mode
  }

  // Enter initialization mode
  FDCAN1->CCCR |= FDCAN_CCCR_INIT;
  
  // Wait for initialization mode (with timeout)
  if (!waitForBitState(&FDCAN1->CCCR, FDCAN_CCCR_INIT, true)) {
    return; // Failed to enter init mode
  }
  
  // Enable configuration change
  FDCAN1->CCCR |= FDCAN_CCCR_CCE;
  
  // Configure bit timing for 1 Mbps with 80 MHz FDCAN clock
  const uint8_t sjw = 1;
  const uint8_t bs1 = 8;
  const uint8_t bs2 = 1;
  const uint8_t prescaler = 8;
  
  FDCAN1->NBTP = (((sjw-1) << FDCAN_NBTP_NSJW_Pos)   |
                  ((bs1-1) << FDCAN_NBTP_NTSEG1_Pos) |
                  ((bs2-1) << FDCAN_NBTP_NTSEG2_Pos)  |
                  ((prescaler-1) << FDCAN_NBTP_NBRP_Pos));
  
  // Setup message RAM
  setupMessageRam();
  
  // Clear all interrupts
  FDCAN1->IR = 0x3FFFFFFF;
  
  // Configure interrupts
  FDCAN1->IE = FDCAN_IE_RF0NE |  // Rx FIFO 0 new message
               FDCAN_IE_RF0FE |  // Rx FIFO 0 Full
               FDCAN_IE_RF1NE |  // Rx FIFO 1 new message  
               FDCAN_IE_RF1FE |  // Rx FIFO 1 Full
               FDCAN_IE_TCE |    // Transmission complete
               FDCAN_IE_BOE;     // Bus off
  
  // Route interrupts
  FDCAN1->ILS = FDCAN_ILS_PERR | FDCAN_ILS_SMSG;
  
  // Configure Tx Buffer Transmission Interrupt Enable for STM32G4
  FDCAN1->TXBTIE = 0x7;
  
  // Enable both interrupt lines
  FDCAN1->ILE = 0x3;
  
  // Leave initialization mode
  FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;
  
  // Wait for normal mode
  waitForBitState(&FDCAN1->CCCR, FDCAN_CCCR_INIT, false);
}

/*
  initialise CAN hardware
 */
void sys_can_init(void)
{
  // Setup CAN RX and TX pins
  // assumes PA11/PA12 for FDCAN1
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11 | LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_9; // AF9 for FDCAN1
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  can_init();

  // Enable interrupt for CAN receive and transmit
  NVIC_SetPriority(FDCAN1_IT0_IRQn, 5);
  NVIC_SetPriority(FDCAN1_IT1_IRQn, 5);
  NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
}

uint32_t get_rtc_backup_register(uint8_t idx)
{
  const volatile uint32_t *bkp = &TAMP->BKP0R;
  return bkp[idx];
}

void set_rtc_backup_register(uint8_t idx, uint32_t value)
{
  volatile uint32_t *bkp = &TAMP->BKP0R;
  bkp[idx] = value;
}

#endif // DRONECAN_SUPPORT && defined(MCU_G431)
