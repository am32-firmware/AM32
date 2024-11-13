/*
  sys_can.c - MCU specific CAN code for AT32 bxCAN
 */

#include "targets.h"

#if DRONECAN_SUPPORT && defined(ARTERY)

#include "sys_can.h"
#include "functions.h"
#include <string.h>
#include <at32f415_can.h>
#include <at32f415_gpio.h>
#include <at32f415_crm.h>


//#pragma GCC optimize("O0")

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
    const uint8_t *uidbase = (const uint8_t *)0x1FFFF7E8; // 96 bit UID
    memcpy(id, uidbase, 12);
    memset(&id[12], 0, 4);
}

/**
  *  @brief  can gpio config
  *  @param  none
  *  @retval none
  */
static void can_gpio_config(void)
{
    gpio_init_type gpio_init_struct;

    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
    gpio_pin_remap_config(CAN1_GMUX_0000,TRUE); // CAN_RX=PA11/CAN_TX=PA12
    // gpio_pin_remap_config(CAN1_GMUX_0010,TRUE); // CAN_RX=PB8/CAN_TX=PB9

    gpio_default_para_init(&gpio_init_struct);
    /* can tx pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = GPIO_PINS_12;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOA, &gpio_init_struct);
    /* can rx pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = GPIO_PINS_11;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOA, &gpio_init_struct);
}

/*
  initialise CAN hardware
 */
void sys_can_init(void)
{
    can_gpio_config();

    can_base_type can_base_struct;
    can_baudrate_type can_baudrate_struct;

    crm_periph_clock_enable(CRM_CAN1_PERIPH_CLOCK, TRUE);
    /* can base init */
    can_default_para_init(&can_base_struct);
    can_base_struct.mode_selection = CAN_MODE_COMMUNICATE;
    can_base_struct.ttc_enable = FALSE;
    can_base_struct.aebo_enable = TRUE;
    can_base_struct.aed_enable = TRUE;
    can_base_struct.prsf_enable = FALSE;
    can_base_struct.mdrsel_selection = CAN_DISCARDING_FIRST_RECEIVED;
    can_base_struct.mmssr_selection = CAN_SENDING_BY_REQUEST;
    can_base_init(CAN1, &can_base_struct);

    crm_clocks_freq_type clocks;
    crm_clocks_freq_get(&clocks);

    /* can baudrate, set baudrate = pclk/(baudrate_div *(1 + bts1_size + bts2_size)) */
    can_baudrate_struct.baudrate_div = 6;
    can_baudrate_struct.rsaw_size = CAN_RSAW_3TQ;
    can_baudrate_struct.bts1_size = CAN_BTS1_8TQ;
    can_baudrate_struct.bts2_size = CAN_BTS2_3TQ;
    can_baudrate_set(CAN1, &can_baudrate_struct);

    /* can filter init */
    can_filter_init_type can_filter_init_struct;
    can_filter_init_struct.filter_activate_enable = TRUE;
    can_filter_init_struct.filter_fifo = CAN_FILTER_FIFO0;
    can_filter_init_struct.filter_number = 0;
    can_filter_init_struct.filter_bit = CAN_FILTER_32BIT;
    can_filter_init_struct.filter_id_high = 0;
    can_filter_init_struct.filter_id_low = 0;
    can_filter_init_struct.filter_mask_high = 0;
    can_filter_init_struct.filter_mask_low = 0;
    can_filter_init(CAN1, &can_filter_init_struct);

    can_filter_init_struct.filter_fifo = CAN_FILTER_FIFO1;
    can_filter_init_struct.filter_number = 1;
    can_filter_init(CAN1, &can_filter_init_struct);

    // interrupt priorities, needs to be a higher number (lower priority) than
    // motor control interrupts
    NVIC_SetPriority(CAN1_SE_IRQn, 5);
    NVIC_SetPriority(CAN1_RX0_IRQn, 5);
    NVIC_SetPriority(CAN1_RX1_IRQn, 5);
    NVIC_SetPriority(CAN1_TX_IRQn, 5);

    /* interrupt enable */
    can_interrupt_enable(CAN1, CAN_TCIEN_INT, TRUE);
    can_interrupt_enable(CAN1, CAN_RF0MIEN_INT, TRUE);
    can_interrupt_enable(CAN1, CAN_RF1MIEN_INT, TRUE);
    can_interrupt_enable(CAN1, CAN_ETRIEN_INT, TRUE);
    can_interrupt_enable(CAN1, CAN_EOIEN_INT, TRUE);
}

void sys_can_enable_IRQ(void)
{
    NVIC_EnableIRQ(CAN1_SE_IRQn);
    NVIC_EnableIRQ(CAN1_RX0_IRQn);
    NVIC_EnableIRQ(CAN1_RX1_IRQn);
    NVIC_EnableIRQ(CAN1_TX_IRQn);
}

void sys_can_disable_IRQ(void)
{
    NVIC_DisableIRQ(CAN1_SE_IRQn);
    NVIC_DisableIRQ(CAN1_RX0_IRQn);
    NVIC_DisableIRQ(CAN1_RX1_IRQn);
    NVIC_DisableIRQ(CAN1_TX_IRQn);
}

/*
  try to transmit a frame.
  return 1 for success, 0 for no space, -ve for error
 */
int16_t sys_can_transmit(const CanardCANFrame* txf)
{
    can_tx_message_type tx_message_struct;
    if (txf->id & CANARD_CAN_FRAME_EFF) {
        tx_message_struct.id_type = CAN_ID_EXTENDED;
        tx_message_struct.standard_id = 0;
        tx_message_struct.extended_id = txf->id & CANARD_CAN_EXT_ID_MASK;
    } else {
        tx_message_struct.id_type = CAN_ID_STANDARD;
        tx_message_struct.standard_id = txf->id & CANARD_CAN_STD_ID_MASK;
        tx_message_struct.extended_id = 0;
    }
    tx_message_struct.frame_type = CAN_TFT_DATA;
    tx_message_struct.dlc = txf->data_len;
    memcpy(tx_message_struct.data, txf->data, txf->data_len);
    const uint8_t transmit_mailbox = can_message_transmit(CAN1, &tx_message_struct);
    if (transmit_mailbox == CAN_TX_STATUS_NO_EMPTY) {
        return 0;
    }
    return 1;
}

/*
  check for an incoming frame
  return 1 for success, 0 for no frame, -ve for error
 */
int16_t sys_can_receive(CanardCANFrame *rx_frame)
{
    can_rx_message_type frm;
    bool have_frame = false;
    if (CAN1->rf0_bit.rf0mn) {
        can_message_receive(CAN1, CAN_RX_FIFO0, &frm);
        have_frame = true;
    } else if (CAN1->rf1_bit.rf1mn) {
        can_message_receive(CAN1, CAN_RX_FIFO1, &frm);
        have_frame = true;
    }
    if (!have_frame) {
        return 0;
    }
    if (frm.id_type == CAN_ID_EXTENDED) {
        rx_frame->id = frm.extended_id | CANARD_CAN_FRAME_EFF;
    } else if (frm.id_type == CAN_ID_STANDARD) {
        rx_frame->id = frm.standard_id;
    } else {
        return 0;
    }
    rx_frame->data_len = frm.dlc;
    memcpy(rx_frame->data, frm.data, rx_frame->data_len);
    return 1;
}

/**
  *  @brief  can1 interrupt function
  *  @param  none
  *  @retval none
  */
void CAN1_RX0_IRQHandler(void)
{
    DroneCAN_receiveFrame();
}

void CAN1_RX1_IRQHandler(void)
{
    DroneCAN_receiveFrame();
}

void CAN1_TX_IRQHandler(void)
{
    CAN1->tsts = CAN_TSTS_TM0TCF_VAL | CAN_TSTS_TM1TCF_VAL | CAN_TSTS_TM2TCF_VAL;
    DroneCAN_processTxQueue();
}

/**
  *  @brief  can1 interrupt function se
  *  @param  none
  *  @retval none
  */
void CAN1_SE_IRQHandler(void)
{
  __IO uint32_t err_index = 0;
  if (CAN1->ests_bit.etr)
  {
    err_index = CAN1->ests & 0x70;
    CAN1->msts = CAN_MSTS_EOIF_VAL;
    CAN1->ests = 0;
    /* error type is stuff error */
    if (err_index == 0x00000010)
    {
        /* when stuff error occur: in order to ensure communication normally,
           user must restart can or send a frame of highest priority message here
        */
    }
  }
}

uint32_t get_rtc_backup_register(uint8_t idx)
{
    return ertc_bpr_data_read(idx);
}

void set_rtc_backup_register(uint8_t idx, uint32_t value)
{
    ertc_bpr_data_write(idx, value);
}

/*
  setup a static port/pin
 */
void setup_portpin(uint16_t portpin, bool enable)
{
    const uint8_t port = portpin >> 8;
    const uint8_t pin = portpin & 0xff;
    const uint32_t pinshift = 1U<<pin;
    gpio_type *pport = port==0?GPIOA:GPIOB;

    if (enable) {
        pport->scr = pinshift;
    } else {
        pport->clr = pinshift;
    }
    
    gpio_init_type gpio_init_struct;

    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins = pinshift;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(pport, &gpio_init_struct);
}

#endif // DRONECAN_SUPPORT && defined(ARTERY)

