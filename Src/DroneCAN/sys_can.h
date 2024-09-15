/*
  sys_can.h - MCU specific CAN API
 */

#pragma once

#include <canard.h>

/*
  CAN statistics shared by low level and high level code
 */
struct CANStats {
    uint32_t num_commands;
    uint32_t total_commands;
    uint32_t num_receive;
    uint32_t num_tx_interrupts;
    uint32_t num_rx_interrupts;
    uint32_t rx_errors;
    uint32_t esr;
    uint32_t rxframe_error;
    int32_t rx_ecode;
    uint32_t should_accept;
    uint32_t on_receive;
    uint64_t last_raw_command_us;
};

extern struct CANStats canstats;


/*
  disable/enable CAN interripts
 */
void sys_can_disable_IRQ(void);
void sys_can_enable_IRQ(void);

/*
  get a 16 byte unique ID for this node
*/
void sys_can_getUniqueID(uint8_t id[16]);

/*
  initialise CAN hardware
 */
void sys_can_init(void);

/*
  called from CAN IRQ indicating we may have a free TX slot
 */
extern void DroneCAN_processTxQueue();

/*
  called from CAN IRQ indicating we have a new frame waiting
 */
extern void DroneCAN_receiveFrame();

/*
  try to transmit a frame.
  return 1 for success, 0 for no space, -ve for error
 */
int16_t sys_can_transmit(const CanardCANFrame* txf);

/*
  check for an incoming frame
  return 1 for success, 0 for no frame, -ve for error
 */
int16_t sys_can_receive(CanardCANFrame *rx_frame);
