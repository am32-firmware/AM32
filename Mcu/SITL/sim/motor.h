/*
  motor.h - BLDC motor, bridge and battery simulation for AM32 SITL
 */

#pragma once

#include <stdint.h>

void motor_init(void);

// advance the electrical/mechanical model by dt_ns. Called from the sim
// thread only
void motor_step(uint64_t now_ns, uint32_t dt_ns);

// 1Hz state line on stderr for --verbose
void motor_print_state(uint64_t now_ns, float time_ratio);

// direct state access for offline tests
void motor_get_state(double* theta, double* omega, double i[3]);

// snapshot for the state streaming port (sitl_state.c), sim thread only
void motor_get_live_state(float* omega, float* theta, float* theta_e,
                          float i[3], float v[3], float* vbus, float* ibus);

// re-derive cached values after a runtime config reload
void motor_config_changed(void);

// accumulate iu,iv,iw,vu,vv,vw,vbus,ibus for averaged state sampling
void motor_add_signals(double acc[8]);

// accumulate audio signals (torque, summed |i|) for the audio stream
void motor_add_audio(double acc[2]);

// called from the firmware main loop for debug tracing
void motor_log_mainloop(void);

// commutation debug logging, called from comStep
void motor_log_commutation(int step);

// generic event logging into the same debug ring
enum motor_ev {
    MEV_COMMUTATE = 0, // a = step
    MEV_EDGE, // comparator edge: a = new level, b = IMR set, c = pended
    MEV_COMP_BLANKED, // comp irq discarded by blanking: a = CNT, b = ci/2
    MEV_COMP_RUN, // comp irq calling interruptRoutine: a = CNT
    MEV_ZC_ACCEPT, // interruptRoutine armed COM timer: a = waitTime
    MEV_MAINLOOP, // firmware main loop iteration: a = avg, b = last_avg, c = gap us
};
void motor_log_event(int kind, uint32_t a, uint32_t b, uint32_t c);
