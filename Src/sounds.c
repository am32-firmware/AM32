/*
 * sounds.c
 *
 *  Created on: May 13, 2020
 *      Author: Alka
 */

#include "sounds.h"
#include "common.h"
#include "eeprom.h"
#include "functions.h"
#include "peripherals.h"
#include "phaseouts.h"
#include "targets.h"

#ifndef ERASED_FLASH_BYTE
#define ERASED_FLASH_BYTE  0xFF
#endif

uint8_t beep_volume;

void pause(uint16_t ms)
{
    SET_DUTY_CYCLE_ALL(0);
    delayMillis(ms);
    SET_DUTY_CYCLE_ALL(beep_volume); // volume of the beep, (duty cycle) don't go
                                     // above 25 out of 2000
}

void setVolume(uint8_t volume)
{
    if (volume > 11) {
        volume = 11;
    }
    beep_volume = volume * 2; // volume variable from 0 - 11 equates to CCR value of 0-22
}

void setCaptureCompare()
{
    SET_DUTY_CYCLE_ALL(beep_volume); // volume of the beep, (duty cycle) don't go
                                     // above 25 out of 2000
}

void playBJNote(uint16_t freq, uint16_t bduration)
{ // hz and ms
    uint16_t timerOne_reload = TIM1_AUTORELOAD;

    SET_PRESCALER_PWM(10);
    timerOne_reload = CPU_FREQUENCY_MHZ*100000 / freq;

    SET_AUTO_RELOAD_PWM(timerOne_reload);
    SET_DUTY_CYCLE_ALL(beep_volume * timerOne_reload / TIM1_AUTORELOAD); // volume of the beep, (duty cycle) don't
                                                                         // go above 25 out of 2000
    delayMillis(bduration);
}

uint16_t getBlueJayNoteFrequency(uint8_t bjarrayfreq)
{
    return 11000000 / (bjarrayfreq * 247 + 4000);
}

void playBlueJayTune()
{
    uint8_t full_time_count = 0;
    uint16_t duration;
    uint16_t frequency;
    comStep(3);
    // read_flash_bin(blueJayTuneBuffer , eeprom_address + 48 , 128);
    // first 4 bytes are reserved for rtttl duration, octave, beat, tempo
    for (int i = 4; i < 128; i += 2) {
        RELOAD_WATCHDOG_COUNTER();
        signaltimeout = 0;

        if (eepromBuffer.tune[i] == 255) {
            full_time_count++;

        } else {
            if (eepromBuffer.tune[i + 1] == 0) {
                duration = full_time_count * 254 + eepromBuffer.tune[i];
                SET_DUTY_CYCLE_ALL(0);
                delayMillis(duration);
            } else {
                frequency = getBlueJayNoteFrequency(eepromBuffer.tune[i + 1]);
                duration = ((full_time_count * 254 + eepromBuffer.tune[i]) * (100000 / frequency)) / 100;
                playBJNote(frequency, duration);
            }
            full_time_count = 0;
        }
    }
    allOff(); // turn all channels low again
    SET_PRESCALER_PWM(0); // set prescaler back to 0.
    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
    signaltimeout = 0;
    RELOAD_WATCHDOG_COUNTER();
}

void playStartupTune()
{
    __disable_irq();

    if (eepromBuffer.tune[0] != ERASED_FLASH_BYTE) {
        playBlueJayTune();
    } else {
        SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
        setCaptureCompare();
        comStep(3); // activate a pwm channel
        SET_PRESCALER_PWM(55); // frequency of beep
        delayMillis(200); // duration of beep

        comStep(5);
        SET_PRESCALER_PWM(40); // next beep is higher frequency
        delayMillis(200);

        comStep(6);
        SET_PRESCALER_PWM(25); // higher again..
        delayMillis(200);

        allOff(); // turn all channels low again
        SET_PRESCALER_PWM(0); // set prescaler back to 0.
        signaltimeout = 0;
    }

    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
    __enable_irq();
}

void playBrushedStartupTune()
{
    __disable_irq();
    SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
    setCaptureCompare();
    comStep(1); // activate a pwm channel
    SET_PRESCALER_PWM(40); // frequency of beep
    delayMillis(300); // duration of beep
    comStep(2); // activate a pwm channel
    SET_PRESCALER_PWM(30); // frequency of beep
    delayMillis(300); // duration of beep
    comStep(3); // activate a pwm channel
    SET_PRESCALER_PWM(25); // frequency of beep
    delayMillis(300); // duration of beep
    comStep(4);
    SET_PRESCALER_PWM(20); // higher again..
    delayMillis(300);
    allOff(); // turn all channels low again
    SET_PRESCALER_PWM(0); // set prescaler back to 0.
    signaltimeout = 0;
    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
    __enable_irq();
}

void playDuskingTune()
{
    setCaptureCompare();
    SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
    comStep(2); // activate a pwm channel
    SET_PRESCALER_PWM(60); // frequency of beep
    delayMillis(200); // duration of beep
    SET_PRESCALER_PWM(55); // next beep is higher frequency
    delayMillis(150);
    SET_PRESCALER_PWM(50); // higher again..
    delayMillis(150);
    SET_PRESCALER_PWM(45); // frequency of beep
    delayMillis(100); // duration of beep
    SET_PRESCALER_PWM(50); // next beep is higher frequency
    delayMillis(100);
    SET_PRESCALER_PWM(55); // higher again..
    delayMillis(100);
    SET_PRESCALER_PWM(25); // higher again..
    delayMillis(200);
    SET_PRESCALER_PWM(55); // higher again..
    delayMillis(150);
    allOff(); // turn all channels low again
    SET_PRESCALER_PWM(0); // set prescaler back to 0.
    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
}

void playInputTune2()
{
    SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
    __disable_irq();
    RELOAD_WATCHDOG_COUNTER();
    SET_PRESCALER_PWM(60);
    setCaptureCompare();
    comStep(1);
    delayMillis(75);
    SET_PRESCALER_PWM(80);
    delayMillis(75);
    SET_PRESCALER_PWM(90);
    RELOAD_WATCHDOG_COUNTER();
    delayMillis(75);
    allOff();
    SET_PRESCALER_PWM(0);
    signaltimeout = 0;
    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
    __enable_irq();
}

void playInputTune()
{
    __disable_irq();
    SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
    RELOAD_WATCHDOG_COUNTER();
    SET_PRESCALER_PWM(80);
    setCaptureCompare();
    comStep(3);
    delayMillis(100);
    SET_PRESCALER_PWM(70);
    delayMillis(100);
    SET_PRESCALER_PWM(40);
    delayMillis(100);
    allOff();
    SET_PRESCALER_PWM(0);
    signaltimeout = 0;
    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
    __enable_irq();
}

void playDefaultTone()
{
    SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
    SET_PRESCALER_PWM(50);
    setCaptureCompare();
    comStep(2);
    delayMillis(150);
    RELOAD_WATCHDOG_COUNTER();
    SET_PRESCALER_PWM(30);
    delayMillis(150);
    allOff();
    SET_PRESCALER_PWM(0);
    signaltimeout = 0;
    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
}

void playChangedTone()
{
    SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
    SET_PRESCALER_PWM(40);
    setCaptureCompare();
    comStep(2);
    delayMillis(150);
    RELOAD_WATCHDOG_COUNTER();
    SET_PRESCALER_PWM(80);
    delayMillis(150);
    allOff();
    SET_PRESCALER_PWM(0);
    signaltimeout = 0;
    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
}

void playBeaconTune3()
{
    SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
    __disable_irq();
    setCaptureCompare();
    for (int i = 119; i > 0; i = i - 2) {
        RELOAD_WATCHDOG_COUNTER();
        comStep(i / 20);
        SET_PRESCALER_PWM(10 + (i / 2));
        delayMillis(10);
    }
    allOff();
    SET_PRESCALER_PWM(0);
    signaltimeout = 0;
    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
    __enable_irq();
}
