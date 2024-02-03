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
    if (volume < 0) {
        volume = 0;
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
    timerOne_reload = 4800000 / freq;

    SET_AUTO_RELOAD_PWM(timerOne_reload);
    SET_DUTY_CYCLE_ALL(beep_volume * timerOne_reload / TIM1_AUTORELOAD); // volume of the beep, (duty cycle) don't
                                                                         // go above 25 out of 2000
    delayMillis(bduration);
}

uint16_t getBlueJayNoteFrequency(uint8_t bjarrayfreq)
{
    return 10000000 / (bjarrayfreq * 247 + 4000);
}

void playBlueJayTune()
{
    uint8_t full_time_count = 0;
    uint16_t duration;
    uint16_t frequency;
    comStep(3);
    // read_flash_bin(blueJayTuneBuffer , EEPROM_START_ADD + 48 , 128);
    for (int i = 52; i < 176; i += 2) {
        RELOAD_WATCHDOG_COUNTER();
        signaltimeout = 0;

        if (eepromBuffer[i] == 255) {
            full_time_count++;

        } else {
            if (eepromBuffer[i + 1] == 0) {
                duration = full_time_count * 254 + eepromBuffer[i];
                SET_DUTY_CYCLE_ALL(0);
                delayMillis(duration);
            } else {
                frequency = getBlueJayNoteFrequency(eepromBuffer[i + 1]);
                duration = ((full_time_count * 254 + eepromBuffer[i]) * (100000 / frequency)) / 100;
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

    uint8_t value = *(uint8_t*)(EEPROM_START_ADD + 48);
    if (value != 0xFF) {
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

// #ifdef GIGADEVICES

// void pause(uint16_t ms){
// 	TIMER_CH0CV(TIMER0) = 0; // volume of the beep, (duty cycle) don't go
// above 25 out of 2000 	TIMER_CH1CV(TIMER0) = 0; 	TIMER_CH2CV(TIMER0) = 0;

// 	delayMillis(ms);
// 	TIMER_CH0CV(TIMER0) = beep_volume; // volume of the beep, (duty cycle)
// don't go above 25 out of 2000 	TIMER_CH1CV(TIMER0) = beep_volume;
// 	TIMER_CH2CV(TIMER0) = beep_volume;
// }

// void setVolume(uint8_t volume){
// 	if(volume > 11){
// 		volume = 11;
// 	}
// 	if(volume < 0){
// 		volume = 0;
// 	}
// 	beep_volume = volume * 2;           // volume variable from 0 - 11
// equates to CCR value of 0-22
// }

// void setCaptureCompare(){
// 	TIMER_CH0CV(TIMER0) = beep_volume; // volume of the beep, (duty cycle)
// don't go above 25 out of 2000 	TIMER_CH1CV(TIMER0) = beep_volume;
// 	TIMER_CH2CV(TIMER0) = beep_volume;
// }

// void playBJNote(uint16_t freq, uint16_t bduration){        // hz and ms
// 	uint16_t timerOne_reload = TIM1_AUTORELOAD;

// 	TIMER_PSC(TIMER0) = 10;
// 	timerOne_reload = 4800000 / freq;

// 	TIMER_CAR(TIMER0) = timerOne_reload;
// 	TIMER_CH0CV(TIMER0) = beep_volume * timerOne_reload /TIM1_AUTORELOAD ;
// // volume of the beep, (duty cycle) don't go above 25 out of 2000
// 	TIMER_CH1CV(TIMER0) = beep_volume * timerOne_reload /TIM1_AUTORELOAD;
// 	TIMER_CH2CV(TIMER0) = beep_volume * timerOne_reload /TIM1_AUTORELOAD;

// 	delayMillis(bduration);
// }

// uint16_t getBlueJayNoteFrequency(uint8_t bjarrayfreq){
// 	return 10000000/(bjarrayfreq * 247 + 4000);
// }

// void playBlueJayTune(){
// 	uint8_t full_time_count = 0;
// 	uint16_t duration;
// 	float frequency;
// 	comStep(3);
// 	//read_flash_bin(blueJayTuneBuffer , EEPROM_START_ADD + 48 , 128);
// 	for(int i = 52 ; i < 176 ; i+=2){
// 		fwdgt_counter_reload();
// 		signaltimeout = 0;

// 		if(eepromBuffer[i] == 255){
// 			full_time_count++;

// 		}else{
// 			if(eepromBuffer[i+1] == 0){
// 				duration = full_time_count * 254 +
// eepromBuffer[i]; 				TIMER_CH0CV(TIMER0) = 0 ; // 				TIMER_CH1CV(TIMER0) = 0;
// 				TIMER_CH2CV(TIMER0) = 0;
// 				delayMillis(duration);
// 			}else{
// 			frequency = getBlueJayNoteFrequency(eepromBuffer[i+1]);
// 			duration= (full_time_count * 254 + eepromBuffer[i])  *
// (float)(1000 / frequency); 			playBJNote(frequency, duration);
// 			}
// 			full_time_count = 0;
// 		}
// 	}
// 	allOff();                // turn all channels low again
// 	TIMER_PSC(TIMER0) = 0;           // set prescaler back to 0.
// 	TIMER_CAR(TIMER0) = TIMER1_MAX_ARR;
// 	signaltimeout = 0;
// 	fwdgt_counter_reload();
// }

// void playStartupTune(){
// 	__disable_irq();

// 	uint8_t value = *(uint8_t*)(EEPROM_START_ADD+48);
// 		if(value != 0xFF){
// 		playBlueJayTune();
// 		}else{
// 	TIMER_CAR(TIMER0) = TIM1_AUTORELOAD;
// 	setCaptureCompare();
// 	comStep(3);       // activate a pwm channel

// 	TIMER_PSC(TIMER0) = 55;        // frequency of beep
//     delayMillis(200);         // duration of beep
//     comStep(5);

//     TIMER_PSC(TIMER0) = 40;            // next beep is higher frequency
// 	delayMillis(200);

// 	comStep(6);
// 	TIMER_PSC(TIMER0) = 25;         // higher again..
// 	delayMillis(200);
// 	allOff();                // turn all channels low again
// 	TIMER_PSC(TIMER0) = 0;           // set prescaler back to 0.
// 	signaltimeout = 0;
// 	}

// 	TIMER_CAR(TIMER0) = TIMER1_MAX_ARR;
// 	__enable_irq();
// }

// void playBrushedStartupTune(){
// 	__disable_irq();
// 	TIMER_CAR(TIMER0) = TIM1_AUTORELOAD;
// 	setCaptureCompare();
// 	comStep(1);       // activate a pwm channel
// 	TIMER_PSC(TIMER0) = 40;        // frequency of beep
//     delayMillis(300);         // duration of beep
// 	comStep(2);       // activate a pwm channel
// 	TIMER_PSC(TIMER0) = 30;        // frequency of beep
//     delayMillis(300);         // duration of beep
// 	comStep(3);       // activate a pwm channel
// 	TIMER_PSC(TIMER0) = 25;        // frequency of beep
//     delayMillis(300);         // duration of beep
//     comStep(4);
// 	TIMER_PSC(TIMER0) = 20;         // higher again..
// 	delayMillis(300);
// 	allOff();                // turn all channels low again
// 	TIMER_PSC(TIMER0) = 0;           // set prescaler back to 0.
// 	signaltimeout = 0;
// 	TIMER_CAR(TIMER0) = TIMER1_MAX_ARR;
// 	__enable_irq();
// }

// void playDuskingTune(){
// 	setCaptureCompare();
// 	TIMER_CAR(TIMER0) = TIM1_AUTORELOAD;
// 	comStep(2);       // activate a pwm channel
// 	TIMER_PSC(TIMER0) = 60;        // frequency of beep
//     delayMillis(200);         // duration of beep
// 	TIMER_PSC(TIMER0) = 55;            // next beep is higher frequency
// 	delayMillis(150);
// 	TIMER_PSC(TIMER0) = 50;         // higher again..
// 	delayMillis(150);
// 	TIMER_PSC(TIMER0) = 45;        // frequency of beep
//     delayMillis(100);         // duration of beep
// 	TIMER_PSC(TIMER0) = 50;            // next beep is higher frequency
// 	delayMillis(100);
// 	TIMER_PSC(TIMER0) = 55;         // higher again..
// 	delayMillis(100);
// 	TIMER_PSC(TIMER0) = 25;         // higher again..
// 	delayMillis(200);
// 	TIMER_PSC(TIMER0) = 55;         // higher again..
// 	delayMillis(150);
// 	allOff();                // turn all channels low again
// 	TIMER_PSC(TIMER0) = 0;           // set prescaler back to 0.
// 	TIMER_CAR(TIMER0) = TIMER1_MAX_ARR;
// }

// void playInputTune2(){
//     TIMER_CAR(TIMER0) = TIM1_AUTORELOAD;
// 	__disable_irq();
//     fwdgt_counter_reload();
// 	TIMER_PSC(TIMER0) = 60;
// 	setCaptureCompare();
// 	comStep(1);
// 	delayMillis(75);
// 	TIMER_PSC(TIMER0) = 80;
// 	delayMillis(75);
// 	TIMER_PSC(TIMER0) = 90;
// 	fwdgt_counter_reload();
// 	delayMillis(75);
// 	allOff();
// 	TIMER_PSC(TIMER0) = 0;
// 	signaltimeout = 0;
// 	TIMER_CAR(TIMER0) = TIMER1_MAX_ARR;
// 	__enable_irq();
// }

// void playInputTune(){
// 	__disable_irq();
// 	TIMER_CAR(TIMER0) = TIM1_AUTORELOAD;
// 	 fwdgt_counter_reload();
// 	TIMER_PSC(TIMER0) = 80;
// 	setCaptureCompare();
// 	comStep(3);
// 	delayMillis(100);
// 	TIMER_PSC(TIMER0) = 70;
// 	delayMillis(100);
// 	TIMER_PSC(TIMER0) = 40;
// 	delayMillis(100);
// 	allOff();
// 	TIMER_PSC(TIMER0) = 0;
// 	signaltimeout = 0;
// 	TIMER_CAR(TIMER0) = TIMER1_MAX_ARR;
// 	__enable_irq();
// }

// void playDefaultTone(){
// 	TIMER_CAR(TIMER0) = TIM1_AUTORELOAD;
// 	TIMER_PSC(TIMER0) = 50;
// 	setCaptureCompare();
// 	comStep(2);
// 	delayMillis(150);
// 	 fwdgt_counter_reload();
// 	TIMER_PSC(TIMER0) = 30;
// 	delayMillis(150);
// 	allOff();
// 	TIMER_PSC(TIMER0) = 0;
// 	signaltimeout = 0;
// 	TIMER_CAR(TIMER0) = TIMER1_MAX_ARR;

// }

// void playChangedTone(){
// 	TIMER_CAR(TIMER0) = TIM1_AUTORELOAD;
// 	TIMER_PSC(TIMER0) = 40;
// 	setCaptureCompare();
// 	comStep(2);
// 	delayMillis(150);
// 	fwdgt_counter_reload();
// 	TIMER_PSC(TIMER0) = 80;
// 	delayMillis(150);
// 	allOff();
// 	TIMER_PSC(TIMER0) = 0;
// 	signaltimeout = 0;
// 	TIMER_CAR(TIMER0) = TIMER1_MAX_ARR;

// }

// void playBeaconTune3(){
// 	TIMER_CAR(TIMER0) = TIM1_AUTORELOAD;
// 	__disable_irq();
// 	setCaptureCompare();
// 	for(int i = 119 ; i > 0 ; i = i- 2){
// 		fwdgt_counter_reload();
// 		comStep(i/20);
// 		TIMER_PSC(TIMER0) = 10+(i / 2);
// 		delayMillis(10);
// 	}
// 	allOff();
// 	TIMER_PSC(TIMER0) = 0;
// 	signaltimeout = 0;
// 	TIMER_CAR(TIMER0) = TIMER1_MAX_ARR;
// 	__enable_irq();
// }
// #endif

// #ifdef ARTERY
// void pause(uint16_t ms){
// 	TMR1->c1dt = 0; // volume of the beep, (duty cycle) don't go above 25
// out of 2000 	TMR1->c2dt = 0; 	TMR1->c3dt = 0;

// 	delayMillis(ms);
// 	TMR1->c1dt = beep_volume; // volume of the beep, (duty cycle) don't go
// above 25 out of 2000 	TMR1->c2dt = beep_volume; 	TMR1->c3dt = beep_volume;
// }

// void setVolume(uint8_t volume){
// 	if(volume > 11){
// 		volume = 11;
// 	}
// 	if(volume < 0){
// 		volume = 0;
// 	}
// 	beep_volume = volume * 2;           // volume variable from 0 - 11
// equates to CCR value of 0-22
// }

// void setCaptureCompare(){
// 	TMR1->c1dt = beep_volume; // volume of the beep, (duty cycle) don't go
// above 25 out of 2000 	TMR1->c2dt = beep_volume; 	TMR1->c3dt = beep_volume;
// }

// void playBJNote(uint16_t freq, uint16_t bduration){        // hz and ms
// 	uint16_t timerOne_reload = TIM1_AUTORELOAD;

// 	TMR1->div = 10;
// 	timerOne_reload = 4800000 / freq;

// 	TMR1->pr = timerOne_reload;
// 	TMR1->c1dt = beep_volume * timerOne_reload /TIM1_AUTORELOAD ; // volume
// of the beep, (duty cycle) don't go above 25 out of 2000 	TMR1->c2dt =
// beep_volume * timerOne_reload /TIM1_AUTORELOAD; 	TMR1->c3dt = beep_volume *
// timerOne_reload /TIM1_AUTORELOAD;

// 	delayMillis(bduration);
// }

// uint16_t getBlueJayNoteFrequency(uint8_t bjarrayfreq){
// 	return 10000000/(bjarrayfreq * 247 + 4000);
// }

// void playBlueJayTune(){
// 	uint8_t full_time_count = 0;
// 	uint16_t duration;
// 	uint16_t frequency;
// 	comStep(3);
// 	//read_flash_bin(blueJayTuneBuffer , EEPROM_START_ADD + 48 , 128);
// 	for(int i = 52 ; i < 176 ; i+=2){
// 	WDT->cmd = WDT_CMD_RELOAD;
// 		signaltimeout = 0;

// 		if(eepromBuffer[i] == 255){
// 			full_time_count++;

// 		}else{
// 			if(eepromBuffer[i+1] == 0){
// 				duration = full_time_count * 254 +
// eepromBuffer[i]; 				TMR1->c1dt = 0 ; // 				TMR1->c2dt = 0; 				TMR1->c3dt = 0;
// 				delayMillis(duration);
// 			}else{
// 			frequency = getBlueJayNoteFrequency(eepromBuffer[i+1]);
// 			duration= ((full_time_count * 254 + eepromBuffer[i])  *
// (100000 / frequency)) / 100; 			playBJNote(frequency, duration);
// 			}
// 			full_time_count = 0;
// 		}
// 	}
// 	allOff();                // turn all channels low again
// 	TMR1->div = 0;           // set prescaler back to 0.
// 	TMR1->pr = TIMER1_MAX_ARR;
// 	signaltimeout = 0;
// WDT->cmd = WDT_CMD_RELOAD;
// }

// void playStartupTune(){
// 	__disable_irq();

// 	uint8_t value = *(uint8_t*)(EEPROM_START_ADD+48);
// 		if(value != 0xFF){
// 		playBlueJayTune();
// 		}else{
// 	TMR1->pr = TIM1_AUTORELOAD;
// 	setCaptureCompare();
// 	comStep(3);       // activate a pwm channel
// WDT->cmd = WDT_CMD_RELOAD;
// 	TMR1->div = 55;        // frequency of beep
//     delayMillis(200);         // duration of beep
//     comStep(5);

//     TMR1->div = 40;            // next beep is higher frequency
// 	delayMillis(200);

// 	comStep(6);
// 	TMR1->div = 25;         // higher again..
// 	delayMillis(200);
// 	allOff();                // turn all channels low again
// 	TMR1->div = 0;           // set prescaler back to 0.
// 	signaltimeout = 0;
// 	}
// 	WDT->cmd = WDT_CMD_RELOAD;
// 	TMR1->pr = TIMER1_MAX_ARR;
// 	__enable_irq();
// }

// void playBrushedStartupTune(){
// 	__disable_irq();
// 	TMR1->pr = TIM1_AUTORELOAD;
// 	setCaptureCompare();
// 	comStep(1);       // activate a pwm channel
// 	TMR1->div = 40;        // frequency of beep
//     delayMillis(300);         // duration of beep
// 	comStep(2);       // activate a pwm channel
// 	TMR1->div = 30;        // frequency of beep
//     delayMillis(300);         // duration of beep
// 	comStep(3);       // activate a pwm channel
// 	TMR1->div = 25;        // frequency of beep
//     delayMillis(300);         // duration of beep
//     comStep(4);
// 	TMR1->div = 20;         // higher again..
// 	delayMillis(300);
// 	allOff();                // turn all channels low again
// 	TMR1->div = 0;           // set prescaler back to 0.
// 	signaltimeout = 0;
// 	TMR1->pr = TIMER1_MAX_ARR;
// 	__enable_irq();
// }

// void playDuskingTune(){
// 	setCaptureCompare();
// 	TMR1->pr = TIM1_AUTORELOAD;
// 	comStep(2);       // activate a pwm channel
// 	TMR1->div = 60;        // frequency of beep
//     delayMillis(200);         // duration of beep
// 	TMR1->div = 55;            // next beep is higher frequency
// 	delayMillis(150);
// 	TMR1->div = 50;         // higher again..
// 	delayMillis(150);
// 	TMR1->div = 45;        // frequency of beep
//     delayMillis(100);         // duration of beep
// 	TMR1->div = 50;            // next beep is higher frequency
// 	delayMillis(100);
// 	TMR1->div = 55;         // higher again..
// 	delayMillis(100);
// 	TMR1->div = 25;         // higher again..
// 	delayMillis(200);
// 	TMR1->div = 55;         // higher again..
// 	delayMillis(150);
// 	allOff();                // turn all channels low again
// 	TMR1->div = 0;           // set prescaler back to 0.
// 	TMR1->pr = TIMER1_MAX_ARR;
// }

// void playInputTune2(){
//     TMR1->pr = TIM1_AUTORELOAD;
// 	__disable_irq();
// WDT->cmd = WDT_CMD_RELOAD;
// 	TMR1->div = 60;
// 	setCaptureCompare();
// 	comStep(1);
// 	delayMillis(75);
// 	TMR1->div = 80;
// 	delayMillis(75);
// 	TMR1->div = 90;
// WDT->cmd = WDT_CMD_RELOAD;
// 	delayMillis(75);
// 	allOff();
// 	TMR1->div = 0;
// 	signaltimeout = 0;
// 	TMR1->pr = TIMER1_MAX_ARR;
// 	__enable_irq();
// }

// void playInputTune(){
// 	__disable_irq();
// 	TMR1->pr = TIM1_AUTORELOAD;
// WDT->cmd = WDT_CMD_RELOAD;
// 	TMR1->div = 80;
// 	setCaptureCompare();
// 	comStep(3);
// 	delayMillis(100);
// 	TMR1->div = 70;
// 	delayMillis(100);
// 	TMR1->div = 40;
// 	delayMillis(100);
// 	allOff();
// 	TMR1->div = 0;
// 	signaltimeout = 0;
// 	TMR1->pr = TIMER1_MAX_ARR;
// 	__enable_irq();
// }

// void playDefaultTone(){
// 	TMR1->pr = TIM1_AUTORELOAD;
// 	TMR1->div = 50;
// 	setCaptureCompare();
// 	comStep(2);
// 	delayMillis(150);
// WDT->cmd = WDT_CMD_RELOAD;
// 	TMR1->div = 30;
// 	delayMillis(150);
// 	allOff();
// 	TMR1->div = 0;
// 	signaltimeout = 0;
// 	TMR1->pr = TIMER1_MAX_ARR;

// }

// void playChangedTone(){
// 	TMR1->pr = TIM1_AUTORELOAD;
// 	TMR1->div = 40;
// 	setCaptureCompare();
// 	comStep(2);
// 	delayMillis(150);
// WDT->cmd = WDT_CMD_RELOAD;
// 	TMR1->div = 80;
// 	delayMillis(150);
// 	allOff();
// 	TMR1->div = 0;
// 	signaltimeout = 0;
// 	TMR1->pr = TIMER1_MAX_ARR;

// }

// void playBeaconTune3(){
// 	TMR1->pr = TIM1_AUTORELOAD;
// 	__disable_irq();
// 	setCaptureCompare();
// 	for(int i = 119 ; i > 0 ; i = i- 2){
// WDT->cmd = WDT_CMD_RELOAD;
// 		comStep(i/20);
// 		TMR1->div = 10+(i / 2);
// 		delayMillis(10);
// 	}
// 	allOff();
// 	TMR1->div = 0;
// 	signaltimeout = 0;
// 	TMR1->pr = TIMER1_MAX_ARR;
// 	__enable_irq();
// }
// #endif