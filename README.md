# AM32-MultiRotor-ESC-firmware
Firmware for ARM-based speed controllers
<p align="left">
  <a href="/LICENSE"><img src="https://img.shields.io/badge/license-GPL--3.0-brightgreen" alt="GitHub license" /></a>
</p>

The AM32 firmware is designed for STM32 ARM processors to control a brushless motor (BLDC).
The firmware is intended to be safe and fast with smooth fast startups and linear throttle. It is meant for use with multiple vehicle types and a flight controllers. The firmware can also be built with support for crawlers.

For crawler usage please read this wiki page: [Crawler Hardware](https://github.com/AlkaMotors/AM32-MultiRotor-ESC-firmware/wiki/Crawler-Hardware-and-AM32).

## Features

AM32 has the following features:

- Firmware upgradeable via Betaflight passthrough, single wire serial or Arduino
- Servo PWM, DShot (300, 600) motor protocol support
- Bi-directional DShot
- KISS standard ESC telemetry
- Variable PWM frequency
- Sinusoidal startup mode, which is designed to get larger motors up to speed

## Build Instructions

Download and install [Keil Community Edition](https://www.keil.arm.com/mdk-community/). Open the Keil project for the MCU you want in the "Keil projects" folder. Install any MCU packs if prompted.
Select the build target from the drop-down box and build the project. 

## Firmware Release & Configuration Tool

The latest release of the firmware can be found [here](https://am32.ca/downloads).

The primary configurator is the [AM32 Configurator](https://am32.ca)
which supports web browser based configuration and firmware update.

You can also use a desktop configurator which you can download from here: [WINDOWS](https://drive.google.com/file/d/16kaPek9umz7fQFunzBeW4pp2LgT6_E5o/view?usp=drive_link), [LINUX](https://drive.google.com/file/d/1QtSKwp3RT6sncPADsPkmdasGqNIk68HH/view?usp=sharing).

Alternatively you can use the [Online-ESC Configurator](https://esc-configurator.com/) to flash or change settings with any web browser that supports web serial.

## Hardware

AM32 currently has support for STSPIN32F0, STM32F051, STM32G071, GD32E230, AT32F415, and AT32F421.
The CKS32F051 is not recommended due to too many random issues.
Target compatibility list can be found [here](https://github.com/am32-firmware/AM32/blob/main/Inc/targets.h).


## Installation & Bootloader

To use AM32 firmware on a blank ESC, a bootloader must first be installed using an ST-LINK, GD-LINK, CMSIS-DAP, or AT-LINK. The bootloader will be dependent on the MCU used on the ESC. Choose the bootloader that matches the MCU type and signal input pin of the ESC.
The compatibility chart has the bootloader pinouts listed.
Current bootloaders can be found [here](https://github.com/am32-firmware/AM32-bootloader).

After the bootloader has been installed, the main firmware can be installed either with the configuration tools and a Betaflight flight controller or a direct connection with a USB serial adapter modified for one wire.

To update an existing AM32 bootloader an update tool can be found [here](https://github.com/am32-firmware/AM32-unlocker).

## Support and Developers Channel

There are two ways you can get support or participate in improving AM32. We have a Discord server [here](https://discord.gg/h7ddYMmEVV).

**Note:** Please wait around long enough for a reply - sometimes people are out flying, asleep, or at work and can't answer immediately. 

If you wish to support the project please join the [Patreon](https://www.patreon.com/user?u=44228479).

## Sponsors

The AM32 project would not have made this far without help from the following sponsors:

Holmes Hobbies - https://holmeshobbies.com/ - The project would not be where it is today without the support of Holmes Hobbies. Check out the Crawlmaster V2 for the best AM32 experience!

Repeat Robotics - https://repeat-robotics.com/ - Bringing AM32 ESCs to the fighting robot community!

Quaternium - https://www.quaternium.com/ - Firmware development support and hardware donations.

Airbot - Many hardware donations.

NeutronRC - Hardware, AM32 promotion, and schematics.

Aikon - Hardware donations and schematics.

Skystars - Hardware and taking a chance on the first commercial AM32 ESCs.

Diatone - Hardware donations.

T-motor - Motor and hardware donations.

HLGRC - Hardware donations.

## Contributors

A big thanks to all those who contributed time, advice, and code to the AM32 project:

- Un!t
- Hugo Chiang (Dusking)
- Michael Keller (Mikeller)
- ColinNiu
- Jacob Walser

And for feedback from pilots and drivers:

- Jye Smith
- Markus Gritsch
- Voodoobrew

(and many more)
