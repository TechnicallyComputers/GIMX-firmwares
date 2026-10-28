This repo is for maintaining GIMX custom firmwares, using an adapted version of the Hisptoot Nintendo Switch firmware.  Here is my contributions to the Switch Pro controller firmware for the Arduino Microchip ATmega32U4 in the custom built GIMX adapter cable.

This firmware repository can be used to build and compile firmware with compatible GIMX ATmega32U4 devices and the GIMX software I maintain.  This will enable the ATmega32U4 to communicate with the NSW as a "Switch Pro" controller.

The original firmware seemed to have an issue with byte overflows causing disconnection / desync of communication between the ATmega32U4 and the Switch, which cause the Switch to no longer accept inputs from the Arduino.  
The goal of this project is to resolve that problem and provide a stable NSW firmware.

The EMUSWITCH firmware folder has changes I've made to add diagnostics and edit communication behaviours based on the original firmware by Hisptoot

Build instructions:

To build firmware for NSW:

`make -C EMUSWITCH`

To install it into local GIMX directory (GIMX must support EMUSWITCH)

`sudo install -Dm644 EMUSWITCH/emu.hex /usr/share/gimx/firmware/EMUSWITCH.hex`

To install into GIMX local repo directly

`cp EMUSWITCH/emu.hex ../GIMX/loader/firmware/EMUSWITCH.hex`
