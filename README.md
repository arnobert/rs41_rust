# rs41-rs

### This repo contains a new approach to a firmware for the Vaisala RS41 radiosonde, written entirely in Rust.

:warning: 2024-05-03: Today I captured a RS41 with a new microcontroller: **STM32L412**. 
This repo is developed on the **old** revision (with STM32F100). Do not attempt to flash a new revision with this firmware!

Template for the **new** revison (at the moment nothing more than a blinky): https://github.com/arnobert/rs41_v2-rs

---

## Obtaining the sonde
The web contains lots of information about how to get and program a RS41.
I am using a standard NXP LPC Link 2 with CMSIS-DAP firmware connected to the 
sonde with the adapter (see KiCad project in the hw folder).

**_Please unsolder and remove the antenna (steel wire next to the 10 pin connector) before first programming!_** 

I have additionally mounted a SMA plug to connect to a SDR (LimeSDR) or analyzer, with a 10 dB pad between.

---
## Sonde hardware
bazjo did an amazing job in reverse-engineering the RS41's PCB.
Please read his documentation: https://github.com/bazjo/radiosonde_hardware/tree/master/Vaisala_RS41

---

### Currently working features:
☑️ Flashing the sonde

☑️ RTIC running

☑️ LEDs indicating status:
* Blinking red LED indicates missing GPS lock
* Green LED indicates communication between STM32 and ublox

☑️ Power button and battery voltage (reading ADC)

☑️ Radio configuration:
  - frequency
  - tx power
  - modulation
  - GPIOs (heater)
    
☑️ Generating ubx messages

☑️ Sending ubx messages to u-blox GPS receiver and parsing responses

☑️ Transmitting data via (G)FSK and Hellschreiber mode
* Coordinates and height
* Callsign
* UTC
* GPS status

The (G)FSK mode also transmits
* sequence number
* CRC

---

## Unlocking the flash:
I am running a stock Arch Linux. Other distros or even OSs have not been tested yet.

### First terminal: open openocd
```
openocd -f interface/cmsis-dap.cfg -c "transport select swd"  -f target/stm32f1x.cfg
```

### Second terminal: commands to openocd and unlock:
```
telnet 127.0.0.1 4444

> flash protect 0 0 15 off
```

### For some sondes the command above did not work. You can try:
```
telnet 127.0.0.1 4444

> reset halt
> stm32f1x unlock 0
```

---

## Building the firmware

```
$ git clone https://github.com/arnobert/rs41_rust
$ cd rs41_rust
$ cargo fetch
$ cargo build
```
With Hellschreiber mode:
```
$ cargo build --features "hell"
```

Flashing the sonde, GFSK/HELL:
```
$ cargo embed
$ cargo embed --features "hell"
```
Cargo embed auto-detects the LPC-Link.
Other programmers might also work, but have not been tested.