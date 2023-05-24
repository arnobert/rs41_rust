# rs41_rust

### This repo contains a new approach to a firmware for the Vaisala RS41 radiosonde, written entirely in Rust.

## This project is in *very* early stage of development at the moment and only a few basic features are working at the moment (see below)!

## Obtaining the sonde
The web contains lots of information about how to get and program a RS41.
I am using a standard NXP LPC Link 2 with CMSIS-DAP firmware works with an adapter.

**_Please unsolder and remove the antenna before first programming!_** 

I have additionally mounted a SMA plug to connect to a SDR (LimeSDR) or analyzer, with a 10 dB pad between.

## Sonde hardware
bazjo did an amazing job in reverse-engineering the RS41's PCB.
Please read his documentation: https://github.com/bazjo/radiosonde_hardware/tree/master/Vaisala_RS41

## Goals, requirements
For a planned HAB launch the sonde has following requirements:
- Transmit **location** (coordinates and altitude)
- **Modulation**: GFSK and Hellschreiber mode
- RTTY optional (is going to be tricky with the Si4032)
- Reading environmental (temperature, pressure) sensor optional
- Additional UART port at external connector

### Already working:
- Flashing the sonde
- RTIC runs
- LEDs blinking
- Power button (reading ADC)
- Radio configuration:
  - frequency
  - tx power
  - modulation
  - ...
- Transmitting data via GFSK
- Hellschreiber mode (needs fine tuning)
- Generating ubx messages
- Sending ubx messages to u-blox GPS receiver

### Next steps:
- Parsing ubx messages from u-blox
- Improving GFSK receiver (GNU Radio)

## Building

```
$ git clone https://github.com/arnobert/rs41_rust
$ git clone https://github.com/arnobert/si4032_driver-rs
$ cd rs41_rust
$ cargo build
```
With Hellschreiber mode:
```
$ cargo build --features "hell"
```
With RTTY mode:
```
$ cargo build --features "rtty"
```
Flashing the sonde, GFSK/HELL/RTTY:
```
$ cargo embed
$ cargo embed --features "hell"
$ cargo embed --features "rtty"
```
Cargo embed auto-detects the LPC Link.