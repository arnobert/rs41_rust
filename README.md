# rs41_rust

### This repo contains a new approach to a firmware for the Vaisala RS41 radiosonde, written entirely in Rust.

## This project is in *very* early stage of development at the moment and none ofthe basic features are working yet!

## Obtaining the sonde
The web contains lots of information about how to get and program a RS41.
I am using a standard NXP LPC Link 2 with CMSIS-DAP firmware works with an adapter.

**_Please unsolder and remove the antenna before first programming!_** 

I have additionally mounted a SMA plug to connect to a LimeSDR, with a 10 dB pad between.

## Sonde hardware
bazjo did an amazing job in reverse-engineering the RS41's PCB.
See documentation: https://github.com/bazjo/radiosonde_hardware/tree/master/Vaisala_RS41

## Goals, requirements
For a planned HAB launch the sonde has to transmit its location (coordinates and altitude).
We would like to use GFSK and (optional) Hellschreiber mode as the Si4032 transmitter also supports OOK.

## Building

```
$ git clone https://github.com/arnobert/rs41_rust
$ git clone https://github.com/arnobert/si4032_driver-rs
$ cd rs41_rust
$ cargo build
```

Flashing the sonde:
```
$ cargo embed
```
Cargo embed auto-detects the LPC Link.