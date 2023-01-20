#![no_std]
#![no_main]

use cortex_m_rt::entry; // The runtime
use embedded_hal::digital::v2::OutputPin; // the `set_high/low`function
use stm32f1xx_hal::{delay::Delay, pac, prelude::*}; // STM32F1 specific functions
#[allow(unused_imports)]
use panic_halt; // When a panic occurs, stop the microcontroller

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut ledr = gpiob.pb8.into_push_pull_output(&mut gpiob.crh);
    let mut ledg = gpiob.pb7.into_push_pull_output(&mut gpiob.crl);

    let mut flash = dp.FLASH.constrain();
    let clocks = rcc.cfgr.sysclk(8.mhz()).freeze(&mut flash.acr);
    let mut delay = Delay::new(cp.SYST, clocks);

    loop {
        ledr.set_high().ok();
        delay.delay_ms(1_000_u16);
        ledr.set_low().ok();
        delay.delay_ms(1_000_u16);
        ledg.set_high().ok();
        delay.delay_ms(1_000_u16);
        ledg.set_low().ok();
       delay.delay_ms(1_000_u16);
    }
}
