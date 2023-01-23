#![no_std]
#![no_main]

mod rs41_led;

use cortex_m_rt::entry;
use stm32f1xx_hal::{delay::Delay, pac, prelude::*};
#[allow(unused_imports)]
use panic_halt;

use crate::rs41_led::RS41_Led;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);

    let mut flash = dp.FLASH.constrain();
    let clocks = rcc.cfgr.sysclk(8.mhz()).freeze(&mut flash.acr);
    let mut delay = Delay::new(cp.SYST, clocks);




    let mut leds : RS41_Led;
    leds = RS41_Led::init(gpiob);
    loop {
        leds.led_r_on();
        delay.delay_ms(1_000_u16);
        leds.led_r_off();
        delay.delay_ms(1_000_u16);
        leds.led_g_on();
        delay.delay_ms(1_000_u16);
        leds.led_g_off();
        delay.delay_ms(1_000_u16);

    }
}
