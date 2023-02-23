#![no_std]
#![no_main]

use cortex_m_rt::entry; // The runtime
use embedded_hal::digital::v2::OutputPin; // the `set_high/low`function
use stm32f1xx_hal::{pac, prelude::*}; // STM32F1 specific functions
#[allow(unused_imports)]
use panic_halt; // When a panic occurs, stop the microcontroller

#[rtic::app(device = stm32f1xx_hal::pac)]
mod app {
    use stm32f1xx_hal::{
        gpio::{gpioa::*, gpiob::*, gpioc::*, Output, PinState, PushPull},
        pac,
        prelude::*,
        timer::{CounterMs, Event},
    };

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led_r: PB8<Output<PushPull>>,
        led_g: PB7<Output<PushPull>>,
        timer_handler: CounterMs<pac::TIM1>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        let clocks = rcc.cfgr.freeze(&mut flash.acr);

        let mut gpiob = cx.device.GPIOB.split();

        let ledr = gpiob
            .pb8
            .into_push_pull_output_with_state(&mut gpiob.crh, PinState::High);

        let ledg = gpiob
            .pb7
            .into_push_pull_output_with_state(&mut gpiob.crl, PinState::Low);



        let mut timer = cx.device.TIM1.counter_ms(&clocks);
        timer.start(1.secs()).unwrap();
        timer.listen(Event::Update);

        (
            Shared {},
            Local {
                led_r: ledr,
                led_g: ledg,
                timer_handler: timer,
            },
            init::Monotonics(),
        )
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            // DO NOT UNCOMMENT UNLESS YOU WANT TO LIFT THE BOOT0 PIN
            //cortex_m::asm::wfi();
        }
    }

}