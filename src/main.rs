#![no_std]
#![no_main]

use cortex_m_rt::entry; // The runtime
use embedded_hal::digital::v2::OutputPin; // the `set_high/low`function
use stm32f1xx_hal::{pac, prelude::*}; // STM32F1 specific functions
#[allow(unused_imports)]
use panic_halt; // When a panic occurs, stop the microcontroller

#[rtic::app(device = stm32f1xx_hal::pac)]
mod app {
    //use microbit as _;
    use rtt_target::{rprintln, rtt_init_print};

    use stm32f1xx_hal::{
        gpio::{gpioa::*, gpiob::*, gpioc::*, Output, PinState, PushPull},
        pac,
        prelude::*,
        timer::{CounterMs, Event},
    };
    use stm32f1xx_hal::serial::{Config, Serial};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led_r: PB8<Output<PushPull>>,
        led_g: PB7<Output<PushPull>>,
        timer_handler: CounterMs<pac::TIM1>,
        //serial: Serial<pac::USART1, pac::USART1>
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        fn init_profile(device: &pac::Peripherals) {
            // On development, keep the DBG module powered on during wfi()
            device.DBGMCU.cr.modify(|_, w| w.dbg_standby().set_bit());
            device.DBGMCU.cr.modify(|_, w| w.dbg_sleep().set_bit());
            device.DBGMCU.cr.modify(|_, w| w.dbg_stop().set_bit());
        }

        //init_profile(&cx.device);

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

        rtt_init_print!();

        let mut timer = cx.device.TIM1.counter_ms(&clocks);
        timer.start(1.secs()).unwrap();
        timer.listen(Event::Update);

        // USART1
        //let tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        //let rx = gpioa.pa10;

        // Set up the usart device. Taks ownership over the USART register and tx/rx pins. The rest of
        // the registers are used to enable and configure the device.
/*        let mut serial = Serial::usart1(
            cx.device.USART1,
            (tx, rx),
            &mut afio.mapr,
            Config::default().baudrate(9600.bps()),
            clocks,
            &mut rcc.apb1,
        );*/

        (
            Shared {},
            Local {
                led_r: ledr,
                led_g: ledg,
                timer_handler: timer,
                //serial
            },
            init::Monotonics(),
        )
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            //rprintln!("kadse");
            // DO NOT UNCOMMENT UNLESS YOU WANT TO LIFT THE BOOT0 PIN
            //cortex_m::asm::wfi();
        }
    }

}