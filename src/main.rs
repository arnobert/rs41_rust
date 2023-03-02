#![no_std]
#![no_main]

/*   RS-41 pin description:
PA:
00 : NFC IN
01 : MEAS OUT
02 : PULLUP HYG
03 : SPST2
04 : FRONTEND ??
05 : VBAT MON
06 : SW
07 : HEAT HUM 1
08 : RPM CLK
09 : GPS RX
10 : GPS TX
11 : NFC IN
12 : SHUTDOWN
13 : XDATA SWDIO
14 : XDATA SWCLK
15 : GPS RESETn

PB:
00 : NFC OUT
01 : REF R THERM
02 : SPI CS
03 : SPDT 1 - SENSOR MUX
04 : SPDT 2
05 : SPDT 3
06 : SPST 1
07 : LED GN
08 : LED RD
09 : HEAT HUM 2
10 : XDATA TX
11 : XDATA RX
12 : PULLUP TMP
13 : SPISCK
14 : SPI SDI
15 : SPI SDO

PC:
13 : CS RADIO
14 : SPST 3
15 : SPST 4

 */
use embedded_hal::spi::{Mode, Phase, Polarity};
pub const SPIMODE: Mode = Mode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

#[allow(unused_imports)]

use cortex_m_rt::entry;
use panic_halt;

#[rtic::app(device = stm32f1xx_hal::pac)]
mod app {
    use core::marker::PhantomData;
    use rtt_target::{rprintln, rtt_init_print};

    use stm32f1xx_hal::{
        gpio::{gpioa::*, gpiob::*, gpioc::*, Output, PinState, PushPull},
        pac,
        prelude::*,
        timer::{CounterMs, Event},
        serial::{Config, Serial},
        spi::{Pins, Spi, Spi1NoRemap},
    };
    use stm32f1xx_hal::gpio::{Alternate, Floating, Input};
    use crate::SPIMODE;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led_r: PB8<Output<PushPull>>,
        led_g: PB7<Output<PushPull>>,
        timer_handler: CounterMs<pac::TIM1>,
        //gps_serial: Serial<stm32f1xx_hal::pac::USART1, (PA9<Alternate<PushPull>>, PA10<Input<Floating>>)>,
        spi: Spi<stm32f1xx_hal::pac::SPI2,
                stm32f1xx_hal::spi::Spi2NoRemap,
                (PB13<Alternate<PushPull>>, PB14, PB15<Alternate<PushPull>>),
                u8 >,
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


        let p = pac::Peripherals::take().unwrap();
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        let clocks = rcc.cfgr.freeze(&mut flash.acr);

        let mut afio = cx.device.AFIO.constrain();

        let mut gpioa = cx.device.GPIOA.split();
        let mut gpiob = cx.device.GPIOB.split();
        let mut gpioc = cx.device.GPIOC.split();

        // LEDs
        let ledr = gpiob
            .pb8
            .into_push_pull_output_with_state(&mut gpiob.crh, PinState::High);

        let ledg = gpiob
            .pb7
            .into_push_pull_output_with_state(&mut gpiob.crl, PinState::Low);

        // SPI
        let spi_clk = gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh);
        let spi_sdo = gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh);
        let spi_sdi = gpiob.pb14;


        let mut rs_spi = Spi::spi2(
            cx.device.SPI2,
            (spi_clk, spi_sdi, spi_sdo),
            SPIMODE,
            1.MHz(),
            clocks,
        );


        rtt_init_print!();

        let mut timer = cx.device.TIM1.counter_ms(&clocks);
        timer.start(1.secs()).unwrap();
        timer.listen(Event::Update);

        // USART1
        //let tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh); //RX
        //let rx = gpioa.pa10;                                         //TX

        //let mut gps_serial = Serial::new(
        //    cx.device.USART1,
        //    (tx, rx),
        //    &mut afio.mapr,
        //    Config::default().baudrate(9600.bps()),
        //    &clocks,
        //);

        (
            Shared {},
            Local {
                led_r: ledr,
                led_g: ledg,
                timer_handler: timer,
                //gps_serial,
                spi: rs_spi,
            },
            init::Monotonics(),
        )
    }

    #[idle(local=[spi])]
    fn idle(cx: idle::Context) -> ! {
        loop {
            let write_data  = [0x42];
            //cx.local.spi.write(&write_data);
            //rprintln!("kadse");
            // DO NOT UNCOMMENT UNLESS YOU WANT TO LIFT THE BOOT0 PIN
            //cortex_m::asm::wfi();
        }
    }


}