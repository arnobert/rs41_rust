#![no_std]
#![no_main]

// USER CONFIG -------------------------------------------------------------------------------------

// CALLSIGN
const CALLSIGN: [char; 7] = ['D', 'Q', '5', '0', 'R', 'U', 'B'];

// TX PERIOD [s]
const TX_PERIOD: u8 = 30;


const TX_POWER: si4032_driver::ETxPower = si4032_driver::ETxPower::P5dBm;
// -------------------------------------------------------------------------------------------------


// Frequency, as calculated by Python script
const HBSEL: bool = true;
const FREQBAND: u8 = 0x00;
const CAR_FREQ: u16 = 0xE9A7;


const F_C_UPPER: u8 = ((CAR_FREQ & 0xFF00) >> 8) as u8;
const F_C_LOWER: u8 = (CAR_FREQ & 0x00FF) as u8;

const HELL_DATA_RATE: u16 = 0x252;
const HELL_DELAY: u32 = 150000;
// 1200 Baud = 0xB6D
const GFSK_DATA_RATE: u16 = 0xB6D;
// -------------------------------------------------------------------------------------------------

const rx_buf_size: usize = 128;


#[cfg(feature = "hell")]
mod hell;

use embedded_hal::spi::{Mode, Phase, Polarity};

pub const SPIMODE: Mode = Mode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

#[allow(unused_imports)]
use cortex_m_rt::entry;
use panic_halt as _;

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [TIM2, TIM3, TIM4])]
mod app {
    use cortex_m::singleton;
    use embedded_hal::adc::Channel;
    //use rtt_target::{rprintln, rtt_init_print};
    use systick_monotonic::{fugit::Duration, Systick};
    use nb::block;
    use stm32f1xx_hal::{
        adc::*,
        gpio::{gpioa::*, gpiob::*, gpioc::*,
               Input, Alternate, Floating,
               Output, PinState, PushPull},
        pac,
        prelude::*,
        timer::{CounterMs, Event},
        serial::{Config, Serial},
        spi::*,
    };
    use crate::{F_C_UPPER, F_C_LOWER, SPIMODE, TX_POWER, FREQBAND, HBSEL, CALLSIGN, rx_buf_size, HELL_DATA_RATE, GFSK_DATA_RATE, HELL_DELAY};

    #[cfg(feature = "hell")]
    use crate::hell;

    use ublox::*;
    use heapless::Vec;
    use si4032_driver::ETxPower;
    use stm32f1xx_hal::gpio::Analog;
    use stm32f1xx_hal::pac::{ADC1, USART1, USART3};
    use stm32f1xx_hal::serial::ReleaseToken;

    //----------------------------------------------------------------------------------------------
    #[shared]
    struct Shared {
        #[lock_free]
        led_g: PB7<Output<PushPull>>,
        position: [u32; 3],
        #[lock_free]
        gps_tx: stm32f1xx_hal::serial::Tx<USART1>,
        #[lock_free]
        gps_rx: stm32f1xx_hal::serial::Rx<USART1>,
        rx_buf: Vec<u8, rx_buf_size>,
    }

    #[local]
    struct Local {
        led_r: PB8<Output<PushPull>>,
        timer_handler: CounterMs<pac::TIM1>,

        radio_spi: si4032_driver::Si4032<Spi<stm32f1xx_hal::pac::SPI2,
            stm32f1xx_hal::spi::Spi2NoRemap,
            (PB13<Alternate<PushPull>>, PB14, PB15<Alternate<PushPull>>),
            u8>,
            PC13<Output<PushPull>>>,
        radio_init: bool,
        freq_upper: u8,
        freq_lower: u8,
        txpwr: ETxPower,
        adc_ch_0: PA5<Analog>,
        adc_ch_1: PA6<Analog>,
        adc_1: stm32f1xx_hal::adc::Adc<ADC1>,
        shutdown: PA12<Output<PushPull>>,
        shutdown_next_cycle: bool,
        dbg_tx: stm32f1xx_hal::serial::Tx<USART3>,
        dbg_rx: stm32f1xx_hal::serial::Rx<USART3>,
        rxd_t1: u8,
        st_det: bool,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        fn init_profile(device: &pac::Peripherals) {
            // On development, keep the DBG module powered on during wfi()
            device.DBGMCU.cr.modify(|_, w| w.dbg_standby().set_bit());
            device.DBGMCU.cr.modify(|_, w| w.dbg_sleep().set_bit());
            device.DBGMCU.cr.modify(|_, w| w.dbg_stop().set_bit());
        }
        //init_profile(&cx.device);

        // Pend interrupts during init -------------------------------------------------------------
        rtic::pend(stm32f1xx_hal::pac::interrupt::TIM2);
        rtic::pend(stm32f1xx_hal::pac::interrupt::USART1);

        // Peripherals -----------------------------------------------------------------------------
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        let mono = Systick::new(cx.core.SYST, 24_000_000);
        let clocks = rcc
            .cfgr
            .use_hse(24.MHz())
            .sysclk(24.MHz())
            .pclk1(24.MHz())
            .freeze(&mut flash.acr);


        let mut afio = cx.device.AFIO.constrain();
        let mut gpioa = cx.device.GPIOA.split();
        let mut gpiob = cx.device.GPIOB.split();
        let mut gpioc = cx.device.GPIOC.split();
        let channels = cx.device.DMA1.split();

        // Disable JTAG ----------------------------------------------------------------------------
        let (mut pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        // GPS disabled for the moment (PinState::High -> enables)
        let mut gps_rstn = pa15.into_open_drain_output_with_state(&mut gpioa.crh, PinState::High);

        // GPIO ------------------------------------------------------------------------------------
        let mut spst_1 = gpiob.pb6.into_floating_input(&mut gpiob.crl);
        let mut spst_2 = spst_1.into_open_drain_output(&mut gpiob.crl);

        spst_2.set_low();

        // LEDs ------------------------------------------------------------------------------------
        let mut ledr = gpiob
            .pb8
            .into_push_pull_output_with_state(&mut gpiob.crh, PinState::High);

        let mut ledg = gpiob
            .pb7
            .into_push_pull_output_with_state(&mut gpiob.crl, PinState::Low);

        ledr.toggle();

        // SPI -------------------------------------------------------------------------------------
        let spi_cs_radio = gpioc.pc13.into_push_pull_output_with_state(&mut gpioc.crh, PinState::High);

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

        let mut radioSPI = si4032_driver::Si4032::new(rs_spi, spi_cs_radio);

        // Timer -----------------------------------------------------------------------------------
        let mut timer = cx.device.TIM1.counter_ms(&clocks);
        timer.start(1.secs()).unwrap();
        timer.listen(Event::Update);

        // USART1 ----------------------------------------------------------------------------------
        let tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let rx = gpioa.pa10;

        let mut gps_serial = Serial::new(
            cx.device.USART1,
            (tx, rx),
            &mut afio.mapr,
            Config::default().baudrate(9600.bps()),
            &clocks,
        );
        let mut gps_tx = gps_serial.tx;
        gps_tx.unlisten();
        let mut gps_rx = gps_serial.rx;
        //gps_rx.unlisten_idle();

        let mut rxbuf: Vec<u8, rx_buf_size> = Vec::new();


        // USART3 (Expansion header)----------------------------------------------------------------
        let exp_tx = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
        let exp_rx = gpiob.pb11;

        let mut dbg_serial = Serial::new(
            cx.device.USART3,
            (exp_tx, exp_rx),
            &mut afio.mapr,
            Config::default().baudrate(9600.bps()),
            &clocks,
        );
        let mut dbg_tx = dbg_serial.tx;

        let mut dbg_rx = dbg_serial.rx;
        dbg_rx.listen();

        // ADC -------------------------------------------------------------------------------------
        let adc_ch0 = gpioa.pa5.into_analog(&mut gpioa.crl); // Battery voltage
        let adc_ch1 = gpioa.pa6.into_analog(&mut gpioa.crl); // Switch

        let adc1 = stm32f1xx_hal::adc::Adc::adc1(cx.device.ADC1, clocks);


        let adc1_ch1 = gpioa.pa1.into_analog(&mut gpioa.crl); // Measurement out

        // SHUTDOWN pin ----------------------------------------------------------------------------
        let shtdwn = gpioa.pa12.into_push_pull_output_with_state(&mut gpioa.crh, PinState::Low);

        // State machine for UART receiver ---------------------------------------------------------
        let mut rxd1: u8 = 0;
        let mut stdet: bool = false;

        // End init --------------------------------------------------------------------------------
        (
            Shared {
                position: [0, 0, 0],
                led_g: ledg,
                gps_tx,
                gps_rx,
                rx_buf: rxbuf,
            },
            Local {
                led_r: ledr,
                timer_handler: timer,
                radio_spi: radioSPI,
                radio_init: false,
                freq_upper: F_C_UPPER,
                freq_lower: F_C_LOWER,
                txpwr: TX_POWER,
                adc_ch_0: adc_ch0,
                adc_ch_1: adc_ch1,
                adc_1: adc1,
                shutdown: shtdwn,
                shutdown_next_cycle: false,
                dbg_tx: dbg_tx,
                dbg_rx: dbg_rx,
                rxd_t1: rxd1,
                st_det: stdet,
            },
            init::Monotonics(mono),
        )
    }


    #[idle()]
    fn idle(cx: idle::Context) -> ! {
        blink_led::spawn_after(Duration::<u64, 1, 1000>::from_ticks(200)).unwrap();
        read_adc::spawn_after(Duration::<u64, 1, 1000>::from_ticks(400)).unwrap();

        toggle_led_g::spawn_after(Duration::<u64, 1, 1000>::from_ticks(10)).unwrap();
        config_gps::spawn_after(Duration::<u64, 1, 1000>::from_ticks(2000)).unwrap();
        query_pos::spawn_after(Duration::<u64, 1, 1000>::from_ticks(10000)).unwrap();

        tx::spawn_after(Duration::<u64, 1, 1000>::from_ticks(2500)).unwrap();


        loop {
            // DO NOT UNCOMMENT UNLESS YOU WANT TO LIFT THE BOOT0 PIN
            //cortex_m::asm::wfi();
            cortex_m::asm::delay(100);
        }
    }

    #[task(local = [led_r])]
    fn blink_led(cx: blink_led::Context) {
        cx.local.led_r.toggle();
        blink_led::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();
    }


    #[task(shared = [led_g])]
    fn toggle_led_g(cx: toggle_led_g::Context) {
        cx.shared.led_g.toggle();
    }

    // ---------------------------------------------------------------------------------------------
    // This is the main task. We receive our GPS location, calculate coordinates,
    // concat the characters and write to radio FIFO.
    // ---------------------------------------------------------------------------------------------
    #[task(priority = 3, local = [radio_spi, radio_init, freq_upper, freq_lower, txpwr], shared = [position])]
    fn tx(cx: tx::Context) {
        let radio = cx.local.radio_spi;

        // Getting position data
        let mut position = cx.shared.position;
        let mut position_X: u8 = 0;

        position.lock(|position| {
            position_X = position[0] as u8;
        });


        // Init Radio ------------------------------------------------------------------------------
        if *cx.local.radio_init == false {
            radio.swreset();
            while !(radio.chip_ready()) {};

            // Set frequencies
            radio.set_hb_sel(HBSEL);
            radio.set_freq_band(FREQBAND);
            radio.set_freq(*cx.local.freq_upper, *cx.local.freq_lower);


            radio.set_tx_pwr(si4032_driver::ETxPower::P1dBm);

            let fband = radio.get_freq_band();

            // Config for HELL mode ----------------------------------------------------------------
            #[cfg(feature = "hell")]
            {
                radio.set_modulation_type(si4032_driver::ModType::OOK);
                radio.set_auto_packet_handler(false);
                radio.set_modulation_source(si4032_driver::ModDataSrc::Fifo);

                radio.set_trxdrtscale(true);
                radio.set_data_rate(HELL_DATA_RATE);
                // CRC
                radio.set_crc(false);
            }

            // Config for RTTY mode ----------------------------------------------------------------
            #[cfg(feature = "rtty")]
            {
                radio.set_modulation_type(si4032_driver::ModType::FSK);
            }

            // Config for GFSK mode ----------------------------------------------------------------
            #[cfg(not(any(feature = "rtty", feature = "hell")))]
            {
                radio.set_modulation_type(si4032_driver::ModType::GFSK);
                radio.set_freq_deviation(0x05);
                //radio.set_freq_offset(0x002);
                radio.set_trxdrtscale(true);
                radio.set_data_rate(GFSK_DATA_RATE);

                radio.set_auto_packet_handler(true);
                radio.set_modulation_source(si4032_driver::ModDataSrc::Fifo);

                // Preamble
                radio.set_tx_prealen(0x20);

                // Sync Word
                // F8D8 = 11100110 11011000
                radio.set_sync_wrd(0xE6D8 << 16);

                // 00 -> Sync Word 3
                // 01 -> Sync Word 3, 2
                radio.set_tx_sync_len(0x01);


                // TX Header
                radio.set_tx_header_len(0);


                // Packet Length
                radio.set_packet_len(24);
                radio.set_tx_fixplen(false);

                // CRC
                //radio.set_crc(false);
            }

            radio.enter_tx();
            *cx.local.radio_init = true;
        }

        // TEXT TO BE SENT:
        // $CALL$ POS:00.00000N, 00.00000E, 13370M

        // OOK / HELL
        #[cfg(feature = "hell")]
        {
            for txchar in CALLSIGN {
                let h_symbol: u128 = hell::get_char(txchar);
                let h_bytes: [u8; 16] = h_symbol.to_be_bytes();

                for txcnt in 0..16
                {
                    let sym = [h_bytes[txcnt as usize]];
                    radio.write_fifo(&sym);
                    radio.tx_on();

                    while !(radio.fifo_empty())
                    {
                        cortex_m::asm::nop();
                    }
                }

                for _ in 0..HELL_DELAY {
                    cortex_m::asm::nop();
                }
            }
        }

        // RTTY
        #[cfg(feature = "rtty")]
        {}


        // GFSK
        #[cfg(not(any(feature = "rtty", feature = "hell")))]
        {
            let sym_0 = [b'D', b'E', b'A', b'D',
                b'B', b'E', b'E', b'F',
                b'D', b'E', b'A', b'D',
                b'B', b'E', b'E', b'F',
                b'D', b'E', b'A', b'D',
                b'B', b'E', b'E', b'F'];

            radio.write_fifo(&sym_0);


            if radio.is_tx_on() == false {
                radio.tx_on();
            }
        }
        tx::spawn_after(Duration::<u64, 1, 1000>::from_ticks(3000)).unwrap();
    }


    // GPS -----------------------------------------------------------------------------------------
    // Config ublox
    #[task(shared = [gps_tx, gps_rx])]
    fn config_gps(mut cx: config_gps::Context) {
        let packet: [u8; 28] = CfgPrtUartBuilder {
            portid: UartPortId::Uart1,
            reserved0: 0,
            tx_ready: 0,
            mode: 0x8d0,
            baud_rate: 9600,
            in_proto_mask: 0x01,
            out_proto_mask: 0x01,
            flags: 0,
            reserved5: 0,
        }.into_packet_bytes();

        cx.shared.gps_tx.bwrite_all(&packet);
        cx.shared.gps_tx.flush();

        cx.shared.gps_rx.listen();
    }


    #[task(shared = [gps_tx])]
    fn query_pos(mut cx: query_pos::Context) {
        let packet = UbxPacketRequest::request_for::<NavPosLlh>().into_packet_bytes();
        cx.shared.gps_tx.bwrite_all(&packet);
        cx.shared.gps_tx.flush();
        query_pos::spawn_after(Duration::<u64, 1, 1000>::from_ticks(3000)).unwrap();
    }


    // Receiving data from ublox. ------------------------------------------------------------------
    #[task(binds = USART1, shared = [gps_rx])]
    fn receive_gps_dx(mut cx: receive_gps_dx::Context) {
        let rx = cx.shared.gps_rx;

        let rxb = rx.read();
        match rxb {
            Ok(T) => {
                process_gps_data::spawn(T).unwrap();
            }
            Err(E) => {
                match E {
                    nb::Error::Other(stm32f1xx_hal::serial::Error::Overrun) => {}
                    nb::Error::Other(stm32f1xx_hal::serial::Error::Framing) => {}
                    nb::Error::Other(stm32f1xx_hal::serial::Error::Noise) => {}
                    nb::Error::Other(stm32f1xx_hal::serial::Error::Parity) => {}
                    _ => {}
                }
            }
        }
    }

    #[task(shared = [rx_buf], local = [rxd_t1, st_det])]
    fn process_gps_data(cx: process_gps_data::Context, rxd: u8) {
        let mut rx_buf = cx.shared.rx_buf;
        let mut rxd1 = cx.local.rxd_t1;
        let mut start_detect = cx.local.st_det;
        let mut msg_len: u8 = 0;

        // Writing into rx buffer
        if (*start_detect == true) {
            rx_buf.lock(|rx_buf| {
                rx_buf.push(rxd);

                let bufp = rx_buf.len() as u8;
                if bufp > 3 {
                    msg_len = rx_buf[2];

                    // Message complete
                    if (bufp > (msg_len + 2)) {
                        print_dbg::spawn(0x10);
                        *start_detect = false;
                    }
                };
            });
        }

        // Detecting magic word 0xB5 0x62
        if ((!*start_detect) && (*rxd1 == 0xB5) && (rxd == 0x62)) {
            *start_detect = true;

            rx_buf.lock(|rx_buf| {
                rx_buf.clear()
            });
        }

        // *Shift register*
        *rxd1 = rxd;
    }

    #[task(shared = [position, rx_buf] )]
    fn parse_gps_pos(cx: parse_gps_pos::Context) {
        let mut buf: Vec<u8, 256> = Vec::new();
        let buf = ublox::FixedLinearBuffer::new(&mut buf[..]);
        let mut parser = ublox::Parser::new(buf);
    }

    // ADC measurements ----------------------------------------------------------------------------
    #[task(shared = [led_g], local = [adc_ch_0, adc_ch_1, adc_1, shutdown, shutdown_next_cycle])]
    fn read_adc(cx: read_adc::Context) {
        let vbat: u16 = cx.local.adc_1.read(cx.local.adc_ch_0).unwrap();
        let pbut: u16 = cx.local.adc_1.read(cx.local.adc_ch_1).unwrap();

        if *cx.local.shutdown_next_cycle {
            cx.local.shutdown.set_high();
        }

        if (vbat - pbut) < 100 {
            *cx.local.shutdown_next_cycle = true;
            cx.shared.led_g.set_high();
        }

        read_adc::spawn_after(Duration::<u64, 1, 1000>::from_ticks(2000)).unwrap();
    }


    // Debug UART ----------------------------------------------------------------------------------
    #[task(local = [dbg_tx], shared = [rx_buf])]
    fn print_dbg(cx: print_dbg::Context, msg: u8) {
        let mut rx_buf = cx.shared.rx_buf;
        let mut dbg_tx = cx.local.dbg_tx;

        rx_buf.lock(|rx_buf| {
            if msg == 0xFF {
                dbg_tx.write(rx_buf.len() as u8);
            }

            // Print RX buffer
            if msg == 0x10 {
                for c in rx_buf.iter() {
                    dbg_tx.write(*c);
                };
            }

            //else {
            //    dbg_tx.write(msg);
            //}
        });
    }

    #[task(binds = USART3, local = [dbg_rx])]
    fn read_dbg(cx: read_dbg::Context) {

        let rx = cx.local.dbg_rx;
        let rxb = rx.read();
        match rxb {
            Ok(T) => {
                toggle_led_g::spawn_after(Duration::<u64, 1, 1000>::from_ticks(10)).unwrap();
            }
            Err(E) => {
                match E {
                    nb::Error::Other(stm32f1xx_hal::serial::Error::Overrun) => {}
                    nb::Error::Other(stm32f1xx_hal::serial::Error::Framing) => {}
                    nb::Error::Other(stm32f1xx_hal::serial::Error::Noise) => {}
                    nb::Error::Other(stm32f1xx_hal::serial::Error::Parity) => {}
                    _ => {}
                }
            }
        }
    }
}