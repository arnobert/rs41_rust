#![no_std]
#![no_main]

#[cfg(feature = "hell")]
mod hell;

use embedded_hal::spi::{Mode, Phase, Polarity};
use panic_halt as _;

// USER CONFIG -------------------------------------------------------------------------------------

// CALLSIGN
const CALLSIGN: [char; 7] = ['D', 'Q', '5', '0', 'R', 'U', 'B'];

// TX PERIOD [s]
const TX_PERIOD: u8 = 30;

// TX power
const TX_POWER: si4032_driver::ETxPower = si4032_driver::ETxPower::P5dBm;

// Frequency, as calculated by Python script
const HBSEL: bool = true;
const FREQBAND: u8 = 0x00;
const CAR_FREQ: u16 = 0xE9A7;

// -------------------------------------------------------------------------------------------------
const F_C_UPPER: u8 = ((CAR_FREQ & 0xFF00) >> 8) as u8;
const F_C_LOWER: u8 = (CAR_FREQ & 0x00FF) as u8;

// Hell mode parameters
const HELL_DATA_RATE: u16 = 0x252;
const HELL_DELAY: u32 = 150000;

// GFSK mode parameters.
// 1200 Baud => 0xB6D
const GFSK_DATA_RATE: u16 = 0xB6D;
// -------------------------------------------------------------------------------------------------

const RX_BUF_SIZE: usize = 128;

pub const SPIMODE: Mode = Mode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

// -------------------------------------------------------------------------------------------------

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [TIM2, TIM3, TIM4])]
mod app {
    use systick_monotonic::{fugit::Duration, Systick};
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
    use crate::{F_C_UPPER, F_C_LOWER, SPIMODE, TX_POWER, FREQBAND, HBSEL, CALLSIGN, RX_BUF_SIZE, HELL_DATA_RATE, GFSK_DATA_RATE, HELL_DELAY};

    #[cfg(feature = "hell")]
    use crate::hell;

    use ublox::*;
    use heapless::Vec;
    use si4032_driver::ETxPower;
    use stm32f1xx_hal::gpio::Analog;
    use stm32f1xx_hal::pac::{ADC1, USART1, USART3};
    use stm32f1xx_hal::time::ms;

    use lexical_core::BUFFER_SIZE;
    //----------------------------------------------------------------------------------------------
    #[shared]
    struct Shared {
        #[lock_free]
        led_g: PB7<Output<PushPull>>,
        position: [f64; 3],
        #[lock_free]
        gps_tx: stm32f1xx_hal::serial::Tx<USART1>,
        #[lock_free]
        gps_rx: stm32f1xx_hal::serial::Rx<USART1>,
        rx_buf: Vec<u8, RX_BUF_SIZE>,
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
        payload_len: u16,
        msg_cnt: u16,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        /*
        fn init_profile(device: &pac::Peripherals) {
            // On development, keep the DBG module powered on during wfi()
            device.DBGMCU.cr.modify(|_, w| w.dbg_standby().set_bit());
            device.DBGMCU.cr.modify(|_, w| w.dbg_sleep().set_bit());
            device.DBGMCU.cr.modify(|_, w| w.dbg_stop().set_bit());
        }
        init_profile(&cx.device);
         */

        // Pend interrupts during init -------------------------------------------------------------
        rtic::pend(stm32f1xx_hal::pac::interrupt::TIM2);

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

        /*
        let channels = cx.device.DMA1.split();
        */

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
        gps_rx.unlisten();

        let mut rxbuf: Vec<u8, RX_BUF_SIZE> = Vec::new();


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
        //dbg_rx.listen();

        // ADC -------------------------------------------------------------------------------------
        let adc_ch0 = gpioa.pa5.into_analog(&mut gpioa.crl); // Battery voltage
        let adc_ch1 = gpioa.pa6.into_analog(&mut gpioa.crl); // Switch

        let adc1 = stm32f1xx_hal::adc::Adc::adc1(cx.device.ADC1, clocks);


        let _adc1_ch1 = gpioa.pa1.into_analog(&mut gpioa.crl); // Measurement out

        // SHUTDOWN pin ----------------------------------------------------------------------------
        let shtdwn = gpioa.pa12.into_push_pull_output_with_state(&mut gpioa.crh, PinState::Low);

        // State machine for UART receiver ---------------------------------------------------------
        let mut rxd1: u8 = 0;
        let mut stdet: bool = false;
        let mut payloadlen: u16 = 0xFFF0;
        let mut msg_cnt: u16 = 0;

        // End init --------------------------------------------------------------------------------
        (
            Shared {
                position: [0.0, 0.0, 0.0],
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
                payload_len: payloadlen,
                msg_cnt: msg_cnt,
            },
            init::Monotonics(mono),
        )
    }


    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        blink_led::spawn_after(Duration::<u64, 1, 1000>::from_ticks(200)).unwrap();
        read_adc::spawn_after(Duration::<u64, 1, 1000>::from_ticks(400)).unwrap();

        //toggle_led_g::spawn_after(Duration::<u64, 1, 1000>::from_ticks(10)).unwrap();
        config_gps::spawn_after(Duration::<u64, 1, 1000>::from_ticks(5000)).unwrap();
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

        let mut position_len  = [b'0'; BUFFER_SIZE];
        let mut position_long  = [b'0'; BUFFER_SIZE];
        let mut position_height  = [b'0'; BUFFER_SIZE];

        position.lock(|position| {
            let c_cnt_len = lexical_core::write(position[0], &mut position_len);
            let c_cnt_long = lexical_core::write(position[1], &mut position_long);
            let c_position_height = lexical_core::write(position[2], &mut position_height);
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


            // Config for GFSK mode ----------------------------------------------------------------
            #[cfg(not(any(feature = "hell")))]
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


        // GFSK
        #[cfg(not(any(feature = "hell")))]
        {
            let sym_0 = b"DEADBEEF";

            radio.write_fifo(sym_0);


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

        _ = cx.shared.gps_tx.bwrite_all(&packet);
        _ = cx.shared.gps_tx.flush();

        cx.shared.gps_rx.listen();
    }


    #[task(shared = [gps_tx])]
    fn query_pos(mut cx: query_pos::Context) {
        let packet = UbxPacketRequest::request_for::<NavPosLlh>().into_packet_bytes();
        _ = cx.shared.gps_tx.bwrite_all(&packet);
        _ = cx.shared.gps_tx.flush();
        query_pos::spawn_after(Duration::<u64, 1, 1000>::from_ticks(3000)).unwrap();
    }


    // Receiving data from ublox isr ---------------------------------------------------------------
    #[task(binds = USART1, shared = [rx_buf, gps_rx], local = [rxd_t1, st_det, payload_len, msg_cnt])]
    fn isr_gps(mut cx: isr_gps::Context) {
        let rx = cx.shared.gps_rx;

        let mut rx_buf = cx.shared.rx_buf;
        let mut rxd1 = cx.local.rxd_t1;
        let mut start_detect = cx.local.st_det;
        let mut payload_len = cx.local.payload_len;
        let mut msg_cnt = cx.local.msg_cnt;


        // Wait until next char is arrived
        while !rx.is_rx_not_empty() {
            cortex_m::asm::delay(5);
        }

        let rxb = rx.read();

        match rxb {
            Ok(T) => {
                let rxd = T;

                // Detecting magic word 0xB5 0x62
                if (*start_detect == false) && (*rxd1 == 0xB5) && (rxd == 0x62) {
                    *start_detect = true;
                    rx_buf.lock(|rx_buf| {
                        rx_buf.clear();
                        _ = rx_buf.push(0xB5);
                        *msg_cnt = 1;
                        return;
                    });
                } else {
                    // *Shift register*
                    if *start_detect == false {
                        *rxd1 = rxd;
                    }
                }

                // Writing into rx buffer
                if *start_detect == true {
                    rx_buf.lock(|rx_buf| {
                        *msg_cnt = *msg_cnt + 1;
                        rx_buf.push(rxd).unwrap();

                        // Reading payload length
                        if *msg_cnt == 7 {
                            *payload_len = ((rx_buf[5] as u16) << 8) + rx_buf[4] as u16;
                        }

                        // Message complete
                        if *msg_cnt >= (*payload_len + 8) {
                            *start_detect = false;
                            *msg_cnt = 0;
                            *payload_len = 0xFFF0;
                            parse_gps_data::spawn().unwrap();
                        }
                    });
                }
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

    #[task(shared = [position, rx_buf])]
    fn parse_gps_data(cx: parse_gps_data::Context) {
        let mut rx_buf = cx.shared.rx_buf;
        let mut position = cx.shared.position;


        // Local buffer
        let mut pbuf: Vec<u8, RX_BUF_SIZE> = Vec::new();

        rx_buf.lock(|rx_buf| {
            let ubxbuf = ublox::FixedLinearBuffer::new(&mut pbuf);
            let mut parser = ublox::Parser::new(ubxbuf);

            let mut rxdt = parser.consume(&rx_buf);

            loop {
                match rxdt.next() {
                    Some(Ok(packet)) => {
                        match packet {
                            PacketRef::NavPosLlh(pack) => {
                                position.lock(|position| {
                                    position[0] = pack.lat_degrees();
                                    position[1] = pack.lon_degrees();
                                    position[2] = pack.height_msl();
                                });
                            }

                            _ => {}
                        };
                    }
                    Some(Err(_)) => {
                        // Received a malformed packet, ignore it
                    }
                    None => {
                        // We've eaten all the packets we have
                        break;
                    }
                }
            }
        });
        toggle_led_g::spawn_after(Duration::<u64, 1, 1000>::from_ticks(10)).unwrap();
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
    fn print_dbg(cx: print_dbg::Context) {
        let mut rx_buf = cx.shared.rx_buf;
        let dbg_tx = cx.local.dbg_tx;

        rx_buf.lock(|rx_buf| {
            for msx in rx_buf {
                while !dbg_tx.is_tx_empty() {};
                _ = dbg_tx.write(*msx);
            }
        });
    }

    #[task(binds = USART3, local = [dbg_rx])]
    fn read_dbg(cx: read_dbg::Context) {
        let rx = cx.local.dbg_rx;
        let rxb = rx.read();
        match rxb {
            Ok(_T) => {
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