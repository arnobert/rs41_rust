#![no_std]
#![no_main]

#[cfg(feature = "hell")]
mod hell;

use embedded_hal::spi::{Mode, Phase, Polarity};
use panic_halt as _;

use rtic_monotonics::systick::Systick;
use rtt_target::{rtt_init_print, rprintln};

use stm32f1xx_hal::{
    adc::*,
    gpio::{gpioa::*, gpiob::*, gpioc::*,
           Input, Alternate, Floating,
           Output, PinState, PushPull},
    prelude::*,
    serial::{Config, Serial},
    spi::*,
    timer::*,
};

use ublox::*;
use heapless::Vec;
use si4032_driver::ETxPower;
use stm32f1xx_hal::gpio::Analog;
use stm32f1xx_hal::pac::{ADC1, USART1, USART3};

use lexical_core::BUFFER_SIZE;

// USER CONFIG -------------------------------------------------------------------------------------

// CALLSIGN
const CALLSIGN: &[u8; 11] = b"DQ50RUB WX ";


// Carrier freq in MHz
const CAR_FREQ: f32 = 432.2;


// TX PERIOD [s]
const TX_PERIOD: u32 = 1;

// TX power
const TX_POWER: si4032_driver::ETxPower = si4032_driver::ETxPower::P5dBm;

const HBSEL: bool = true;
const HBSEL_F32: f32 = if HBSEL {1.0} else {0.0};


const FREQBAND: u8 = if CAR_FREQ < 433.0 {0} else {1};

const FC: f32 = (CAR_FREQ / ((26.0/3.0) * (HBSEL_F32 + 1.0)) - FREQBAND as f32 - 24.0)  * 64000.0;
const FCU: u32 = FC as u32;

const F_C_UPPER: u8 = ((FCU & 0xFF00) >> 8) as u8;
const F_C_LOWER: u8 = (FCU & 0x00FF) as u8;

// -------------------------------------------------------------------------------------------------
//const F_C_UPPER: u8 = ((CAR_FREQ_REG & 0xFF00) >> 8) as u8;
//const F_C_LOWER: u8 = (CAR_FREQ_REG & 0x00FF) as u8;

// Hell mode parameters
// Set data rate to:
// 0x254 for Feld Hell
// TBD: Slow Hell, Hell x5 ...

#[cfg(feature = "hell")]
const HELL_DATA_RATE: u16 = 0x252;

#[cfg(feature = "hell")]
const HELL_DELAY: u32 = 150000;

#[cfg(feature = "hell")]
const COORD_HEIGHT: &[u8; 7] = b"HEIGHT ";

#[cfg(feature = "hell")]
const COORD_LEN: &[u8; 4] = b"LEN ";

#[cfg(feature = "hell")]
const COORD_LONG: &[u8; 5] = b"LONG ";

// GFSK mode parameters.
// 1200 Baud => 0xB6D
//const GFSK_DATA_RATE: u16 = 0x1D4D;
const GFSK_DATA_RATE: u16 = 0x0EA6;
// -------------------------------------------------------------------------------------------------

const RX_BUF_SIZE: usize = 128;

pub const SPIMODE: Mode = Mode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

// -------------------------------------------------------------------------------------------------

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [TIM3, TIM4, TIM5, TIM13])]
mod app {
    use core::ops::Deref;
    use rtic::Mutex;
    use stm32f1xx_hal::pac::{DBGMCU, TIM2};
    use super::*;

    #[shared]
    struct Shared {
        led_g: PB7<Output<PushPull>>,
        position: [f64; 3],
        position_raw: [i32; 3],
        utc_hour: u8,
        utc_min: u8,
        utc_sec: u8,
        vbat: u16,
        gps_tx: stm32f1xx_hal::serial::Tx<USART1>,
        gps_rx_idle: bool,
        rx_buf: Vec<u8, RX_BUF_SIZE>,
    }

    #[local]
    struct Local {
        led_r: PB8<Output<PushPull>>,
        gps_rx: stm32f1xx_hal::serial::Rx<USART1>,
        timer_handler: TIM2,
        timer_ticks: u32,
        radio_spi: si4032_driver::Si4032<Spi<stm32f1xx_hal::pac::SPI2,
            Spi2NoRemap,
            (PB13<Alternate<PushPull>>, PB14, PB15<Alternate<PushPull>>),
            u8>,
            PC13<Output<PushPull>>>,
        freq_upper: u8,
        freq_lower: u8,
        txpwr: ETxPower,
        adc_ch_0: PA5<Analog>,
        adc_ch_1: PA6<Analog>,
        adc_1: Adc<ADC1>,
        shutdown: PA12<Output<PushPull>>,
        shutdown_next_cycle: bool,
        dbg_tx: stm32f1xx_hal::serial::Tx<USART3>,
        dbg_rx: stm32f1xx_hal::serial::Rx<USART3>,
        rxd_t1: u8,
        st_det: bool,
        payload_len: u16,
        msg_cnt: u16,
        gpio_temp: stm32f1xx_hal::gpio::Pin<'A', 1>,
        packet_cnt: u8
    }


    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
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
        cx.device.RCC.apb1enr.modify(|r, w| w.tim2en().enabled());
        let rcc = cx.device.RCC.constrain();

        let mono_token = rtic_monotonics::create_systick_token!();
        let mono = Systick::start(cx.core.SYST, 24_000_000, mono_token);
        let clocks = rcc
            .cfgr
            .use_hse(24.MHz())
            .sysclk(24.MHz())
            .pclk1(24.MHz())
            .freeze(&mut flash.acr);

        let mut dbg = cx.device.DBGMCU;
        let mut afio = cx.device.AFIO.constrain();
        let mut gpioa = cx.device.GPIOA.split();
        let mut gpiob = cx.device.GPIOB.split();
        let mut gpioc = cx.device.GPIOC.split();

        // Disable JTAG ----------------------------------------------------------------------------
        let (mut pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        // GPS disabled for the moment (PinState::High -> enables)
        let mut gps_rstn = pa15.into_open_drain_output_with_state(&mut gpioa.crh, PinState::High);

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

        let mut radio = si4032_driver::Si4032::new(rs_spi, spi_cs_radio);

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

        // SHUTDOWN pin ----------------------------------------------------------------------------
        let shtdwn = gpioa.pa12.into_push_pull_output_with_state(&mut gpioa.crh, PinState::Low);

        // Metrology interface ---------------------------------------------------------------------

        let mut pullup_temp = gpiob.pb12.into_push_pull_output_with_state(&mut gpiob.crh, PinState::High); // Boom Temp
        let mut spst_1 = gpiob.pb6.into_push_pull_output_with_state(&mut gpiob.crl, PinState::Low); // Boom Temp
        let mut spst_2 = gpioa.pa3.into_push_pull_output_with_state(&mut gpioa.crl, PinState::Low); // Boom Hygro
        let mut spst_3 = gpioc.pc14.into_push_pull_output_with_state(&mut gpioc.crh, PinState::Low);
        let mut spst_4 = gpioc.pc15.into_push_pull_output_with_state(&mut gpioc.crh, PinState::High);

        let mut pullup_hyg = gpioa.pa2.into_push_pull_output_with_state(&mut gpioa.crl, PinState::Low); // Boom Temp
        let mut spdt_1 = pb3.into_push_pull_output(&mut gpiob.crl);
        let mut spdt_2 = pb4.into_push_pull_output(&mut gpiob.crl);
        let mut spdt_3 = gpiob.pb5.into_push_pull_output(&mut gpiob.crl);

        let mut meas_in = gpioa.pa1.into_floating_input(&mut gpioa.crl);

        // fRes temp boom: ~ 63 kHz @ room temp
        // TIMER -----------------------------------------------------------------------------------
        //let mut timer2: Counter<TIM2, 1_000_000> = cx.device.TIM2.counter(&clocks).into();
        let mut timer2 = cx.device.TIM2;

        // Start timer
        timer2.cr1.modify(|r, w| w.cen().enabled());
        // Trigger
        timer2.smcr.modify(|r, w| w.ts().ti2fp2());
        timer2.smcr.modify(|r, w| { w.sms().ext_clock_mode() });


        // State machine for UART receiver ---------------------------------------------------------
        let mut rxd1: u8 = 0;
        let mut stdet: bool = false;
        let mut payloadlen: u16 = 0xFFF0;
        let mut msg_cnt: u16 = 0;


        // GPS -------------------------------------------------------------------------------------
        // Config ublox

        let packet: [u8; 28] = CfgPrtUartBuilder {
            portid: UartPortId::Uart1,
            reserved0: 0,
            tx_ready: 0,
            mode: 0x8d0.into(),
            baud_rate: 9600,
            in_proto_mask: InProtoMask::UBLOX,
            out_proto_mask: OutProtoMask::UBLOX,
            flags: 0,
            reserved5: 0,
        }.into_packet_bytes();

        let _ = gps_tx.bwrite_all(&packet);
        let _ = gps_tx.flush();

        // Set Dynamic model: Airborne <2g
        let model_packet = CfgNav5Builder {
            mask: CfgNav5Params::DYN,
            dyn_model: CfgNav5DynModel::AirborneWith4gAcceleration,
            fix_mode: CfgNav5FixMode::default(),
            fixed_alt: 0.0,
            fixed_alt_var: 0.0,
            min_elev_degrees: 0,
            dr_limit: 0,
            pdop: 0.0,
            tdop: 0.0,
            pacc: 0,
            tacc: 0,
            static_hold_thresh: 0.0,
            dgps_time_out: 0,
            cno_thresh_num_svs: 0,
            cno_thresh: 0,
            reserved1: [0; 2],
            static_hold_max_dist: 0,
            utc_standard: CfgNav5UtcStandard::default(),
            reserved2: [0; 5],

        }.into_packet_bytes();

        let _ = gps_tx.bwrite_all(&model_packet);
        gps_rx.listen();


        // Init Radio ------------------------------------------------------------------------------
        radio.swreset();
        while !(radio.chip_ready()) {};

        // Set frequencies
        radio.set_hb_sel(HBSEL);
        radio.set_freq_band(FREQBAND);
        radio.set_freq(F_C_UPPER, F_C_LOWER);

        radio.init_gpio_1();
        radio.set_gpio_1(false);

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
            // Reference Heat
            radio.set_gpio_1(false);

            radio.set_man_en(true);
            radio.set_modulation_type(si4032_driver::ModType::FSK);
            radio.set_freq_deviation(0x05);
            radio.set_trxdrtscale(true);
            radio.set_data_rate(GFSK_DATA_RATE);

            radio.set_auto_packet_handler(true);
            radio.set_modulation_source(si4032_driver::ModDataSrc::Fifo);

            // Preamble
            radio.set_tx_prealen(0x0F);

            // Sync Word
            radio.set_sync_wrd(0x1798 << 16);

            // 00 -> Sync Word 3
            // 01 -> Sync Word 3, 2
            radio.set_tx_sync_len(0x01);


            // TX Header
            radio.set_tx_header_len(0);

            // CRC
            radio.set_crc_en(true);
            radio.set_crc_d_only(true);

            // Packet Length
            radio.set_packet_len(14);
            radio.set_tx_fixplen(false);
        }

        radio.enter_tx();

        // End init --------------------------------------------------------------------------------
        (
            Shared {
                position: [0.0; 3],
                position_raw: [0; 3],
                utc_hour: 0,
                utc_min: 0,
                utc_sec: 0,
                vbat: 0,
                led_g: ledg,
                gps_tx,
                //gps_rx,
                rx_buf: rxbuf,
                gps_rx_idle: true,
            },
            Local {
                gps_rx,
                led_r: ledr,
                timer_handler: timer2,
                timer_ticks: 0,
                radio_spi: radio,
                freq_upper: F_C_UPPER,
                freq_lower: F_C_LOWER,
                txpwr: TX_POWER,
                adc_ch_0: adc_ch0,
                adc_ch_1: adc_ch1,
                adc_1: adc1,
                shutdown: shtdwn,
                shutdown_next_cycle: false,
                dbg_tx,
                dbg_rx,
                rxd_t1: rxd1,
                st_det: stdet,
                payload_len: payloadlen,
                msg_cnt,
                gpio_temp: meas_in,
                packet_cnt: 0,
            },
        )
    }


    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        rtt_init_print!();
        blink_led::spawn().unwrap();
        cortex_m::asm::delay(10_000_000);
        //tim2_tick::spawn().unwrap();
        //read_adc::spawn().unwrap();
        cortex_m::asm::delay(10_000_000);
        tx::spawn().unwrap();
        loop {
            // DO NOT UNCOMMENT UNLESS YOU WANT TO LIFT THE BOOT0 PIN
            //cortex_m::asm::wfi();
            cortex_m::asm::delay(10_000);
        }
    }

    #[task(priority = 1, local = [led_r])]
    async fn blink_led(cx: blink_led::Context) {
        loop {
            cx.local.led_r.toggle();
            Systick::delay(1_000.millis()).await;
        }
    }


    // ---------------------------------------------------------------------------------------------
    // This is the main task. We receive our GPS location, calculate coordinates,
    // concat the characters and write to radio FIFO.
    // ---------------------------------------------------------------------------------------------
    #[task(priority = 2, local = [radio_spi, freq_upper, freq_lower, txpwr, packet_cnt], shared = [position, position_raw, gps_tx, gps_rx_idle, utc_hour, utc_min, utc_sec, vbat])]
    async fn tx(cx: tx::Context) {
        let radio = cx.local.radio_spi;
        let mut gps_rx_idle = cx.shared.gps_rx_idle;

        let mut utc_hour = cx.shared.utc_hour;
        let mut utc_min = cx.shared.utc_min;
        let mut utc_sec = cx.shared.utc_sec;

        // Getting position data
        let mut position = cx.shared.position;
        let mut position_raw = cx.shared.position_raw;

        let mut packet_cnt = cx.local.packet_cnt;

        let mut vbat = cx.shared.vbat;

        // GNSS-------------------------------------------------------------------------------------
        let mut tx = cx.shared.gps_tx;

        #[cfg(feature = "hell")]
        fn tx_hell(txdt: &[u8], tradio: &mut si4032_driver::Si4032<Spi<stm32f1xx_hal::pac::SPI2,
            stm32f1xx_hal::spi::Spi2NoRemap,
            (PB13<Alternate<PushPull>>, PB14, PB15<Alternate<PushPull>>), u8>,
            PC13<Output<PushPull>>>)
        {
            for txchar in txdt {
                let h_symbol: u128 = hell::get_char(*txchar);
                let h_bytes: [u8; 16] = h_symbol.to_be_bytes();

                for txcnt in 0..16
                {
                    let sym = [h_bytes[txcnt as usize]];
                    tradio.write_fifo(&sym);
                    tradio.tx_on();

                    while !(tradio.fifo_empty())
                    {
                        cortex_m::asm::nop();
                    }
                }

                for _ in 0..HELL_DELAY {
                    cortex_m::asm::nop();
                }
            }
        }


        let packet_pos = UbxPacketRequest::request_for::<NavPosLlh>().into_packet_bytes();
        let packet_utc = UbxPacketRequest::request_for::<NavTimeUTC>().into_packet_bytes();

        let mut position_len = [b'0'; BUFFER_SIZE];
        let mut position_long = [b'0'; BUFFER_SIZE];
        let mut position_height = [b'0'; BUFFER_SIZE];

        loop {

            tx.lock(|tx| {
                let _ = tx.bwrite_all(&packet_pos);
                cortex_m::asm::delay(10000000);
                //let _ = tx.bwrite_all(&packet_utc);
                let _ = tx.flush();
            });

            position.lock(|position| {
                let _ = lexical_core::write(position[0], &mut position_len);
                let _ = lexical_core::write(position[1], &mut position_long);
                let _ = lexical_core::write(position[2], &mut position_height);
            });

            let (f_len, _) = position_len.split_at_mut(8);
            let (f_long, _) = position_long.split_at_mut(8);
            let (f_height, _) = position_height.split_at_mut(5);

            // OOK / HELL
            #[cfg(feature = "hell")]
            {
                tx_hell(CALLSIGN, radio);

                tx_hell(COORD_HEIGHT, radio);
                tx_hell(f_height, radio);

                tx_hell(COORD_LEN, radio);
                tx_hell(f_len, radio);

                tx_hell(COORD_LONG, radio);
                tx_hell(&f_long, radio);

                utc_hour.lock(|utc_hour| {
                    hour = *utc_hour;
                });

                utc_min.lock(|utc_min| {
                    min = *utc_min;
                });

                utc_sec.lock(|utc_sec| {
                    sec = *utc_sec
                });


                let mut p_hour = [b'0'; BUFFER_SIZE];
                let mut p_min  = [b'0'; BUFFER_SIZE];
                let mut p_sec  = [b'0'; BUFFER_SIZE];

                let _ = lexical_core::write(hour, &mut p_hour);
                let _ = lexical_core::write(min, &mut p_min);
                let _ = lexical_core::write(sec, &mut p_sec);

                let (f_hour, _) = p_hour.split_at_mut(2);
                let (f_min, _) = p_min.split_at_mut(2);
                let (f_sec, _) = p_sec.split_at_mut(2);


                tx_hell(&f_hour, radio);
                tx_hell(&f_min, radio);
                tx_hell(&f_sec, radio);

            }


            // GFSK
            #[cfg(not(any(feature = "hell")))]
            {
                // --------------------------------
                // HORUS V2 16 Byte Format, modified
                // ---------------------------------------------------
                // BYTE NUM | SITE (BYTES) | DATA TYPE | Description
                // ---------------------------------------------------
                //    0     |       1      |  uint8    | Payload ID -> 0x42
                //    1     |       1      |  uint8    | Sequence No
                //    2     |       2      |  uint16   | Seconds of UTC
                //    4     |       3      |  Q9.15    | Latitude
                //    7     |       3      |  Q9.15    | Longitude
                //    10    |       2      |  uint16   | Height (m)
                //    12    |       1      |  uint8    | Battery Voltage
                //    13    |       1      |  uint8    | Flags Byte
                //    14    |       2      |  uint16   | CRC

                let mut sec: u8 = 0;
                utc_sec.lock(|utc_sec| {
                    sec = *utc_sec;
                });

                let mut lat: [u8; 3] = [0x0; 3];
                let mut long: [u8; 3] = [0x0; 3];
                let mut height: [u8; 2] = [0x0; 2];

                position_raw.lock(|position_raw| {
                    lat[0] = (position_raw[0] >> 24 & 0xFF) as u8;
                    lat[1] = (position_raw[0] >> 16 & 0xFF) as u8;
                    lat[2] = (position_raw[0] >> 8 & 0xFF) as u8;

                    long[0] = (position_raw[1] >> 24 & 0xFF) as u8;
                    long[1] = (position_raw[1] >> 16 & 0xFF) as u8;
                    long[2] = (position_raw[1] >> 8 & 0xFF) as u8;

                    height[0] = (position_raw[2] >> 16 & 0xFF) as u8;
                    height[1] = (position_raw[2] >> 8 & 0xFF) as u8;
                });

                let mut bat_volt: [u8; 1] = [0x00];
                vbat.lock(|vbat| {
                    let u: u8 = (*vbat >> 4) as u8;
                    rprintln!("VBAT: {}", u);
                    bat_volt = [u];
                });


                let flag_byte: [u8; 1] = [0xFF];

                // --- PACKET ASSEMBLY ------------
                radio.write_fifo(&[0x42]);
                radio.write_fifo(&[*packet_cnt]);
                radio.write_fifo(&[0x00, sec]);
                radio.write_fifo(&[lat[0]]);
                radio.write_fifo(&[lat[1]]);
                radio.write_fifo(&[lat[2]]);
                radio.write_fifo(&[long[0]]);
                radio.write_fifo(&[long[1]]);
                radio.write_fifo(&[long[2]]);
                radio.write_fifo(&[height[0]]);
                radio.write_fifo(&[height[1]]);
                radio.write_fifo(&bat_volt);
                radio.write_fifo(&flag_byte);

                if !radio.is_tx_on() {
                    radio.tx_on();
                }

                *packet_cnt = (*packet_cnt % 255) + 1;
            }
            Systick::delay(TX_PERIOD.secs()).await;
        }


    }


    // Receiving data from ublox isr ---------------------------------------------------------------
    #[task(priority = 4, binds = USART1, shared = [rx_buf, gps_rx_idle], local = [gps_rx, rxd_t1, st_det, payload_len, msg_cnt])]
    fn isr_gps(mut cx: isr_gps::Context) {
        //let rx = cx.shared.gps_rx;
        let rx = cx.local.gps_rx;

        let mut rx_buf = cx.shared.rx_buf;
        let mut rxd1 = cx.local.rxd_t1;
        let mut gps_rx_idle = cx.shared.gps_rx_idle;
        let mut start_detect = cx.local.st_det;
        let mut payload_len = cx.local.payload_len;
        let mut msg_cnt = cx.local.msg_cnt;


        // Wait until next char is arrived
        while !rx.is_rx_not_empty() {
            cortex_m::asm::delay(5);
        }

        let rxb = rx.read();

        match rxb {
            Ok(t) => {
                let rxd = t;

                // Detecting magic word 0xB5 0x62
                if (!*start_detect) && (*rxd1 == 0xB5) && (rxd == 0x62) {
                    *start_detect = true;

                    rx_buf.lock(|rx_buf| {
                        rx_buf.clear();
                        _ = rx_buf.push(0xB5);
                        *msg_cnt = 1;
                        return;
                    });
                } else {
                    // *Shift register*
                    if !*start_detect {
                        *rxd1 = rxd;
                    }
                }

                // Writing into rx buffer
                if *start_detect {
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
            Err (e) => {
                match e {
                    nb::Error::Other(stm32f1xx_hal::serial::Error::Overrun) => {}
                    nb::Error::Other(stm32f1xx_hal::serial::Error::Framing) => {}
                    nb::Error::Other(stm32f1xx_hal::serial::Error::Noise) => {}
                    nb::Error::Other(stm32f1xx_hal::serial::Error::Parity) => {}
                    _ => {}
                }
            }
        }
    }

    #[task(priority = 2, shared = [position, position_raw, rx_buf, utc_hour, utc_min, utc_sec])]
    async fn parse_gps_data(cx: parse_gps_data::Context) {
        let mut rx_buf = cx.shared.rx_buf;
        let mut position = cx.shared.position;
        let mut position_raw = cx.shared.position_raw;
        let mut utc_hour = cx.shared.utc_hour;
        let mut utc_min = cx.shared.utc_min;
        let mut utc_sec = cx.shared.utc_sec;

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

                                #[cfg(feature = "hell")]
                                position.lock(|position| {
                                    position[0] = pack.lat_degrees();
                                    position[1] = pack.lon_degrees();
                                    position[2] = pack.height_msl();
                                });

                                #[cfg(not(any(feature = "hell")))]
                                position_raw.lock(|position_raw| {
                                    position_raw[0] = pack.lat_degrees_raw();
                                    position_raw[1] = pack.lon_degrees_raw();
                                    position_raw[2] = pack.height_msl_raw();

                                    rprintln!("LAT: {}", pack.lat_degrees_raw());
                                    rprintln!("LONG: {}", pack.lon_degrees_raw());
                                    rprintln!("HEIGHT: {}", pack.height_msl_raw());

                                });
                            }

                            PacketRef::NavTimeUTC(pack) => {

                                utc_hour.lock(|utc_hour| {
                                    *utc_hour = pack.hour();
                                    //rprintln!("HOUR: {} ", utc_hour);
                                });

                                utc_min.lock(|utc_min| {
                                    *utc_min = pack.min();
                                    //rprintln!("MIN: {} ", utc_min);
                                });

                                utc_sec.lock(|utc_sec| {
                                    *utc_sec = pack.sec();
                                    //rprintln!("SEC: {} ", utc_sec);
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
    }




    // Measure Temperature -------------------------------------------------------------------------
    #[task(priority = 3, local = [timer_handler, timer_ticks, gpio_temp])]
    async fn tim2_tick(cx: tim2_tick::Context) {
        loop {
            let mut timer2 = &cx.local.timer_handler;
            let mut cnt_reg = &timer2.cnt;
            let x = cnt_reg.read().cnt().bits();
            timer2.cnt.write(|w| w.cnt().bits(0));
            Systick::delay(1200.millis()).await;
        }
    }

    /*

    // Debug UART ----------------------------------------------------------------------------------
    #[task(priority = 1, local = [dbg_tx], shared = [rx_buf])]
    async fn print_dbg(cx: print_dbg::Context) {
        let mut rx_buf = cx.shared.rx_buf;
        let dbg_tx = cx.local.dbg_tx;

        rx_buf.lock(|rx_buf| {
            for msx in rx_buf {
                while !dbg_tx.is_tx_empty() {};
                _ = dbg_tx.write(*msx);
            }
        });
        Systick::delay(1000.millis()).await;
    }

    #[task(binds = USART3, local = [dbg_rx])]
    fn read_dbg(cx: read_dbg::Context) {
        let rx = cx.local.dbg_rx;
        let rxb = rx.read();
        match rxb {
            Ok(_t) => {
                //toggle_led_g::spawn_after(Duration::<u64, 1, 1000>::from_ticks(10)).unwrap();
            }
            Err(e) => {
                match e {
                    nb::Error::Other(stm32f1xx_hal::serial::Error::Overrun) => {}
                    nb::Error::Other(stm32f1xx_hal::serial::Error::Framing) => {}
                    nb::Error::Other(stm32f1xx_hal::serial::Error::Noise) => {}
                    nb::Error::Other(stm32f1xx_hal::serial::Error::Parity) => {}
                    _ => {}
                }
            }
        }
    }
    */




    // ADC measurements ----------------------------------------------------------------------------
    #[task(priority = 1, shared = [led_g, vbat], local = [adc_ch_0, adc_ch_1, adc_1, shutdown, shutdown_next_cycle])]
    async fn read_adc(mut cx: read_adc::Context) {
        let mut svbat = cx.shared.vbat;

        loop {
            let vbat: u16 = cx.local.adc_1.read(cx.local.adc_ch_0).unwrap();

            svbat.lock(|svbat| {
                *svbat = vbat;
            });

            //Too large for mem
            //let mut vbat_v: f64 = (vbat as f64) * 0.00143 + 0.08;
            //rprintln!("BAT VOLT: {}", vbat_v);

            let pbut: u16 = cx.local.adc_1.read(cx.local.adc_ch_1).unwrap();

            if *cx.local.shutdown_next_cycle {
                cx.local.shutdown.set_high();
            }

            if (vbat - pbut) < 100 {
                Systick::delay(2000.millis()).await;
                let vbat_z1: u16 = cx.local.adc_1.read(cx.local.adc_ch_0).unwrap();
                let pbut_z1: u16 = cx.local.adc_1.read(cx.local.adc_ch_1).unwrap();

                if (vbat_z1 - pbut_z1) < 100 {
                    cx.shared.led_g.lock(|led_g|
                    led_g.toggle()
                    );
                    Systick::delay(2000.millis()).await;
                    cx.local.shutdown.set_high();
                }
            }

            Systick::delay(1000.millis()).await;
        }
    }
}
