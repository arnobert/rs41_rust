/*
RS 41 LEDs
 */

use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::afio::Parts;
use stm32f1xx_hal::gpio::gpiob::PB8;
use stm32f1xx_hal::gpio::gpiob::PB7;
use stm32f1xx_hal::gpio::{Output, PushPull}; // STM32F1 specific functions


pub struct RS41_Led {
   led_r: PB8<Output<PushPull>>,
   led_g: PB7<Output<PushPull>>,
}

impl RS41_Led {
   pub fn init(mut gpiob: stm32f1xx_hal::gpio::gpiob::Parts) -> RS41_Led {
      let mut ledr = gpiob.pb8.into_push_pull_output(&mut gpiob.crh);
      let mut ledg = gpiob.pb7.into_push_pull_output(&mut gpiob.crl);
      RS41_Led{led_r: ledr, led_g: ledg}
   }

   pub fn led_r_on(&mut self) {
      self.led_r.set_low();
   }
   pub fn led_r_off(&mut self) {
      self.led_r.set_high();
   }

   pub fn led_g_on(&mut self) {
      self.led_g.set_low();
   }
   pub fn led_g_off(&mut self) {
      self.led_g.set_high();
   }

}