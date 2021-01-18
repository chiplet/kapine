//! Blinks an LED
//!
//! This assumes that a LED is connected to pc13 as is the case on the blue pill board.
//!
//! Note: Without additional hardware, PC13 should not be used to drive an LED, see page 5.1.2 of
//! the reference manual for an explanation. This is not an issue on the blue pill.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::{pac, prelude::*, delay::Delay};

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();    // get access to cortex-m core peripherals
    let dp = pac::Peripherals::take().unwrap();         // get access to mcu device peripherals

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Acquire the GPIOC peripheral
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh); // PC13 contols dev board LED
    let mut em = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);  // PB12 controls electromagnet gate

    let mut delay = Delay::new(cp.SYST, clocks);

    loop {
        // turn off indicator and wait for a while
        led.set_high().unwrap();
        delay.delay_ms(3000_u16);

        // warn about turning on electromagnet by blinking LED
        
        for _ in 0..5 {
            led.set_low().unwrap();
            delay.delay_ms(100_u16);
            led.set_high().unwrap();
            delay.delay_ms(100_u16);
        }
        led.set_low().unwrap();

        // toggle electromagnet on for 500ms
        em.set_high().unwrap();
        delay.delay_ms(500_u16);
        em.set_low().unwrap();
    }
}
