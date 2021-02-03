#![deny(unsafe_code)]
#![no_std]
#![no_main]

// use panic_halt as _;
use panic_semihosting as _;

use cortex_m_rt::entry;
use cortex_m::asm;
use embedded_hal::digital::v2::OutputPin;
use stm32f3xx_hal::{pac, prelude::*, delay::Delay};

// D13 (PA5) is connected to user LED (green)
// https://www.st.com/resource/en/user_manual/dm00105823-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf

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

    // Acquire GPIO peripherals
    let mut gpio_a = dp.GPIOA.split(&mut rcc.ahb);
    let mut _gpio_b = dp.GPIOB.split(&mut rcc.ahb);
    let mut _gpio_c = dp.GPIOC.split(&mut rcc.ahb);
    let mut _gpio_d = dp.GPIOD.split(&mut rcc.ahb);
    let mut _gpio_e = dp.GPIOE.split(&mut rcc.ahb);
    let mut _gpio_f = dp.GPIOF.split(&mut rcc.ahb);
    let mut _gpio_g = dp.GPIOG.split(&mut rcc.ahb);
    let mut _gpio_h = dp.GPIOH.split(&mut rcc.ahb);

    // Configure gpio A pin 5 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let mut led = gpio_a.pa5.into_push_pull_output(&mut gpio_a.moder, &mut gpio_a.otyper).downgrade().downgrade();
    // let mut em = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);   // PB12 controls electromagnet gate

    let mut delay = Delay::new(cp.SYST, clocks);

    loop {
        // turn off indicator and wait for a while
        led.set_low().unwrap();
        asm::delay(3*8_000_000);

        // warn about turning on electromagnet by blinking LED
        
        for _ in 0..5 {
            led.set_high().unwrap();
            delay.delay_ms(100_u16);
            led.set_low().unwrap();
            delay.delay_ms(100_u16);
        }
        led.set_high().unwrap();

        // toggle electromagnet on for 500ms
        // em.set_high().unwrap();
        delay.delay_ms(500_u16);
        // em.set_low().unwrap();
    }
}
