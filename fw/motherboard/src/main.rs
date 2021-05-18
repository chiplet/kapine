#![deny(unsafe_code)]
#![no_std]
#![no_main]

use core::cell::RefCell;

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
    let mut gpio_b = dp.GPIOB.split(&mut rcc.ahb);
    let mut gpio_c = dp.GPIOC.split(&mut rcc.ahb);
    let mut gpio_d = dp.GPIOD.split(&mut rcc.ahb);
    let mut gpio_e = dp.GPIOE.split(&mut rcc.ahb);
    let mut gpio_f = dp.GPIOF.split(&mut rcc.ahb);
    let mut gpio_g = dp.GPIOG.split(&mut rcc.ahb);
    let mut gpio_h = dp.GPIOH.split(&mut rcc.ahb);

    // Configure gpio A pin 5 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let mut debug_1 = gpio_a.pa0.into_push_pull_output(&mut gpio_a.moder, &mut gpio_a.otyper).downgrade().downgrade();
    let mut debug_2 = gpio_a.pa1.into_push_pull_output(&mut gpio_a.moder, &mut gpio_a.otyper).downgrade().downgrade();
    // let mut em = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);   // PB12 controls electromagnet gate

    // electromagnet signals
    let mut m0 = RefCell::new(gpio_b.pb5.into_push_pull_output(&mut gpio_b.moder, &mut gpio_b.otyper).downgrade().downgrade());
    let mut m1 = RefCell::new(gpio_d.pd2.into_push_pull_output(&mut gpio_d.moder, &mut gpio_d.otyper).downgrade().downgrade());
    let mut m2 = RefCell::new(gpio_c.pc11.into_push_pull_output(&mut gpio_c.moder, &mut gpio_c.otyper).downgrade().downgrade());
    let mut m3 = RefCell::new(gpio_a.pa12.into_push_pull_output(&mut gpio_a.moder, &mut gpio_a.otyper).downgrade().downgrade());
    let mut m4 = RefCell::new(gpio_a.pa8.into_push_pull_output(&mut gpio_a.moder, &mut gpio_a.otyper).downgrade().downgrade());
    let mut m5 = RefCell::new(gpio_c.pc8.into_push_pull_output(&mut gpio_c.moder, &mut gpio_c.otyper).downgrade().downgrade());
    let mut m6 = RefCell::new(gpio_c.pc6.into_push_pull_output(&mut gpio_c.moder, &mut gpio_c.otyper).downgrade().downgrade());
    let mut m7 = RefCell::new(gpio_b.pb14.into_push_pull_output(&mut gpio_b.moder, &mut gpio_b.otyper).downgrade().downgrade());
    let mut m8 = RefCell::new(gpio_b.pb11.into_push_pull_output(&mut gpio_b.moder, &mut gpio_b.otyper).downgrade().downgrade());
    let mut m9 = RefCell::new(gpio_b.pb2.into_push_pull_output(&mut gpio_b.moder, &mut gpio_b.otyper).downgrade().downgrade());
    let mut m10 = RefCell::new(gpio_c.pc5.into_push_pull_output(&mut gpio_c.moder, &mut gpio_c.otyper).downgrade().downgrade());
    let mut m11 = RefCell::new(gpio_a.pa3.into_push_pull_output(&mut gpio_a.moder, &mut gpio_a.otyper).downgrade().downgrade());
    let mut m12 = RefCell::new(gpio_c.pc3.into_push_pull_output(&mut gpio_c.moder, &mut gpio_c.otyper).downgrade().downgrade());
    let mut m13 = RefCell::new(gpio_c.pc1.into_push_pull_output(&mut gpio_c.moder, &mut gpio_c.otyper).downgrade().downgrade());
    let mut m14 = RefCell::new(gpio_b.pb9.into_push_pull_output(&mut gpio_b.moder, &mut gpio_b.otyper).downgrade().downgrade());
    let mut m15 = RefCell::new(gpio_b.pb7.into_push_pull_output(&mut gpio_b.moder, &mut gpio_b.otyper).downgrade().downgrade());

    // temporary helper for turning on all LEDs
    let mut leds_on = || {
        m0.borrow_mut().set_high().unwrap();
        m1.borrow_mut().set_high().unwrap();
        m2.borrow_mut().set_high().unwrap();
        m3.borrow_mut().set_high().unwrap();
        m4.borrow_mut().set_high().unwrap();
        m5.borrow_mut().set_high().unwrap();
        m6.borrow_mut().set_high().unwrap();
        m7.borrow_mut().set_high().unwrap();
        m8.borrow_mut().set_high().unwrap();
        m9.borrow_mut().set_high().unwrap();
        m10.borrow_mut().set_high().unwrap();
        m11.borrow_mut().set_high().unwrap();
        m12.borrow_mut().set_high().unwrap();
        m13.borrow_mut().set_high().unwrap();
        m14.borrow_mut().set_high().unwrap();
        m15.borrow_mut().set_high().unwrap();
    };

    // temporary helper for turning on all LEDs
    let mut leds_off = || {
        m0.borrow_mut().set_low().unwrap();
        m1.borrow_mut().set_low().unwrap();
        m2.borrow_mut().set_low().unwrap();
        m3.borrow_mut().set_low().unwrap();
        m4.borrow_mut().set_low().unwrap();
        m5.borrow_mut().set_low().unwrap();
        m6.borrow_mut().set_low().unwrap();
        m7.borrow_mut().set_low().unwrap();
        m8.borrow_mut().set_low().unwrap();
        m9.borrow_mut().set_low().unwrap();
        m10.borrow_mut().set_low().unwrap();
        m11.borrow_mut().set_low().unwrap();
        m12.borrow_mut().set_low().unwrap();
        m13.borrow_mut().set_low().unwrap();
        m14.borrow_mut().set_low().unwrap();
        m15.borrow_mut().set_low().unwrap();
    };


    let mut delay = Delay::new(cp.SYST, clocks);

    loop {
        // turn off indicator and wait for a while
        debug_1.set_low().unwrap();
        debug_2.set_high().unwrap();
        leds_off();
        asm::delay(3*8_000_000);

        // warn about turning on electromagnet by blinking LED
        
        debug_2.set_low().unwrap();
        for _ in 0..5 {
            debug_1.set_high().unwrap();
            leds_on();
            delay.delay_ms(100_u16);
            debug_1.set_low().unwrap();
            leds_off();
            delay.delay_ms(100_u16);
        }
        debug_1.set_high().unwrap();

        // toggle electromagnet on for 500ms
        // em.set_high().unwrap();
        delay.delay_ms(500_u16);
        // em.set_low().unwrap();
    }
}
