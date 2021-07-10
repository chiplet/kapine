#![deny(unsafe_code)]
#![no_std]
#![no_main]

use core::cell::RefCell;

// use panic_halt as _;
use panic_semihosting as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use cortex_m::asm;
use cortex_m::peripheral::SYST;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::interrupt::Mutex;
// use embedded_hal::digital::v2::OutputPin;
use stm32f3xx_hal as hal;
use hal::{pac, prelude::*, delay::Delay};
use hal::time::rate::Megahertz; // embedded-time
use hal::timer::Timer;
use hal::gpio::{gpioc, Output, PushPull};
use ws2812_timer_delay as ws2812;
use ws2812::Ws2812;
// use cortex_m::peripheral::Peripherals;
use smart_leds::{brightness, SmartLedsWrite, RGB8};

type LedPin = gpioc::PC13<Output<PushPull>>;
static LED: Mutex<RefCell<Option<LedPin>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {

    let CORE_MHZ: u32 = 48;
    let CORE_HZ: u32 = CORE_MHZ * 1_000_000;

    let cp = cortex_m::Peripherals::take().unwrap();    // get access to cortex-m core peripherals
    let dp = pac::Peripherals::take().unwrap();         // get access to mcu device peripherals

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.sysclk(CORE_MHZ.MHz()).freeze(&mut flash.acr);

    // Acquire GPIO peripherals
    let mut gpio_a = dp.GPIOA.split(&mut rcc.ahb);
    let mut gpio_b = dp.GPIOB.split(&mut rcc.ahb);
    let mut gpio_c = dp.GPIOC.split(&mut rcc.ahb);
    let mut gpio_d = dp.GPIOD.split(&mut rcc.ahb);
    let mut gpio_e = dp.GPIOE.split(&mut rcc.ahb);
    let mut gpio_f = dp.GPIOF.split(&mut rcc.ahb);
    let mut gpio_g = dp.GPIOG.split(&mut rcc.ahb);
    let mut gpio_h = dp.GPIOH.split(&mut rcc.ahb);

    // PA5 (Arduino Pin 13) is connected to user LED (green)
    // https://www.st.com/resource/en/user_manual/dm00105823-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf
    // Configure gpio A pin 5 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let mut led = RefCell::new(gpio_a.pa5.into_push_pull_output(&mut gpio_a.moder, &mut gpio_a.otyper).downgrade().downgrade());
    let mut rgb = gpio_c.pc13.into_push_pull_output(&mut gpio_c.moder, &mut gpio_c.otyper);

    // temporary helper for turning on all LEDs
    // m0.borrow_mut().set_high().unwrap();

    // let mut delay = Delay::new(cp.SYST, clocks);

    /*
    loop {
        // turn off indicator and wait for a while
        led.borrow_mut().set_high().unwrap();
        asm::delay(CORE_HZ);
        led.borrow_mut().set_low().unwrap();
        asm::delay(CORE_HZ);
    }
    */

    // if let (Some(p), Some(cp)) = (stm32f3xx_hal::stm32::Peripherals::take(), cortex_m::peripheral::Peripherals::take()) {

    /* (Re-)configure PC13 as output */
    // let ws_data_pin = cortex_m::interrupt::free(move |cs| gpio_c.pc13.into_push_pull_output(&mut gpio_c.moder, &mut gpio_c.otyper));
    // let ws_data_pin = gpio_c.pc13.into_push_pull_output(&mut gpio_c.moder, &mut gpio_c.otyper);


    // move ownership of the rgb to global LED
    // cortex_m::interrupt::free( |cs| *LED.borrow(cs).borrow_mut() = Some(rgb));

    let timer = Timer::tim1(dp.TIM1, 3_000_000.Hz(), clocks, &mut rcc.apb2);

    // Get delay provider
    let mut delay = Delay::new(cp.SYST, clocks);

    let mut ws = Ws2812::new(timer, rgb);

    const NUM_LEDS: usize = 1;
    let mut data = [RGB8::default(); NUM_LEDS];

    loop {
        for j in 0..(256 * 5) {
            for i in 0..NUM_LEDS {
                data[i] = wheel((((i * 256) as u16 / NUM_LEDS as u16 + j as u16) & 255) as u8);
            }
            ws.write(brightness(data.iter().cloned(), 32)).unwrap();
            delay.delay_ms(5u8);
        }
    }
}

/// Input a value 0 to 255 to get a color value
/// The colours are a transition r - g - b - back to r.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        return (255 - wheel_pos * 3, 0, wheel_pos * 3).into();
    }
    if wheel_pos < 170 {
        wheel_pos -= 85;
        return (0, wheel_pos * 3, 255 - wheel_pos * 3).into();
    }
    wheel_pos -= 170;
    (wheel_pos * 3, 255 - wheel_pos * 3, 0).into()
}
