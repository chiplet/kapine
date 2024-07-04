#![deny(unsafe_code)]
#![no_main]
#![no_std]

use rtic::app;
use rtic_monotonics::systick::prelude::*;

use panic_probe as _;
use defmt_rtt as _;

use stm32f3xx_hal::{
    gpio::{Output, PXx, PushPull},
    prelude::*,
};

// UART receive FSM STATE
#[derive(Debug)]
pub enum RxState {
    Sync1,
    Sync2,
    Command,
    Length,
    Payload(u8),
    Crc1,
    Crc2,
}

systick_monotonic!(Mono, 1000);

#[app(
        device = stm32f3xx_hal::pac,
        peripherals = true,
        dispatchers = [USB_HP, USB_LP, USB_WKUP_EXTI, TIM20_BRK]
        // dispatchers = [USB_HP, USB_LP, USB_WKUP_EXTI, TIM20_BRK, TIM20_UP, TIM20_TRG_COM, TIM20_CC, SPI4, I2C3_ER, I2C3_EV]
)]
mod app {
    use core::slice;

    use stm32f3xx_hal::{gpio::{AF5, PA5, PA6, PA7}, pac::SPI1, spi::Spi};

    use super::*;
    const SENSOR_LUT: [usize; 16] = [13, 9, 11, 2, 0, 12, 15, 5, 14, 4, 8, 3, 1, 7, 10, 6];

    #[shared]
    struct Shared {
        magnets: [PXx<Output<PushPull>>; 16],
    }

    #[local]
    struct Local {
        debug_leds: [PXx<Output<PushPull>>; 2],
        local_cnt: u8,
        spi: Spi<SPI1, (PA5<AF5<PushPull>>, PA6<AF5<PushPull>>, PA7<AF5<PushPull>>)>
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        // hprintln!("init").unwrap();

        let mut core = cx.core;
        let device = cx.device;

        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let mut exti = device.EXTI;
        let mut syscfg = device.SYSCFG.constrain(&mut rcc.apb2);

        // configure clocks
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(72.MHz())
            .hclk(72.MHz())
            .pclk1(36.MHz())
            .pclk2(72.MHz())
            .freeze(&mut flash.acr);

        // Initialize (enable) the monotonic timer (CYCCNT)
        // core.DCB.enable_trace();
        // core.DWT.enable_cycle_counter();

        Mono::start(core.SYST, 72_000_000);

        // Acquire GPIO peripherals
        let mut gpio_a = device.GPIOA.split(&mut rcc.ahb);
        let mut gpio_b = device.GPIOB.split(&mut rcc.ahb);
        let mut gpio_c = device.GPIOC.split(&mut rcc.ahb);
        let mut gpio_d = device.GPIOD.split(&mut rcc.ahb);
        let mut _gpio_e = device.GPIOE.split(&mut rcc.ahb);
        let mut _gpio_f = device.GPIOF.split(&mut rcc.ahb);
        let mut _gpio_g = device.GPIOG.split(&mut rcc.ahb);
        let mut _gpio_h = device.GPIOH.split(&mut rcc.ahb);

        // let uart_pins = (
        //     gpio_a
        //         .pa9
        //         .into_af_push_pull(&mut gpio_a.moder, &mut gpio_a.otyper, &mut gpio_a.afrh),
        //     gpio_a
        //         .pa10
        //         .into_af_push_pull(&mut gpio_a.moder, &mut gpio_a.otyper, &mut gpio_a.afrh),
        // );

        // use SPI1 peripheral to drive WS2812B RGB leds
        // let rgb_pin = gpio_a.pa7.into_af_push_pull(&mut gpio_a.moder, &mut gpio_a.otyper, &mut gpio_a.afrh);

        let spi1_sck = gpio_a
            .pa5
            .into_af_push_pull(&mut gpio_a.moder, &mut gpio_a.otyper, &mut gpio_a.afrl);
        let spi1_miso = gpio_a
            .pa6
            .into_af_push_pull(&mut gpio_a.moder, &mut gpio_a.otyper, &mut gpio_a.afrl);
        let spi1_mosi = gpio_a
            .pa7
            .into_af_push_pull(&mut gpio_a.moder, &mut gpio_a.otyper, &mut gpio_a.afrl);

        let mut spi1 = Spi::new(device.SPI1, (spi1_sck, spi1_miso, spi1_mosi), 5.MHz(), clocks, &mut rcc.apb2);


        // debug LEDs
        let mut debug_leds: [PXx<Output<PushPull>>; 2] = [
            gpio_a
                .pa0
                .into_push_pull_output(&mut gpio_a.moder, &mut gpio_a.otyper)
                .downgrade()
                .downgrade(),
            gpio_a
                .pa1
                .into_push_pull_output(&mut gpio_a.moder, &mut gpio_a.otyper)
                .downgrade()
                .downgrade(),
        ];

        // electromagnet outputs
        let mut magnets: [PXx<Output<PushPull>>; 16] = [
            gpio_b
                .pb5
                .into_push_pull_output(&mut gpio_b.moder, &mut gpio_b.otyper)
                .downgrade()
                .downgrade(),
            gpio_d
                .pd2
                .into_push_pull_output(&mut gpio_d.moder, &mut gpio_d.otyper)
                .downgrade()
                .downgrade(),
            gpio_c
                .pc11
                .into_push_pull_output(&mut gpio_c.moder, &mut gpio_c.otyper)
                .downgrade()
                .downgrade(),
            gpio_a
                .pa12
                .into_push_pull_output(&mut gpio_a.moder, &mut gpio_a.otyper)
                .downgrade()
                .downgrade(),
            gpio_a
                .pa8
                .into_push_pull_output(&mut gpio_a.moder, &mut gpio_a.otyper)
                .downgrade()
                .downgrade(),
            gpio_c
                .pc8
                .into_push_pull_output(&mut gpio_c.moder, &mut gpio_c.otyper)
                .downgrade()
                .downgrade(),
            gpio_c
                .pc6
                .into_push_pull_output(&mut gpio_c.moder, &mut gpio_c.otyper)
                .downgrade()
                .downgrade(),
            gpio_b
                .pb14
                .into_push_pull_output(&mut gpio_b.moder, &mut gpio_b.otyper)
                .downgrade()
                .downgrade(),
            gpio_b
                .pb11
                .into_push_pull_output(&mut gpio_b.moder, &mut gpio_b.otyper)
                .downgrade()
                .downgrade(),
            gpio_b
                .pb2
                .into_push_pull_output(&mut gpio_b.moder, &mut gpio_b.otyper)
                .downgrade()
                .downgrade(),
            gpio_c
                .pc5
                .into_push_pull_output(&mut gpio_c.moder, &mut gpio_c.otyper)
                .downgrade()
                .downgrade(),
            gpio_a
                .pa4
                .into_push_pull_output(&mut gpio_a.moder, &mut gpio_a.otyper)
                .downgrade()
                .downgrade(), // m11 (bodge)
            gpio_c
                .pc3
                .into_push_pull_output(&mut gpio_c.moder, &mut gpio_c.otyper)
                .downgrade()
                .downgrade(),
            gpio_c
                .pc1
                .into_push_pull_output(&mut gpio_c.moder, &mut gpio_c.otyper)
                .downgrade()
                .downgrade(),
            gpio_b
                .pb9
                .into_push_pull_output(&mut gpio_b.moder, &mut gpio_b.otyper)
                .downgrade()
                .downgrade(),
            gpio_b
                .pb7
                .into_push_pull_output(&mut gpio_b.moder, &mut gpio_b.otyper)
                .downgrade()
                .downgrade(),
        ];

        for magnet in &mut magnets {
            magnet
                .set_low()
                .expect("could not turn magnet off at startup");
        }


        defmt::info!("spawning software tasks");
        // spawn software tasks
        blink_task::spawn().unwrap();
        rgb_task::spawn().unwrap();

        defmt::info!("init done :)");

        (
            Shared {
                magnets,
            },
            Local {
                debug_leds,
                local_cnt: 0u8,
                spi: spi1,
            },
        )
    }

    #[task(priority = 3, local = [debug_leds, local_cnt], shared=[magnets])]
    async fn blink_task(cx: blink_task::Context) {
        let mut magnets = cx.shared.magnets;
        loop {
            let idx = *cx.local.local_cnt;
            cx.local.debug_leds[0].toggle().expect("Could not toggle debug led");
            let mut magnets = magnets.lock(|mags| {
                mags[idx as usize].toggle().unwrap();
            });
            *cx.local.local_cnt = if *cx.local.local_cnt + 1 == 16 { 0 } else { *cx.local.local_cnt + 1 };
            Mono::delay(50.millis()).await;
        }
    }

    #[task(priority = 4, local = [spi])]
    async fn rgb_task(cx: rgb_task::Context) {
        const ZERO: u8 = 0b0100_0000;
        const ONE: u8 = 0b0111_1110;

        let r = 128u8;
        let g = 0u8;
        let b = 255u8;
        let mut rgb: [u8; 24] = [ZERO; 24];

        loop {
            for i in 0..8 {
                rgb[i + 0] = if (r >> i) & 1 == 1 { ONE } else { ZERO };
                rgb[i + 8] = if (g >> i) & 1 == 1 { ONE } else { ZERO };
                rgb[i + 16] = if (b >> i) & 1 == 1 { ONE } else { ZERO };
                // defmt::trace!("r[{}] = {}", i, rgb[i]);
            }
            let mut msg_sending = rgb;
            let _msg_received = cx.local.spi.transfer(&mut msg_sending).unwrap();
            Mono::delay(1.millis()).await;
        }
    }
}
