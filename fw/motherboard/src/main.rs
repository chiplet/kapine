#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m_semihosting::{debug, hprint, hprintln};
use panic_semihosting as _;
use rtic::app;

use stm32f3xx_hal::pac as pac;

use stm32f3xx_hal::{
    prelude::*,
    serial::{
        Serial,
        Tx,
        Rx,
        Event::Rxne
    }
};

// interrupts
use pac::Interrupt::{
    self,
    USART1_EXTI25
};

use kapine_packet::Packet;

const RX_BUF_SIZE: usize = 10;

pub struct Buffer {
    index: usize,
    pub data: [u8; RX_BUF_SIZE],
}

impl Buffer {
    const fn new() -> Self {
        Buffer {
            index: 0,
            data: [0; RX_BUF_SIZE],
        }
    }

    // TODO: add error handling
    fn push(&mut self, data: u8) {
        self.data[self.index] = data;
        self.index += 1;
    }
}

#[rtic::app(device = stm32f3xx_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        TX: Tx<pac::USART1>,
        RX: Rx<pac::USART1>,
        #[init(Buffer::new())]
        RX_BUF: Buffer,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        hprintln!("init").unwrap();

        let _core = cx.core;
        let device = cx.device;

        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();

        // configure clocks
        let clocks = rcc.cfgr.freeze(&mut flash.acr);

        let mut gpioa = device.GPIOA.split(&mut rcc.ahb);

        let pins = (
            gpioa.pa9.into_af7(&mut gpioa.moder, &mut gpioa.afrh),
            gpioa.pa10.into_af7(&mut gpioa.moder, &mut gpioa.afrh),
        );
        let mut serial = Serial::usart1(device.USART1, pins, 9600.bps(), clocks, &mut rcc.apb2);
        serial.listen(stm32f3xx_hal::serial::Event::Rxne);

        let (tx, rx) = serial.split();

        rtic::pend(Interrupt::USART1_EXTI25);

        // let RX_BUF: 

        init::LateResources {
            TX: tx,
            RX: rx,
        }
    }

    #[idle( resources = [RX_BUF] )]
    fn idle(mut cx: idle::Context) -> ! {


        // rtic::pend(Interrupt::USART1_EXTI25);

        hprintln!("idle").unwrap();

        loop {
            cx.resources.RX_BUF.lock(|RX_BUF| {
                if (RX_BUF.data[0] != 0) {
                    hprint!("{:?} ", RX_BUF.data);
                }
            });
        }
    }

    #[task(binds = USART1_EXTI25, priority = 1, resources=[RX,RX_BUF,TX])]
    fn usart1(cx: usart1::Context) {

        match cx.resources.RX.read() {
            Ok(c) => {
                cx.resources.RX_BUF.push(c);
            }
            Err(e) => (),
        };
    }
};
