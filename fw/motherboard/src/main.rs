#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m_semihosting::{debug, hprint, hprintln};
use panic_semihosting as _;
use nb::block;
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

// Inter Process Communication (IPC) types
use heapless::{
    spsc::{Producer, Consumer, Queue},
};

// interrupts
use pac::Interrupt::{
    self,
    USART1_EXTI25
};

use kapine_packet::Packet;

const RX_BUF_SIZE: usize = 10;

// UART receive FSM state
pub enum RxState {
    sync1,
    sync2,
    command,
    length,
    payload(u8),
    crc1,
    crc2
}

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

const RX_QUEUE_LEN: usize = 16;

#[rtic::app(device = stm32f3xx_hal::pac, peripherals = true)]
const APP: () = {

    struct Resources {
        TX: Tx<pac::USART1>,
        RX: Rx<pac::USART1>,
        #[init(RxState::sync1)]
        RX_STATE: RxState,
        RX_BUF: Packet,
        RX_P: Producer<'static, u32, RX_QUEUE_LEN>,
        RX_C: Consumer<'static, u32, RX_QUEUE_LEN>,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        // NOTE: static mutable variables must appear at the top of the function!
        static mut Q: Queue<u32, RX_QUEUE_LEN> = Queue::new();

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

        let dma1 = device.DMA1.split(&mut rcc.ahb);
        // DMA channel selection depends on the peripheral:
        // - USART1: TX = 4, RX = 5
        // - USART2: TX = 6, RX = 7
        // - USART3: TX = 3, RX = 2
        let (tx_channel, rx_channel) = (dma1.ch4, dma1.ch5);


        rtic::pend(Interrupt::USART1_EXTI25);

        // init empty receive Packet structu
        let payload = [0u8; 255];
        let packet = Packet::new(0u8, Some(&payload));

        let (RX_P, RX_C) = Q.split();

        init::LateResources {
            TX: tx,
            RX: rx,
            RX_BUF: packet,
            RX_P,
            RX_C
        }
    }

    /*
    #[idle( resources = [TX, RX_BUF, RX_C] )]
    fn idle(mut cx: idle::Context) -> ! {
        // rtic::pend(Interrupt::USART1_EXTI25);

        hprintln!("idle").unwrap();

        loop {
            let mut payload = [0u8; 255];
            let mut buf_local = Packet::new(0u8, Some(&payload));

            cx.resources.RX_BUF.lock(|RX_BUF| {
                // manually clone all RX_BUF fields
                buf_local.sync = RX_BUF.sync;
                buf_local.command = RX_BUF.command;
                buf_local.length = RX_BUF.length;
                buf_local.checksum = RX_BUF.checksum;
                for (src, dest) in RX_BUF.payload.iter().zip(buf_local.payload.iter_mut()) {
                    *dest = *src;
                }
            });
            // hprint!("{:?} ", buf_local).unwrap();
        }
    }
    */


    /* tx example
    for c in b"rx buffer overrun error\r\n" {
        block!(cx.resources.TX.write(*c)).ok();
    }
    */


    // TODO: move FSM logic out of receive interrupt, this should only push the raw received bytes
    // into a FIFO or other similar IPC structure. Then it's possible to create a rx task for the
    // packet deserialisation.
    #[task(binds = USART1_EXTI25, priority = 2, resources=[RX,RX_STATE,RX_BUF,TX,RX_P])]
    fn usart1(cx: usart1::Context) {
        let state: &mut RxState = cx.resources.RX_STATE;
        let rx_buf: &mut Packet = cx.resources.RX_BUF;
         
        let rx_byte: Option<u8> = match cx.resources.RX.read() {
            Ok(c) => {
                Some(c)
                // cx.resources.RX_BUF.push(c);
            }
            Err(e) => {
                match e {
                    nb::Error::WouldBlock => None, // no data available
                    nb::Error::Other(stm32f3xx_hal::serial::Error::Overrun) => panic!("rx buffer overrun"),
                    _ => panic!("{:?}", e),
                }
            }
        };

        // only mutate receiver state if a new byte was received
        if let Some(rx_byte) = rx_byte {

            // state machine logic
            match state {
                RxState::sync1 => {
                    match rx_byte {
                        0xAA => *state = RxState::sync2,
                        _ => *state = RxState::sync1,
                    }
                },
                RxState::sync2 => {
                    match rx_byte {
                        0x55 => *state = RxState::command,
                        _ => *state = RxState::sync1,
                    }
                },
                RxState::command => {
                    *state = RxState::length;

                    rx_buf.command = rx_byte;
                },
                RxState::length => {
                    *state = RxState::payload(rx_byte);

                    rx_buf.length = rx_byte;

                },
                RxState::payload(i) => {
                    // FIXME: something strange going on here with borrowing
                    // TODO: come up with something simpler
                    let i: u8 = *i; 
                    match i {
                        0 => *state = RxState::crc1,
                        _ => *state = RxState::payload(i-1),
                    }

                    if i != 0 {
                        rx_buf.payload.unwrap()[(255 - i) as usize] = rx_byte;

                        for c in b"length!\r\n" {
                            block!(cx.resources.TX.write(*c)).ok();
                        }
                    }
                },
                RxState::crc1 => {
                    *state = RxState::crc2;

                    rx_buf.checksum = 0u16;
                    rx_buf.checksum |= rx_byte as u16;
                },
                RxState::crc2 => {
                    *state = RxState::sync1;

                    rx_buf.checksum |= (rx_byte as u16) << 8;
                    // TODO: push packet to relevant queue

                    for c in b"received!\r\n" {
                        block!(cx.resources.TX.write(*c)).ok();
                    }
                }
            }
        }
    }

};
