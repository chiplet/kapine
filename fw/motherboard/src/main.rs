#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m_semihosting::hprintln;
use panic_semihosting as _;
use nb::block;
use rtic::{
    app,
    cyccnt::{
        // Instant,
        U32Ext as _},
};

use stm32f3xx_hal::pac as pac;
use stm32f3xx_hal::{
    prelude::*,
    Toggle,
    timer::{
        Timer,
        Event,
    },
    gpio::{
        Edge,
        Input, Output, PushPull,
        PXx, PA9, PA10, AF7
    },
    serial::{
        self,
        Serial,
        Tx,
        Rx,
    },
};

// Inter Process Communication (IPC) types
use heapless::{
    spsc::{Producer, Consumer, Queue},
};

// interrupts
use pac::{
    Interrupt,
    TIM2,
};

use kapine_packet::Packet;

// UART receive FSM STATE
enum RxState {
    Sync1,
    Sync2,
    Command,
    Length,
    Payload(u8),
    Crc1,
    Crc2
}


const RX_QUEUE_LEN: usize = 512;
const TX_QUEUE_LEN: usize = 4;
const SENSOR_LUT: [usize; 16] = [13, 9, 11, 2, 0, 12, 15, 5, 14, 4, 8, 3, 1, 7, 10, 6];

fn sensor_handler(index: usize, magnets: &mut [PXx<Output<PushPull>>], timer: &mut Timer<TIM2>) {
    for magnet in magnets.iter_mut() {
        magnet.set_low().unwrap();
    }
    let mut index: isize = index as isize;
    index = index - 1;
    if index < 0 {
        index = 15;
    }
    magnets[index as usize].set_high().unwrap();
    timer.start(2.Hz());
}

#[app(device = stm32f3xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {

    struct Resources {
        TX: Tx<pac::USART1, PA9<AF7<PushPull>>>,
        RX: Rx<pac::USART1, PA10<AF7<PushPull>>>,
        #[init(RxState::sync1)]
        RX_STATE: RxState,
        RX_BUF: Packet,
        RX_P: Producer<'static, u8, RX_QUEUE_LEN>,
        RX_C: Consumer<'static, u8, RX_QUEUE_LEN>,
        TX_P: Producer<'static, Packet, TX_QUEUE_LEN>,
        TX_C: Consumer<'static, Packet, TX_QUEUE_LEN>,
        MAGNETS: [PXx<Output<PushPull>>; 16],
        SENSORS: [PXx<Input>; 16],
        TIMER: Timer<TIM2>,
    }


    #[init(spawn = [rx_task, tx_task, moi_task])]
    fn init(cx: init::Context) -> init::LateResources {
        // NOTE: static mutable variables must appear at the top of the function!
        static mut Q: Queue<u8, RX_QUEUE_LEN> = Queue::new();
        static mut TX_Q: Queue<Packet, TX_QUEUE_LEN> = Queue::new();

        hprintln!("init").unwrap();

        let mut core = cx.core;
        let device = cx.device;

        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let mut exti = device.EXTI;
        let mut syscfg = device.SYSCFG.constrain(&mut rcc.apb2);

        // configure clocks
        let clocks = rcc.cfgr
            .use_hse(8.MHz())
            .sysclk(72.MHz())
            .hclk(72.MHz())
            .pclk1(36.MHz())
            .pclk2(72.MHz())
            .freeze(&mut flash.acr);

        // Initialize (enable) the monotonic timer (CYCCNT)
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();

        // Acquire GPIO peripherals
        let mut gpio_a = device.GPIOA.split(&mut rcc.ahb);
        let mut gpio_b = device.GPIOB.split(&mut rcc.ahb);
        let mut gpio_c = device.GPIOC.split(&mut rcc.ahb);
        let mut gpio_d = device.GPIOD.split(&mut rcc.ahb);
        let mut _gpio_e = device.GPIOE.split(&mut rcc.ahb);
        let mut _gpio_f = device.GPIOF.split(&mut rcc.ahb);
        let mut _gpio_g = device.GPIOG.split(&mut rcc.ahb);
        let mut _gpio_h = device.GPIOH.split(&mut rcc.ahb);

        let pins = (
            gpio_a.pa9.into_af7_push_pull(&mut gpio_a.moder, &mut gpio_a.otyper, &mut gpio_a.afrh),
            gpio_a.pa10.into_af7_push_pull(&mut gpio_a.moder, &mut gpio_a.otyper, &mut gpio_a.afrh),
        );
        let mut serial = Serial::new(device.USART1, pins, 115_200.Bd(), clocks, &mut rcc.apb2);
        // serial.listen(stm32f3xx_hal::serial::Event::Rxne);
        serial.configure_interrupt(serial::Event::ReceiveDataRegisterNotEmpty, Toggle::On);

        // electromagnet outputs
        let mut MAGNETS: [PXx<Output<PushPull>>; 16] = [
            gpio_b.pb5.into_push_pull_output(&mut gpio_b.moder, &mut gpio_b.otyper).downgrade().downgrade(),
            gpio_d.pd2.into_push_pull_output(&mut gpio_d.moder, &mut gpio_d.otyper).downgrade().downgrade(),
            gpio_c.pc11.into_push_pull_output(&mut gpio_c.moder, &mut gpio_c.otyper).downgrade().downgrade(),
            gpio_a.pa12.into_push_pull_output(&mut gpio_a.moder, &mut gpio_a.otyper).downgrade().downgrade(),
            gpio_a.pa8.into_push_pull_output(&mut gpio_a.moder, &mut gpio_a.otyper).downgrade().downgrade(),
            gpio_c.pc8.into_push_pull_output(&mut gpio_c.moder, &mut gpio_c.otyper).downgrade().downgrade(),
            gpio_c.pc6.into_push_pull_output(&mut gpio_c.moder, &mut gpio_c.otyper).downgrade().downgrade(),
            gpio_b.pb14.into_push_pull_output(&mut gpio_b.moder, &mut gpio_b.otyper).downgrade().downgrade(),
            gpio_b.pb11.into_push_pull_output(&mut gpio_b.moder, &mut gpio_b.otyper).downgrade().downgrade(),
            gpio_b.pb2.into_push_pull_output(&mut gpio_b.moder, &mut gpio_b.otyper).downgrade().downgrade(),
            gpio_c.pc5.into_push_pull_output(&mut gpio_c.moder, &mut gpio_c.otyper).downgrade().downgrade(),
            gpio_a.pa4.into_push_pull_output(&mut gpio_a.moder, &mut gpio_a.otyper).downgrade().downgrade(), // m11 (bodge)
            gpio_c.pc3.into_push_pull_output(&mut gpio_c.moder, &mut gpio_c.otyper).downgrade().downgrade(),
            gpio_c.pc1.into_push_pull_output(&mut gpio_c.moder, &mut gpio_c.otyper).downgrade().downgrade(),
            gpio_b.pb9.into_push_pull_output(&mut gpio_b.moder, &mut gpio_b.otyper).downgrade().downgrade(),
            gpio_b.pb7.into_push_pull_output(&mut gpio_b.moder, &mut gpio_b.otyper).downgrade().downgrade(),
        ];

        for magnet in &mut MAGNETS {
            magnet.set_low().expect("could not turn magnet off at startup");
        }


        // photo-gate sensor inputs
        // enable EXTIx for x in {0, 1, 2, 4, 6, 7, 8, 9, 10, 11, 12, 13, 15}
        // required 3, 5, 14
        let mut SENSORS: [PXx<Input>; 16] = [
            gpio_b.pb4.into_pull_up_input(&mut gpio_b.moder, &mut gpio_b.pupdr).downgrade().downgrade(),
            gpio_c.pc12.into_pull_up_input(&mut gpio_c.moder, &mut gpio_c.pupdr).downgrade().downgrade(),
            gpio_a.pa3.into_pull_up_input(&mut gpio_a.moder, &mut gpio_a.pupdr).downgrade().downgrade(), // s2 (bodge)
            gpio_a.pa11.into_pull_up_input(&mut gpio_a.moder, &mut gpio_a.pupdr).downgrade().downgrade(),
            gpio_c.pc9.into_pull_up_input(&mut gpio_c.moder, &mut gpio_c.pupdr).downgrade().downgrade(),
            gpio_c.pc7.into_pull_up_input(&mut gpio_c.moder, &mut gpio_c.pupdr).downgrade().downgrade(),
            gpio_b.pb15.into_pull_up_input(&mut gpio_b.moder, &mut gpio_b.pupdr).downgrade().downgrade(),
            gpio_b.pb13.into_pull_up_input(&mut gpio_b.moder, &mut gpio_b.pupdr).downgrade().downgrade(),
            gpio_b.pb10.into_pull_up_input(&mut gpio_b.moder, &mut gpio_b.pupdr).downgrade().downgrade(),
            gpio_b.pb1.into_pull_up_input(&mut gpio_b.moder, &mut gpio_b.pupdr).downgrade().downgrade(),
            gpio_c.pc14.into_pull_up_input(&mut gpio_c.moder, &mut gpio_c.pupdr).downgrade().downgrade(), // s10 (bodge)
            gpio_a.pa2.into_pull_up_input(&mut gpio_a.moder, &mut gpio_a.pupdr).downgrade().downgrade(),
            gpio_a.pa5.into_pull_up_input(&mut gpio_a.moder, &mut gpio_a.pupdr).downgrade().downgrade(), // s12 (bodge)
            gpio_c.pc0.into_pull_up_input(&mut gpio_c.moder, &mut gpio_c.pupdr).downgrade().downgrade(),
            gpio_b.pb8.into_pull_up_input(&mut gpio_b.moder, &mut gpio_b.pupdr).downgrade().downgrade(),
            gpio_b.pb6.into_pull_up_input(&mut gpio_b.moder, &mut gpio_b.pupdr).downgrade().downgrade(),
        ];


        // enable sensor interrupts
        for sensor in &mut SENSORS {
            sensor.trigger_on_edge(&mut exti, Edge::Falling);
            syscfg.select_exti_interrupt_source(sensor);
            sensor.enable_interrupt(&mut exti);
        }

        let (tx, rx) = serial.split();

        rtic::pend(Interrupt::USART1_EXTI25);

        // init empty receive Packet structu
        let payload = [0u8; 255];
        let packet = Packet::from(0u8, Some(&payload));

        let (RX_P, RX_C) = Q.split();
        let (TX_P, TX_C) = TX_Q.split();

        let mut TIMER = Timer::tim2(device.TIM2, 2.Hz(), clocks, &mut rcc.apb1);
        TIMER.enable_interrupt(Event::Update);
        TIMER.stop();

        // spawn software tasks
        cx.spawn.tx_task().unwrap();
        cx.spawn.rx_task().unwrap();

        init::LateResources {
            TX: tx,
            RX: rx,
            RX_BUF: packet,
            RX_P,
            RX_C,
            TX_P,
            TX_C,
            MAGNETS,
            SENSORS,
            TIMER,    
        }
    }

    // reads a packet from the tx queue and transmits it over serial
    #[task(schedule = [tx_task], priority = 3, resources = [TX, TX_C])]
    fn tx_task(cx: tx_task::Context) {
        // TODO: make tx_buf length a constant in kapine-packet
        let mut tx_buf = [0u8; 261];

        if let Some(packet) = cx.resources.TX_C.dequeue() {
            let length: usize = packet.write_bytes(&mut tx_buf[..]) as usize;

            for byte in tx_buf[..length].iter() {
                block!(cx.resources.TX.write(*byte)).unwrap();
                block!(cx.resources.TX.flush()).unwrap();
            }
        }

        cx.schedule.tx_task(cx.scheduled + 80_000.cycles()).unwrap();
    }

    #[task(priority = 2, resources = [TX_P])]
    fn moi_task(mut cx: moi_task::Context) {
        cx.resources.TX_P.lock(|TX_P| {
            TX_P.enqueue(Packet::from(3, Some(b"jee :D"))).unwrap();
        });
    }

    #[idle(spawn = [moi_task])]
    fn idle(_cx: idle::Context) -> ! {
        hprintln!("idle").unwrap();
        loop {}
    }

    // shut all magnets of immediately
    #[task(binds = TIM2, priority = 6, resources = [TIMER, MAGNETS])]
    fn kill_magnets(cx: kill_magnets::Context) {
        if cx.resources.TIMER.is_event_triggered(Event::Update) {
            cx.resources.TIMER.stop();
            cx.resources.TIMER.clear_events();
            for magnet in cx.resources.MAGNETS.iter_mut() {
                magnet.set_low().ok(); // continue shutting off magnets even if one `set_low` call fails
            }
        }
    }


    // TODO: start/reset kill_magnets timer
    // TODO: write a macro for this... and call it in a loop :D
    // #[task(binds = EXTI0,  priority = 4, resources = [SENSORS, MAGNETS])]
    // fn exti0_handler(mut cx: exti0_handler::Context) {
    //     cx.resources.MAGNETS.lock(|MAGNETS| { sensor_handler(SENSOR_LUT[0], MAGNETS) });
    //     cx.resources.SENSORS[SENSOR_LUT[0]].clear_interrupt();
    // }
    #[task(binds = EXTI0, priority = 4, resources = [SENSORS, MAGNETS, TIMER])]
    fn exti0_handler(mut cx: exti0_handler::Context) {
        let mut timer = cx.resources.TIMER;
        cx.resources.MAGNETS.lock(|MAGNETS| {
            timer.lock(|timer| { sensor_handler(SENSOR_LUT[0], MAGNETS, timer) });
        });
        cx.resources.SENSORS[SENSOR_LUT[0]].clear_interrupt();
    }
    #[task(binds = EXTI1, priority = 4, resources = [SENSORS, MAGNETS, TIMER])]
    fn exti1_handler(mut cx: exti1_handler::Context) {
        let mut timer = cx.resources.TIMER;
        cx.resources.MAGNETS.lock(|MAGNETS| {
            timer.lock(|timer| { sensor_handler(SENSOR_LUT[1], MAGNETS, timer) });
        });
        cx.resources.SENSORS[SENSOR_LUT[1]].clear_interrupt();
    }
    #[task(binds = EXTI2_TSC, priority = 4, resources = [SENSORS, MAGNETS, TIMER])]
    fn exti2_handler(mut cx: exti2_handler::Context) {
        let mut timer = cx.resources.TIMER;
        cx.resources.MAGNETS.lock(|MAGNETS| {
            timer.lock(|timer| { sensor_handler(SENSOR_LUT[2], MAGNETS, timer) });
        });
        cx.resources.SENSORS[SENSOR_LUT[2]].clear_interrupt();
    }
    #[task(binds = EXTI3, priority = 4, resources = [SENSORS, MAGNETS, TIMER])]
    fn exti3_handler(mut cx: exti3_handler::Context) {
        let mut timer = cx.resources.TIMER;
        cx.resources.MAGNETS.lock(|MAGNETS| {
            timer.lock(|timer| { sensor_handler(SENSOR_LUT[3], MAGNETS, timer) });
        });
        cx.resources.SENSORS[SENSOR_LUT[3]].clear_interrupt();
    }
    #[task(binds = EXTI4, priority = 4, resources = [SENSORS, MAGNETS, TIMER])]
    fn exti4_handler(mut cx: exti4_handler::Context) {
        let mut timer = cx.resources.TIMER;
        cx.resources.MAGNETS.lock(|MAGNETS| {
            timer.lock(|timer| { sensor_handler(SENSOR_LUT[4], MAGNETS, timer) });
        });
        cx.resources.SENSORS[SENSOR_LUT[4]].clear_interrupt();
    }
    #[task(binds = EXTI9_5, priority = 4, resources = [SENSORS, MAGNETS, TIMER])]
    // 5 doesn't work
    //
    fn exti9_5_handler(mut cx: exti9_5_handler::Context) {
        let mut timer = cx.resources.TIMER;
        for i in 5..(9+1) {
            if cx.resources.SENSORS[SENSOR_LUT[i]].is_interrupt_pending() {
                cx.resources.MAGNETS.lock(|MAGNETS| {
                    timer.lock(|timer| { sensor_handler(SENSOR_LUT[i], MAGNETS, timer) });
                });
                cx.resources.SENSORS[SENSOR_LUT[i]].clear_interrupt();
            }
        }
    }

    #[task(binds = EXTI15_10, priority = 4, resources = [SENSORS, MAGNETS, TIMER])]
    fn exti15_10_handler(mut cx: exti15_10_handler::Context) {
        let mut timer = cx.resources.TIMER;
        for i in 10..(15+1) {
            if cx.resources.SENSORS[SENSOR_LUT[i]].is_interrupt_pending() {
                cx.resources.MAGNETS.lock(|MAGNETS| {
                    timer.lock(|timer| { sensor_handler(SENSOR_LUT[i], MAGNETS, timer) });
                });
                cx.resources.SENSORS[SENSOR_LUT[i]].clear_interrupt();
            }
        }
    }

    // Move received byte into serial receive queue
    // TODO: remove panicing once serial communications are reliable enough
    #[task(binds = USART1_EXTI25, spawn = [rx_task], priority = 5, resources = [RX, RX_P])]
    fn usart1(cx: usart1::Context) {
        match cx.resources.RX.read() {
            Ok(c) => {
                cx.resources.RX_P.enqueue(c).expect("serial rx queue overflow");
                // TODO: remove??
                // cx.spawn.rx_task().unwrap();
            }
            Err(e) => {
                match e {
                    nb::Error::WouldBlock => (), // no data available
                    nb::Error::Other(stm32f3xx_hal::serial::Error::Overrun) => panic!("rx buffer overrun"),
                    _ => panic!("{:?}", e),
                }
            }
        };
    }

    #[task(schedule = [rx_task], priority = 4, resources = [RX_C, TX_P])]
    fn rx_task(cx: rx_task::Context) {
        static mut STATE: RxState = RxState::Sync1;
        static mut PACKET: Packet = Packet::new();
        static mut PAYLOAD_BUF: [u8; 255] = [0u8; 255];

        

        if let Some(byte) = cx.resources.RX_C.dequeue() {

            // cx.resources.TX_P.enqueue(Packet::from(3, Some(b"0")));

            // packet receive STATE machine
            match STATE {
                RxState::Sync1 => {
                    match byte {
                        0xAA => *STATE = RxState::Sync2,
                        _ => *STATE = RxState::Sync1,
                    }
                },
                RxState::Sync2 => {
                    match byte {
                        0x55 => *STATE = RxState::Command,
                        _ => *STATE = RxState::Sync1,
                    }
                },
                RxState::Command => {
                    *STATE = RxState::Length;

                    PACKET.command = byte;
                },
                RxState::Length => {
                    match byte {
                        0 => {
                            *STATE = RxState::Crc1;
                        },
                        _ => {
                            *STATE = RxState::Payload(byte-1);
                            PACKET.payload = None;
                        },
                    }

                    PACKET.length = byte;
                },
                RxState::Payload(i) => {
                    // FIXME: something strange going on here with borrowing
                    // TODO: come up with something simpler
                    let i: u8 = *i; 
                    match i {
                        0 => *STATE = RxState::Crc1,
                        _ => *STATE = RxState::Payload(i-1),
                    }

                    PAYLOAD_BUF[(PACKET.length - 1 - i) as usize] = byte;
                    if i == 0 {
                        PACKET.payload = Some(*PAYLOAD_BUF);
                    }
                },
                RxState::Crc1 => {
                    *STATE = RxState::Crc2;

                    PACKET.checksum = 0u16;
                    PACKET.checksum |= byte as u16;
                },
                RxState::Crc2 => {
                    *STATE = RxState::Sync1;

                    PACKET.checksum |= (byte as u16) << 8;

                    // TODO: remove panicing
                    *PACKET = PACKET.validate().expect("invalid checksum");

                    cx.resources.TX_P.enqueue(*PACKET).expect("could not push to tx channel");
                }
            }
        }

        // bytes arriving at 14_400 Hz
        cx.schedule.rx_task(cx.scheduled + 5_000.cycles()).unwrap();
    }

    // unused interrupts used to schedule software tasks
    extern "C" {
        fn USB_HP();
        fn USB_LP();
        fn USB_WKUP_EXTI();
        fn TIM20_BRK();
        fn TIM20_UP();
        fn TIM20_TRG_COM();
        fn TIM20_CC();
        fn SPI4();
        fn I2C3_ER();
        fn I2C3_EV();
    }
};
