#![deny(unsafe_code)]
#![no_main]
#![no_std]

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

use rtic::app;
use stm32f3xx_hal::{
    gpio::{Output, PXx, PushPull},
    pac::{self, TIM2},
    prelude::*,
    timer::Timer,
};

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
    timer.start(500.milliseconds());
}

#[app(device = stm32f3xx_hal::pac, peripherals = true, dispatchers = [USB_HP, USB_LP, USB_WKUP_EXTI, TIM20_BRK, TIM20_UP, TIM20_TRG_COM, TIM20_CC, SPI4, I2C3_ER, I2C3_EV])]
mod app {

    use cortex_m_semihosting::hprintln;
    use nb::block;
    use panic_semihosting as _;

    use rtic::Mutex;
    use stm32f3xx_hal::pac;
    use stm32f3xx_hal::pac::USART1;
    use stm32f3xx_hal::time::duration::*;
    use stm32f3xx_hal::time::rate::*;
    use stm32f3xx_hal::{
        gpio::{Edge, Input, Output, PXx, PushPull, AF7, PA10, PA9},
        prelude::*,
        serial::{self, Rx, Serial, Tx},
        timer::{Event, Timer},
        Toggle,
    };

    use systick_monotonic::*;

    // Inter Process Communication (IPC) types
    use heapless::spsc::{Consumer, Producer, Queue};

    // interrupts
    use pac::{Interrupt, TIM2};

    use super::RxState;
    use kapine_packet::Packet;

    const RX_QUEUE_LEN: usize = 512;
    const TX_QUEUE_LEN: usize = 4;
    const SENSOR_LUT: [usize; 16] = [13, 9, 11, 2, 0, 12, 15, 5, 14, 4, 8, 3, 1, 7, 10, 6];

    #[shared]
    struct Shared {
        rx_c: Consumer<'static, u8, RX_QUEUE_LEN>,
        tx_p: Producer<'static, Packet, TX_QUEUE_LEN>,
        tx_c: Consumer<'static, Packet, TX_QUEUE_LEN>,
        sensors: [PXx<Input>; 16],
        magnets: [PXx<Output<PushPull>>; 16],
        timer: Timer<TIM2>,
        serial: Serial<USART1, (PA9<AF7<PushPull>>, PA10<AF7<PushPull>>)>,
    }

    #[local]
    struct Local {
        rx_state: RxState,
        rx_buf: Packet,
        rx_p: Producer<'static, u8, RX_QUEUE_LEN>,
        debug_leds: [PXx<Output<PushPull>>; 2],
        payload_buf: [u8; 255],
        packet: Packet,
        local_cnt: u8,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[init(local = [
        Q: Queue<u8, RX_QUEUE_LEN> = Queue::new(),
        TX_Q: Queue<Packet, TX_QUEUE_LEN> = Queue::new()
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        hprintln!("init").unwrap();

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
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();

        let systick = core.SYST;
        let mono = Systick::new(systick, 72_000_000);

        // Acquire GPIO peripherals
        let mut gpio_a = device.GPIOA.split(&mut rcc.ahb);
        let mut gpio_b = device.GPIOB.split(&mut rcc.ahb);
        let mut gpio_c = device.GPIOC.split(&mut rcc.ahb);
        let mut gpio_d = device.GPIOD.split(&mut rcc.ahb);
        let mut _gpio_e = device.GPIOE.split(&mut rcc.ahb);
        let mut _gpio_f = device.GPIOF.split(&mut rcc.ahb);
        let mut _gpio_g = device.GPIOG.split(&mut rcc.ahb);
        let mut _gpio_h = device.GPIOH.split(&mut rcc.ahb);

        let uart_pins = (
            gpio_a
                .pa9
                .into_af_push_pull(&mut gpio_a.moder, &mut gpio_a.otyper, &mut gpio_a.afrh),
            gpio_a
                .pa10
                .into_af_push_pull(&mut gpio_a.moder, &mut gpio_a.otyper, &mut gpio_a.afrh),
        );
        let mut serial = Serial::new(
            device.USART1,
            uart_pins,
            115_200.Bd(),
            clocks,
            &mut rcc.apb2,
        );
        // serial.listen(stm32f3xx_hal::serial::Event::Rxne);
        serial.configure_interrupt(serial::Event::ReceiveDataRegisterNotEmpty, Toggle::On);

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

        // photo-gate sensor inputs
        // enable EXTIx for x in {0, 1, 2, 4, 6, 7, 8, 9, 10, 11, 12, 13, 15}
        // required 3, 5, 14
        let mut sensors: [PXx<Input>; 16] = [
            gpio_b
                .pb4
                .into_pull_up_input(&mut gpio_b.moder, &mut gpio_b.pupdr)
                .downgrade()
                .downgrade(),
            gpio_c
                .pc12
                .into_pull_up_input(&mut gpio_c.moder, &mut gpio_c.pupdr)
                .downgrade()
                .downgrade(),
            gpio_a
                .pa3
                .into_pull_up_input(&mut gpio_a.moder, &mut gpio_a.pupdr)
                .downgrade()
                .downgrade(), // s2 (bodge)
            gpio_a
                .pa11
                .into_pull_up_input(&mut gpio_a.moder, &mut gpio_a.pupdr)
                .downgrade()
                .downgrade(),
            gpio_c
                .pc9
                .into_pull_up_input(&mut gpio_c.moder, &mut gpio_c.pupdr)
                .downgrade()
                .downgrade(),
            gpio_c
                .pc7
                .into_pull_up_input(&mut gpio_c.moder, &mut gpio_c.pupdr)
                .downgrade()
                .downgrade(),
            gpio_b
                .pb15
                .into_pull_up_input(&mut gpio_b.moder, &mut gpio_b.pupdr)
                .downgrade()
                .downgrade(),
            gpio_b
                .pb13
                .into_pull_up_input(&mut gpio_b.moder, &mut gpio_b.pupdr)
                .downgrade()
                .downgrade(),
            gpio_b
                .pb10
                .into_pull_up_input(&mut gpio_b.moder, &mut gpio_b.pupdr)
                .downgrade()
                .downgrade(),
            gpio_b
                .pb1
                .into_pull_up_input(&mut gpio_b.moder, &mut gpio_b.pupdr)
                .downgrade()
                .downgrade(),
            gpio_c
                .pc14
                .into_pull_up_input(&mut gpio_c.moder, &mut gpio_c.pupdr)
                .downgrade()
                .downgrade(), // s10 (bodge)
            gpio_a
                .pa2
                .into_pull_up_input(&mut gpio_a.moder, &mut gpio_a.pupdr)
                .downgrade()
                .downgrade(),
            gpio_a
                .pa5
                .into_pull_up_input(&mut gpio_a.moder, &mut gpio_a.pupdr)
                .downgrade()
                .downgrade(), // s12 (bodge)
            gpio_c
                .pc0
                .into_pull_up_input(&mut gpio_c.moder, &mut gpio_c.pupdr)
                .downgrade()
                .downgrade(),
            gpio_b
                .pb8
                .into_pull_up_input(&mut gpio_b.moder, &mut gpio_b.pupdr)
                .downgrade()
                .downgrade(),
            gpio_b
                .pb6
                .into_pull_up_input(&mut gpio_b.moder, &mut gpio_b.pupdr)
                .downgrade()
                .downgrade(),
        ];

        // enable sensor interrupts
        for sensor in &mut sensors {
            sensor.trigger_on_edge(&mut exti, Edge::Falling);
            syscfg.select_exti_interrupt_source(sensor);
            sensor.enable_interrupt(&mut exti);
        }

        //let (tx, rx) = serial.split();

        rtic::pend(Interrupt::USART1_EXTI25);

        // init empty receive Packet structu
        let payload = [0u8; 255];
        let packet = Packet::from(0u8, Some(&payload));

        let Q: &'static mut Queue<u8, RX_QUEUE_LEN> = cx.local.Q;
        let TX_Q: &'static mut Queue<Packet, TX_QUEUE_LEN> = cx.local.TX_Q;
        let (rx_p, rx_c) = Q.split();
        let (tx_p, tx_c) = TX_Q.split();

        // 2Hz
        let mut timer = Timer::new(device.TIM2, clocks, &mut rcc.apb1);
        timer.enable_interrupt(Event::Update);
        timer.stop();

        // spawn software tasks
        debug_leds[1].set_high().ok();
        //tx_task::spawn().unwrap();
        rx_task::spawn().unwrap();
        command_task::spawn().unwrap();

        // best task
        // moi_task::spawn().unwrap();
        packet_cat_task::spawn().unwrap();

        (
            Shared {
                rx_c,
                tx_c,
                tx_p,
                sensors,
                magnets,
                timer,
                serial,
            },
            Local {
                rx_state: RxState::Sync1,
                rx_buf: packet,
                rx_p,
                debug_leds,
                payload_buf: [0u8; 255],
                packet: Packet::new(),
                local_cnt: 0u8,
            },
            init::Monotonics(mono),
        )
    }

    #[task(priority = 2, shared = [tx_p])]
    fn moi_task(mut cx: moi_task::Context) {
        cx.shared.tx_p.lock(|tx_p| {
            tx_p.enqueue(Packet::from(3, Some(b"jee :D"))).unwrap();
        });
    }

    #[task(priority = 3, local = [debug_leds])]
    fn blink_task(cx: blink_task::Context) {
        for led in cx.local.debug_leds.iter_mut() {
            led.toggle().ok();
        }
        blink_task::spawn_after(1.secs()).unwrap();
    }

    // shut all magnets of immediately
    #[task(binds = TIM2, priority = 6, shared = [timer, magnets])]
    fn kill_magnets(cx: kill_magnets::Context) {
        let timer = cx.shared.timer;
        let magnets = cx.shared.magnets;
        (timer, magnets).lock(|timer, magnets| {
            if timer.is_event_triggered(Event::Update) {
                timer.stop();
                timer.clear_events();
                for magnet in magnets.iter_mut() {
                    magnet.set_low().ok(); // continue shutting off magnets even if one `set_low` call fails
                }
            }
        })
    }

    // TODO: start/reset kill_magnets timer
    // TODO: write a macro for this... and call it in a loop :D
    #[task(binds = EXTI0, priority = 4, shared = [sensors, magnets, timer])]
    fn exti0_handler(cx: exti0_handler::Context) {
        let timer = cx.shared.timer;
        let sensors = cx.shared.sensors;
        let magnets = cx.shared.magnets;
        (timer, sensors, magnets).lock(|timer, sensors, magnets| {
            super::sensor_handler(SENSOR_LUT[0], magnets, timer);
            sensors[SENSOR_LUT[0]].clear_interrupt();
        });
    }
    #[task(binds = EXTI1, priority = 4, shared = [sensors, magnets, timer])]
    fn exti1_handler(cx: exti1_handler::Context) {
        let timer = cx.shared.timer;
        let sensors = cx.shared.sensors;
        let magnets = cx.shared.magnets;
        (timer, sensors, magnets).lock(|timer, sensors, magnets| {
            super::sensor_handler(SENSOR_LUT[0], magnets, timer);
            sensors[SENSOR_LUT[1]].clear_interrupt();
        });
    }
    #[task(binds = EXTI2_TSC, priority = 4, shared = [sensors, magnets, timer])]
    fn exti2_handler(cx: exti2_handler::Context) {
        let timer = cx.shared.timer;
        let sensors = cx.shared.sensors;
        let magnets = cx.shared.magnets;
        (timer, sensors, magnets).lock(|timer, sensors, magnets| {
            super::sensor_handler(SENSOR_LUT[0], magnets, timer);
            sensors[SENSOR_LUT[2]].clear_interrupt();
        });
    }
    #[task(binds = EXTI3, priority = 4, shared = [sensors, magnets, timer])]
    fn exti3_handler(cx: exti3_handler::Context) {
        let timer = cx.shared.timer;
        let sensors = cx.shared.sensors;
        let magnets = cx.shared.magnets;
        (timer, sensors, magnets).lock(|timer, sensors, magnets| {
            super::sensor_handler(SENSOR_LUT[0], magnets, timer);
            sensors[SENSOR_LUT[3]].clear_interrupt();
        });
    }
    #[task(binds = EXTI4, priority = 4, shared = [sensors, magnets, timer])]
    fn exti4_handler(cx: exti4_handler::Context) {
        let timer = cx.shared.timer;
        let sensors = cx.shared.sensors;
        let magnets = cx.shared.magnets;
        (timer, sensors, magnets).lock(|timer, sensors, magnets| {
            super::sensor_handler(SENSOR_LUT[0], magnets, timer);
            sensors[SENSOR_LUT[4]].clear_interrupt();
        });
    }
    // 5 doesn't work
    #[task(binds = EXTI9_5, priority = 4, shared = [sensors, magnets, timer])]
    fn exti9_5_handler(cx: exti9_5_handler::Context) {
        let timer = cx.shared.timer;
        let sensors = cx.shared.sensors;
        let magnets = cx.shared.magnets;
        (timer, sensors, magnets).lock(|timer, sensors, magnets| {
            for i in 5..(9 + 1) {
                if sensors[SENSOR_LUT[i]].is_interrupt_pending() {
                    super::sensor_handler(SENSOR_LUT[i], magnets, timer);
                    sensors[SENSOR_LUT[i]].clear_interrupt();
                }
            }
        });
    }

    #[task(binds = EXTI15_10, priority = 4, shared = [sensors, magnets, timer])]
    fn exti15_10_handler(cx: exti15_10_handler::Context) {
        let timer = cx.shared.timer;
        let sensors = cx.shared.sensors;
        let magnets = cx.shared.magnets;
        (timer, sensors, magnets).lock(|timer, sensors, magnets| {
            for i in 10..(15 + 1) {
                if sensors[SENSOR_LUT[i]].is_interrupt_pending() {
                    super::sensor_handler(SENSOR_LUT[i], magnets, timer);
                    sensors[SENSOR_LUT[i]].clear_interrupt();
                }
            }
        });
    }

    // Move received byte into serial receive queue
    // TODO: remove panicing once serial communications are reliable enough
    #[task(binds = USART1_EXTI25, priority = 8, local = [rx_p], shared = [serial])]
    fn uart_handler(mut cx: uart_handler::Context) {
        let rx_p = cx.local.rx_p;

        cx.shared.serial.lock(|serial| {
            if serial.is_event_triggered(serial::Event::ReceiveDataRegisterNotEmpty) {
                serial.configure_interrupt(serial::Event::ReceiveDataRegisterNotEmpty, Toggle::Off);
                match serial.read() {
                    Ok(c) => {
                        rx_p.enqueue(c).expect("serial rx queue overflow");
                        serial.configure_interrupt(
                            serial::Event::ReceiveDataRegisterNotEmpty,
                            Toggle::On,
                        );
                        //blink_task::spawn().unwrap();
                        //rx_task::spawn().unwrap();
                    }
                    Err(e) => {
                        match e {
                            nb::Error::WouldBlock => (), // no data available
                            nb::Error::Other(stm32f3xx_hal::serial::Error::Overrun) => {
                                panic!("rx buffer overrun")
                            }
                            _ => panic!("{:?}", e),
                        }
                    }
                };
            }
        })
    }

    #[task(priority = 4, shared = [rx_c, tx_c])]
    fn packet_cat_task(mut cx: packet_cat_task::Context) {
        // TODO: make tx_buf length a constant in kapine-packet
        hprintln!("packet_cat_task").unwrap();
        let mut tx_buf = [0u8; 261];
        cx.shared.tx_c.lock(|tx_c| {
            if let Some(packet) = tx_c.dequeue() {
                hprintln!("{:?}", packet);
                // let length: usize = packet.write_bytes(&mut tx_buf[..]) as usize;
                // hprintln!("{:?}", tx_buf);
            }
        });
        // cx.shared.rx_c.lock(|rx_c| {
        //     if let Some(char) = rx_c.dequeue() {
        //         hprintln!("{}", char).unwrap();
        //     }
        // });

        packet_cat_task::spawn_after(1.secs()).unwrap();
    }

    // FIXME: increase rx_c buffer size and run at some sensible frequency e.g. 1kHz
    #[task(priority = 7, shared = [rx_c, tx_p], local = [rx_state, local_cnt, packet, payload_buf])]
    fn rx_task(cx: rx_task::Context) {

        // hprintln!("rx_task {:?}", cx.local.rx_state).unwrap();
        // let local_cnt = cx.local.local_cnt;
        // hprintln!("local_cnt {}", local_cnt);
        // *local_cnt += 1;

        let mut rx_consumer = cx.shared.rx_c;
        let tx_producer = cx.shared.tx_p;
        let rx_state = cx.local.rx_state;
        let packet = cx.local.packet;
        let payload_buf = cx.local.payload_buf;
        (rx_consumer, tx_producer).lock(|rx_consumer, tx_producer| {
            if let Some(byte) = rx_consumer.dequeue() {
                // hprintln!("0x{:02x}", byte).unwrap();
                // cx.shared.tx_p.enqueue(Packet::from(3, Some(b"0")));

                // packet receive STATE machine
                match rx_state {
                    RxState::Sync1 => match byte {
                        0xAA => *rx_state = RxState::Sync2,
                        _ => *rx_state = RxState::Sync1,
                    },
                    RxState::Sync2 => match byte {
                        0x55 => *rx_state = RxState::Command,
                        _ => *rx_state = RxState::Sync1,
                    },
                    RxState::Command => {
                        *rx_state = RxState::Length;

                        packet.command = byte;
                    }
                    RxState::Length => {
                        match byte {
                            0 => {
                                *rx_state = RxState::Crc1;
                            }
                            _ => {
                                *rx_state = RxState::Payload(byte - 1);
                                packet.payload = None;
                            }
                        }

                        packet.length = byte;
                    }
                    RxState::Payload(i) => {
                        // FIXME: indexing error on packet deser, tries to index 255 with array length 255
                        // FIXME: something strange going on here with borrowing
                        // TODO: come up with something simpler
                        let i: u8 = *i;
                        match i {
                            0 => *rx_state = RxState::Crc1,
                            _ => *rx_state = RxState::Payload(i - 1),
                        }

                        if i > 0 {
                            payload_buf[(packet.length - 1 - i) as usize] = byte;
                        } else {
                            packet.payload = Some(*payload_buf);
                        }
                    }
                    RxState::Crc1 => {
                        *rx_state = RxState::Crc2;

                        packet.checksum = 0u16;
                        packet.checksum |= byte as u16;
                    }
                    RxState::Crc2 => {
                        *rx_state = RxState::Sync1;

                        // packet.checksum |= (byte as u16) << 8;
                        // TODO: compute checksum

                        // TODO: remove panicing
                        //packet.validate().expect("invalid checksum");
                        //hprintln!("packet: {:?}", packet).unwrap();

                        tx_producer
                            .enqueue(*packet)
                            .expect("could not push to tx channel");
                    }
                }
            }
        });

        // bytes arriving at 14_400 Hz
        rx_task::spawn_after(5000.micros()).unwrap();
    }

    // reads a packet from the tx queue and transmits it over serial
    #[task(priority = 4, shared = [serial, tx_c])]
    fn tx_task(mut cx: tx_task::Context) {
        // TODO: make tx_buf length a constant in kapine-packet
        let mut tx_buf = [0u8; 261];

        let serial = cx.shared.serial;
        let tx_c = cx.shared.tx_c;
        (serial, tx_c).lock(|serial, tx_c| {
            if let Some(packet) = tx_c.dequeue() {
                let length: usize = packet.write_bytes(&mut tx_buf[..]) as usize;

                for byte in tx_buf[..length].iter() {
                    block!(serial.write(*byte)).unwrap();
                    block!(serial.flush()).unwrap();
                }
            }
        });

        tx_task::spawn_after(1.millis()).unwrap();
    }

    // reads a packet from the tx queue and transmits it over serial
    #[task(priority = 4, shared = [tx_c, magnets, timer])]
    fn command_task(mut cx: command_task::Context) {
        let tx_c = cx.shared.tx_c;
        let magnets = cx.shared.magnets;
        let timer = cx.shared.timer;
        (tx_c, magnets, timer).lock(|tx_c, magnets, timer| {
            if let Some(packet) = tx_c.dequeue() {
                // TODO: enumerate commands somewhere
                match packet.command {
                    4 => {
                        magnets[packet.payload.unwrap()[0] as usize].set_high().unwrap();
                        timer.start(500.milliseconds());
                    }
                    _ => unimplemented!()
                }
            }
        });

        command_task::spawn_after(1.millis()).unwrap();
    }
}
