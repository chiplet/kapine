use std::io;
use std::time::Duration;
use kapine_packet::Packet;

fn main() {

    println!("Hello, world! We are about to do some serious serial communication ·D");

    let mut device_counter = 0;
    let ports = serialport::available_ports().expect("No ports found!");
    for p in &ports {
        println!("{}: {}", device_counter, p.port_name);
        device_counter = device_counter + 1;
    }

    println!("Choose a port by writing the corresponding number:");

    let mut number = String::new();
    io::stdin().read_line(&mut number).expect("Unable to read user input.");
    let port_index: usize = number.trim().parse().expect("Index entered was not a number.");

    let port_name = &ports[port_index].port_name;
    let baud_rate = 9_600;
    println!("The chosen port is {}", port_name);
    let mut port = serialport::new(port_name, baud_rate)
        .timeout(Duration::from_millis(10))
        .open();


    match port {
        Ok(mut port) => {

            // writing a message to the port
            let payload = [1, 2, 3];
            let message = Packet::new(5, Some(&payload));
            const BUF_SIZE: usize = 261;
            let mut buffer: [u8; BUF_SIZE] = [0; BUF_SIZE];
            let length = message.write_bytes(&mut buffer) as usize;

            println!("length = {}", length);

            println!("Wrote bytes: {:?}", &buffer[..length]);
            //let output = "Testiviesti".as_bytes();
            port.write(&buffer[..length]).expect("Write failed!");

            let mut serial_buf: Vec<u8> = vec![0; 1000];
            println!("Receiving data on {} at {} baud:", &port_name, &baud_rate);
            loop {
                match port.read(serial_buf.as_mut_slice()) {
                    Ok(t) => {
                        let message = String::from_utf8((&serial_buf[..t]).to_vec()).expect("Could not convert to string");
                        println!("{}", message);
                    },
                    Err(ref e) if e.kind() == io::ErrorKind::TimedOut => (),
                    Err(e) => eprintln!("{:?}", e),
                }
            }
        }
        Err(e) => {
            eprintln!("Failed to open \"{}\". Error: {}", port_name, e);
            ::std::process::exit(1);
        }
    }

}
