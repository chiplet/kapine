use std::io;
use std::time::Duration;

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
    println!("The chosen port is {}", port_name);
    let mut port = serialport::new(port_name, 9_600)
        .timeout(Duration::from_millis(10))
        .open().expect("Failed to open port");

    println!("Connected to port {}", port_name);

    let output = "Testiviesti".as_bytes();
    port.write(output).expect("Write failed!");
}
