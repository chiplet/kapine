pub fn main() {

    const BUF_SIZE: usize = 261;
    let mut buffer: [u8; BUF_SIZE] = [0; BUF_SIZE];

    let packet: [u8; 7] = [0xAA, 0x55, 0x01, 0x03, 0x12, 0x34, 0x56];

    let mut payload = [0; 255];
    payload[0] = 1;
    payload[1] = 2;
    payload[2] = 3;

    let packet_struct = pc_serial::packet::Packet {
        sync: 0x55AA,
        command: 0x01,
        length: 3,
        payload: payload,
        checksum: 0u16,
    };

    println!("{:?}\n", packet_struct);

    let len = packet_struct.write_bytes(&mut buffer);

    println!("bytes: {:?}", buffer);
    println!("length: {:?}", len);


}
