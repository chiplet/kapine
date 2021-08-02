pub mod packet {

    #[derive(Debug)]
    pub struct Packet {
        pub sync: u16,
        pub command: u8,
        pub length: u8,
        pub payload: [u8; 255],
        pub checksum: u16
    }

    impl Packet {
        // TODO: come up with a better implementation
        pub fn write_bytes(&self, buffer: &mut [u8]) -> u32 {

            let mut i = 0;
            buffer[i] = self.sync as u8; i = i + 1;
            buffer[i] = (self.sync >> 8) as u8; i = i + 1;
            buffer[i] = self.command; i = i + 1;
            buffer[i] = self.length; i = i + 1;

            let mut j = 0;
            let mut byte = self.payload[j];
            while byte != 0x00 {
                buffer[i] = self.payload[j];
                i = i + 1;
                j = j + 1;
                byte = self.payload[j]
            }

            buffer[i] = self.checksum as u8; i = i + 1;
            buffer[i] = (self.checksum >> 8) as u8;

            (self.length + 6).into()
        }

        fn compute_checksum(&mut self) {
            unimplemented!("TODO: Implement checksum computation");
        }

        fn new(command: u8, payload: [u8; 255]) -> Self {
            let payload_length = payload.len();
            assert!(payload_length <= 255);

            let mut packet = Packet {
                sync: 0x55AA,
                command: command,
                length: payload_length as u8,
                payload: payload,
                checksum: 0u16, // populated by compute_checksum
            };
            packet.compute_checksum();
            packet
        }
    }

}

#[cfg(test)]
mod tests {
    use super::packet;

    #[test]
    fn sample_test() {
        assert_eq!(2 + 2, 4);
    }

    #[test]
    fn packet_test() {

        const BUF_SIZE: usize = 261;
        let mut buffer: [u8; BUF_SIZE] = [0; BUF_SIZE];

        let packet: [u8; 7] = [0xAA, 0x55, 0x01, 0x03, 0x12, 0x34, 0x56];

        let mut payload = [0; 255];
        payload[0] = 1;
        payload[1] = 2;
        payload[2] = 3;
        let packet_struct = packet::Packet {
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
}
