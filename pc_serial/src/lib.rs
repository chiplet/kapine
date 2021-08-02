mod packet {

    #[derive(Debug)]
    struct Packet {
        sync: u16,
        command: u8,
        length: u8,
        payload: [u8; 255],
        checksum: u16
    }

    impl Packet {
        // TODO: come up with a better implementation
        fn write_bytes(&self, buffer: &mut [u8]) -> u32 {

            let mut i = 0;
            buffer[i] = self.sync as u8; i = i + 1;
            buffer[i] = (self.sync >> 8) as u8; i = i + 1;

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
    #[test]
    fn sample_test() {
        assert_eq!(2 + 2, 4);
    }
}
