mod packet {

    struct Packet {
        sync: u16,
        command: u8,
        length: u8,
        payload: [u8; 255],
        checksum: u16
    }

    impl Packet {

        fn as_bytes() -> {
            
        }

        fn new (command: u8, payload: &[u8]) -> Self {
            Packet {
                sync: 0x55AA,
                command: command,
                length: payload.len(),
                payload: payload,
                checksum:
            }
        }
    }

}
