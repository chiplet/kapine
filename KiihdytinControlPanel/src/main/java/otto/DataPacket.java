package otto;

import java.io.InputStream;
import java.io.ByteArrayInputStream;
import java.io.DataInputStream;
import java.io.IOException;
import java.util.Arrays;

public class DataPacket {

    private short sync;
    private byte commandCode;
    private byte payloadLength;
    private byte[] payload;
    private short checksum;

    public DataPacket(short sync, byte commandCode, byte payloadLength, byte[] payload, short checksum) {
        this.sync = sync;
        this.commandCode = commandCode;
        this.payloadLength = payloadLength;
        this.payload = payload;
        this.checksum = checksum;
    }

    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("Sync:");
        int a = this.sync;
        sb.append(Integer.toHexString(a));
        sb.append("Command code:");
        sb.append(this.commandCode);
        sb.append("Payload length:");
        sb.append(this.payloadLength);
        sb.append("Payload:");
        String payload = new String(this.payload);
        sb.append(payload);
        sb.append("Checksum:");
        int b = this.checksum;
        sb.append(Integer.toHexString(b));
        return sb.toString();
    }

    public static DataPacket fromBytes(byte[] packet) {
        InputStream packetByteStream = new ByteArrayInputStream(packet);
        DataInputStream dataInputStream = new DataInputStream(packetByteStream);

        short sync = -1;
        byte commandCode = -1;
        byte payloadLength = -1;
        byte[] payload = null;
        short checksum = -1;

        try {
            sync = dataInputStream.readShort();
            commandCode = dataInputStream.readByte();
            payloadLength = dataInputStream.readByte();
            payload = new byte[payloadLength];
            dataInputStream.read(payload, 0, payloadLength);
            checksum = dataInputStream.readShort();
        } catch (IOException e) {
            return null;
        }

        return new DataPacket(sync, commandCode, payloadLength, payload, checksum);
    }
}

