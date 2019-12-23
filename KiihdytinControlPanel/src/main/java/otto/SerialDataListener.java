package otto;

import com.fazecast.jSerialComm.*;
import java.util.Arrays;

public class SerialDataListener implements SerialPortDataListener {

    public SerialDataListener() {
        //super();
    }

    @Override
    public int getListeningEvents() {
        return SerialPort.LISTENING_EVENT_DATA_AVAILABLE;
    }

    @Override
    public void serialEvent(SerialPortEvent event) {
        if (event.getEventType() != SerialPort.LISTENING_EVENT_DATA_AVAILABLE) {
            return;
        }

        byte[] newData = new byte[event.getSerialPort().bytesAvailable()];
        int receivedBytes = event.getSerialPort().readBytes(
                newData, event.getSerialPort().bytesAvailable());

        DataPacket packet = DataPacket.fromBytes(newData);
        // TODO: do something with the packet
        // if packet == null, there was an error decoding the byte array into a DataPacket

        System.out.println("Received this: " + Arrays.toString(newData));
    }

}
