package otto;

import com.fazecast.jSerialComm.*;
import java.util.Arrays;
import java.util.ArrayList;

public class SerialDataListener implements SerialPortDataListener {

    private ArrayList<Byte> receivedByteBuffer;

    public SerialDataListener() {
        //super();
        this.receivedByteBuffer = new ArrayList<>();
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
        for (byte b : newData) {
            this.receivedByteBuffer.add(b);
        }

        System.out.println("Received this: " + Arrays.toString(newData));
    }

}
