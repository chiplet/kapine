package otto;

import com.fazecast.jSerialComm.*;
import java.util.ArrayList;
import java.util.Arrays;

public class SerialInterface {

    private SerialPort port;
    private SerialDataListener listener;

    public void SerialInterface() {
    }

    public ArrayList<String> availablePorts() {
        ArrayList<String> portNames = new ArrayList<>();

        for (SerialPort port : SerialPort.getCommPorts()) {
            portNames.add(port.getSystemPortName());
        }

        return portNames;
    }

    public boolean connect(String portName) {
        this.port = SerialPort.getCommPort(portName);
        boolean successful = this.port.openPort();
        if (successful) {
            this.port.setComPortParameters(9600, 8, 1, 0);
            this.listener = new SerialDataListener();
            this.port.addDataListener(this.listener);
        }
        return successful;
    }

    public boolean disconnect() {
        return this.port.closePort();
    }

    public long testLatency() {
        if (this.port == null) {
            return -1;
        }

        long start = System.currentTimeMillis();

        byte[] testData = {25, 30};

        int bytesWritten = this.port.writeBytes(testData, testData.length);
        System.out.println("Successfully sent " + bytesWritten + " bytes:" + Arrays.toString(testData));

        while (this.port.bytesAvailable() == 0) {
            if (System.currentTimeMillis() - start > 1000) {
                System.out.println("Timed out waiting for data to be echoed back");
                return -1;
            }
        }

        byte[] readBuffer = new byte[this.port.bytesAvailable()];
        int numRead = this.port.readBytes(readBuffer, readBuffer.length);
        System.out.println("Received " + numRead + " bytes:");
        for (byte b : readBuffer) {
            System.out.println("Received " + b);
        }

        long end = System.currentTimeMillis();

        return end - start;
    }

    public boolean send(byte[] message) {
        return false;
    }

}
