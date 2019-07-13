/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package otto.kiihdytincontrolpanel;

import com.fazecast.jSerialComm.*;
import java.util.ArrayList;

/**
 *
 * @author ottokuusela
 */
public class SerialLatencyTest {
   
    private SerialPort port;
    
    public void SerialLatencyTest() {
    }
    
    public ArrayList<String> availablePorts() {      
        ArrayList<String> portNames = new ArrayList<>();
     
        for (SerialPort port : SerialPort.getCommPorts()) {
            portNames.add(port.getPortDescription());
        }
        
        return portNames;
    }    
    public boolean connect(String portDescription) {
        this.port = SerialPort.getCommPort(portDescription);
        return this.port.openPort();
    }
    
    public long testLatency() {
        if (this.port == null) {
            return -1;
        }
        
        long start = System.currentTimeMillis();
        
        byte[] testData = "5".getBytes();
        
        this.port.writeBytes(testData, testData.length);
        
        while (this.port.bytesAvailable() == 0) {
            if (System.currentTimeMillis() - start > 1000) {
                return -1;
            }
        }
        
        long end = System.currentTimeMillis();
        
        return end - start;
    }
    
}
