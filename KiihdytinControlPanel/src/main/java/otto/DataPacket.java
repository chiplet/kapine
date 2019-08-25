package otto;

import java.util.ArrayList;

public class DataPacket {

    private MessageClass type;
    private ArrayList<Byte> messageContent;

    public DataPacket(MessageClass type, ArrayList<Byte> content) {
        this.type = type;
        this.messageContent = content;
    }

}

enum MessageClass {
    ACCELERATION_START, MAGNET_ON, MAGNET_OFF, SENSOR_DETECTION
}
