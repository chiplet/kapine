package otto.kiihdytincontrolpanel;

import java.net.URL;
import java.util.ResourceBundle;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.control.Label;
import javafx.scene.control.ListView;


public class FXMLController implements Initializable {
    
    private SerialLatencyTest test;
    
    @FXML
    private Label label;
    @FXML
    private ListView list;
    
    @FXML
    private void handleButtonAction(ActionEvent event) {
        if (this.list.getSelectionModel().getSelectedItem() == null) {
            return;
        }
        
        String port = this.list.getSelectionModel().getSelectedItem().toString();
        this.test.connect(port);
        long result = this.test.testLatency();
        label.setText(Long.toString(result));
    }
    
    @Override
    public void initialize(URL url, ResourceBundle rb) {
        this.test = new SerialLatencyTest();
        ObservableList<String> list = FXCollections.observableArrayList(this.test.availablePorts());
        this.list.setItems(list);
    }    
}
