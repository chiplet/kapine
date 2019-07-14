package otto.kiihdytincontrolpanel;

import javafx.application.Application;
import javafx.collections.FXCollections;
import javafx.scene.Scene;
import javafx.stage.Stage;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.ListView;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.VBox;

public class MainApp extends Application {

    @Override
    public void start(Stage stage) throws Exception {
        SerialLatencyTest test = new SerialLatencyTest();

        ListView list = new ListView(FXCollections.observableArrayList(test.availablePorts()));
        Label infoLabel = new Label();

        Button testButton = new Button();
        testButton.setText("Test");
        testButton.setDisable(true);
        testButton.setOnAction((event) -> {
            long result = test.testLatency();
            infoLabel.setText(Long.toString(result));
        });

        Button connectButton = new Button();
        connectButton.setText("Connect");
        connectButton.setOnAction((event) -> {
            if (connectButton.getText().equals("Disconnect")) {
                boolean disconnectSuccessful = test.disconnect();
                if (disconnectSuccessful) {
                    testButton.setDisable(true);
                    connectButton.setText("Connect");
                    infoLabel.setText("Disconnected");
                    return;
                }
            }

            if (list.getSelectionModel().getSelectedItem() == null) {
                return;
            }

            String port = list.getSelectionModel().getSelectedItem().toString();
            boolean connectionSuccessful = test.connect(port);
            System.out.println("Connection successful? " + connectionSuccessful);

            if (connectionSuccessful) {
                infoLabel.setText("Connected to " + port);
                testButton.setDisable(false);
                connectButton.setText("Disconnect");
            }
        });

        VBox centerControls = new VBox(connectButton, testButton, infoLabel);
        BorderPane pane = new BorderPane();
        pane.setPrefSize(600, 400);
        pane.setLeft(list);
        pane.setCenter(centerControls);

        Scene scene = new Scene(pane);
        scene.getStylesheets().add("/styles/Styles.css");

        stage.setTitle("PC -> Arduino -> PC latency tester");
        stage.setScene(scene);
        stage.show();
    }

    /**
     * The main() method is ignored in correctly deployed JavaFX application.
     * main() serves only as fallback in case the application can not be
     * launched through deployment artifacts, e.g., in IDEs with limited FX
     * support. NetBeans ignores main().
     *
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        launch(MainApp.class);
    }

}
