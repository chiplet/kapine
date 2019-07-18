package otto.kiihdytincontrolpanel;

import javafx.application.Application;
import javafx.collections.FXCollections;
import javafx.geometry.Insets;
import javafx.scene.Scene;
import javafx.stage.Stage;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.layout.VBox;
import javafx.scene.layout.BorderPane;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.control.ComboBox;
import javafx.scene.layout.HBox;
import javafx.scene.paint.Color;

public class MainApp extends Application {

    @Override
    public void start(Stage stage) throws Exception {
        SerialLatencyTest test = new SerialLatencyTest();

        /*ListView list = new ListView(FXCollections.observableArrayList(test.availablePorts()));
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
         */
        BorderPane pane = new BorderPane();
        pane.setPadding(new Insets(20, 20, 20, 20));

        VBox leftArea = new VBox();
        leftArea.setSpacing(10);
        Canvas canvas = new Canvas(400, 300);
        GraphicsContext painter = canvas.getGraphicsContext2D();
        painter.setFill(Color.RED);
        painter.fillRect(0,0,400,300);
        HBox accelerationControls = new HBox();
        accelerationControls.setSpacing(10);
        Button initiateButton = new Button();
        Button terminateButton = new Button();
        Button connectButton = new Button();
        ComboBox deviceSelector = new ComboBox();

        VBox statusPanel = new VBox();
        Label maxSpeedCaption = new Label("Maximum speed:");
        Label maxSpeedData = new Label();
        Label currentStatusLabel = new Label("Not connected");

        initiateButton.setText("Initiate");
        initiateButton.setDisable(true);
        initiateButton.setOnAction((event) -> {
            // TODO: Initiate acceleration
            terminateButton.setDisable(false);
            initiateButton.setDisable(true);
        });

        terminateButton.setText("Terminate");
        terminateButton.setDisable(true);
        terminateButton.setOnAction((event) -> {
            // TODO: Terminate acceleration
            initiateButton.setDisable(false);
            terminateButton.setDisable(true);
        });

        connectButton.setText("Connect");
        connectButton.setOnAction((event) -> {
            if (connectButton.getText().equals("Disconnect")) {
                boolean disconnectSuccessful = test.disconnect();
                if (disconnectSuccessful) {
                    initiateButton.setDisable(true);
                    terminateButton.setDisable(true);
                    connectButton.setText("Connect");
                    currentStatusLabel.setText("Disconnected");
                    return;
                }
            }

            if (deviceSelector.getSelectionModel().getSelectedItem() == null) {
                return;
            }

            String port = deviceSelector.getSelectionModel().getSelectedItem().toString();
            boolean connectionSuccessful = test.connect(port);
            System.out.println("Connection successful? " + connectionSuccessful);

            if (connectionSuccessful) {
                currentStatusLabel.setText("Connected to " + port);
                initiateButton.setDisable(false);
                connectButton.setText("Disconnect");
            }
        });

        deviceSelector.setItems(FXCollections.observableArrayList(test.availablePorts()));

        accelerationControls.getChildren().addAll(
                initiateButton, terminateButton, connectButton, deviceSelector);

        leftArea.getChildren().addAll(canvas, accelerationControls);
        statusPanel.getChildren().addAll(
                maxSpeedCaption, maxSpeedData, currentStatusLabel);
        
        pane.setCenter(leftArea);
        pane.setRight(statusPanel);
        
        Scene scene = new Scene(pane);
        scene.getStylesheets().add("/styles/Styles.css");

        stage.setTitle("Accelerator Control Panel");
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
