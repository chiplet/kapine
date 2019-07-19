package otto;

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

/**
 * JavaFX App
 */
public class App extends Application {

    @Override
    public void start(Stage stage) {
        SerialInterface comm = new SerialInterface();

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
                boolean disconnectSuccessful = comm.disconnect();
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
            boolean connectionSuccessful = comm.connect(port);

            if (connectionSuccessful) {
                currentStatusLabel.setText("Connected to " + port);
                initiateButton.setDisable(false);
                connectButton.setText("Disconnect");
            }
        });

        deviceSelector.setItems(FXCollections.observableArrayList(comm.availablePorts()));

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

    public static void main(String[] args) {
        launch();
    }

}