package otto;

import javafx.application.Application;
import javafx.collections.FXCollections;
import javafx.geometry.Insets;
import javafx.scene.Scene;
import javafx.scene.layout.*;
import javafx.stage.Stage;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.ComboBox;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.XYChart;

import java.io.FileInputStream;
import java.io.DataInputStream;
import java.io.InputStream;
import java.io.File;
import java.io.IOException;


/**
 * JavaFX App
 */
public class MainApp extends Application {

    @Override
    public void start(Stage stage) {
        SerialInterface comm = new SerialInterface();

        AnchorPane pane = new AnchorPane();
        pane.setPadding(new Insets(20, 20, 20, 20));

        AcceleratorStatusCanvas canvas = new AcceleratorStatusCanvas();
        StackPane canvasPanel = new StackPane();
        canvasPanel.getChildren().add(canvas);
        canvasPanel.setPrefSize(500, 350);
        HBox accelerationControls = new HBox();
        accelerationControls.setSpacing(10);
        Button initiateButton = new Button();
        Button terminateButton = new Button();
        Button connectButton = new Button();
        ComboBox deviceSelector = new ComboBox();

        VBox statusPanel = new VBox();
        statusPanel.getStyleClass().add("statusPane");
        statusPanel.setPadding(new Insets(10, 10, 10, 10));
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

        NumberAxis sensorAxis = new NumberAxis(1,6,1);
        NumberAxis timeAxis = new NumberAxis(0, 500, 100);

        // asetetaan akseleille nimet
        sensorAxis.setLabel("Sensori");
        timeAxis.setLabel("Aika (ms)");

        // luodaan viivakaavio. Viivakaavion arvot annetaan numeroina
        // ja se käyttää aiemmin luotuja x- ja y-akseleita
        LineChart<Number, Number> viivakaavio = new LineChart<>(sensorAxis, timeAxis);

        // luodaan viivakaavioon lisättävä datajoukko
        XYChart.Series testiData = new XYChart.Series();
        // lisätään datajoukkoon yksittäisiä pisteitä
        testiData.getData().add(new XYChart.Data(1, 200));
        testiData.getData().add(new XYChart.Data(2, 390));
        testiData.getData().add(new XYChart.Data(3, 400));
        testiData.getData().add(new XYChart.Data(4, 405));
        testiData.getData().add(new XYChart.Data(5, 407));
        testiData.getData().add(new XYChart.Data(6, 408));

        // lisätään datajoukko viivakaavioon
        viivakaavio.getData().add(testiData);
        viivakaavio.setPrefSize(120, 150);

        deviceSelector.setItems(FXCollections.observableArrayList(comm.availablePorts()));

        accelerationControls.getChildren().addAll(
                initiateButton, terminateButton, connectButton, deviceSelector);

        statusPanel.getChildren().addAll(
                maxSpeedCaption, maxSpeedData, viivakaavio, currentStatusLabel);

        pane.getChildren().addAll(canvasPanel, accelerationControls, statusPanel);
        AnchorPane.setLeftAnchor(canvasPanel, 0.0);
        AnchorPane.setTopAnchor(canvasPanel, 0.0);
        AnchorPane.setBottomAnchor(canvasPanel, 50.0);
        AnchorPane.setRightAnchor(canvasPanel, 245.0);
        AnchorPane.setLeftAnchor(accelerationControls, 0.0);
        AnchorPane.setBottomAnchor(accelerationControls, 0.0);
        AnchorPane.setRightAnchor(statusPanel, 0.0);
        AnchorPane.setTopAnchor(statusPanel, 0.0);
        AnchorPane.setBottomAnchor(statusPanel, 0.0);

        canvas.widthProperty().bind(canvasPanel.widthProperty());
        canvas.heightProperty().bind(canvasPanel.heightProperty());

        pane.setPrefSize(800, 600);

        Scene scene = new Scene(pane);
        //scene.getStylesheets().add("/styles/Styles.css");

        try {
            File file = new File("blob.bin");
            InputStream input = new FileInputStream(file);
            DataInputStream stream = new DataInputStream(input);
            byte[] bytes = new byte[stream.available()];
            stream.readFully(bytes);


            //byte[] bytes = {170, 85, 1, 17, 74, 101, 101, 33, 32, 74, 111, 107, 111, 32, 116, 111, 105, 109, 105, 105, 63, 146, 235};
            DataPacket packet = DataPacket.fromBytes(bytes);
            System.out.println(packet.toString());
        } catch (IOException e) {
            System.out.println(e.getMessage());
        }


        stage.setTitle("Accelerator Control Panel");
        stage.setScene(scene);
        stage.show();
    }

    public static void main(String[] args) {
        launch();
    }

}