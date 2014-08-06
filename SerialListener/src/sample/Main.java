package sample;

import javafx.application.Application;
import javafx.scene.Scene;
import javafx.stage.Stage;

import java.io.UnsupportedEncodingException;
import java.util.logging.Level;
import java.util.logging.Logger;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.scene.control.Button;
import javafx.scene.control.ComboBox;
import javafx.scene.control.TextField;
import javafx.scene.layout.StackPane;
import javafx.scene.layout.VBox;
import javafx.stage.WindowEvent;
import jssc.SerialPort;
import jssc.SerialPortEvent;
import jssc.SerialPortEventListener;
import jssc.SerialPortException;
import jssc.SerialPortList;

public class Main extends Application {

    private SerialPort _serialPort;
    private ObservableList<String> _portList;

    ComboBox comboBoxPorts;
    TextField textFieldOut, textFieldIn;
    Button btnOpenSerial, btnCloseSerial, btnSend;

    private void detectPort() {

        _portList = FXCollections.observableArrayList();

        String[] serialPortNames = SerialPortList.getPortNames();
        for (String name : serialPortNames) {
            System.out.println(name);
            _portList.add(name);
        }
    }

    @Override
    public void start(Stage primaryStage) {

        detectPort();

        comboBoxPorts = new ComboBox(_portList);
        textFieldOut = new TextField();
        textFieldIn = new TextField();

        btnOpenSerial = new Button("Open Serial Port");
        btnCloseSerial = new Button("Close Serial Port");
        btnSend = new Button("Send");
        btnSend.setDisable(true);   //default disable before serial port open

        btnOpenSerial.setOnAction(t -> {
            closeSerialPort();              //close serial port before open
            if(openSerialPort()){
                btnSend.setDisable(false);
            }else{
                btnSend.setDisable(true);
            }
        });

        btnCloseSerial.setOnAction(t -> {
            closeSerialPort();
            btnSend.setDisable(true);
        });

        btnSend.setOnAction(t -> {

            if(_serialPort != null && _serialPort.isOpened()){
                try {
                    String stringOut = textFieldOut.getText();
                    _serialPort.writeBytes(stringOut.getBytes());
                } catch (SerialPortException ex) {
                    Logger.getLogger("Test").log(Level.SEVERE, null, ex);
                }
            }else{
                System.out.println("Something wrong!");
            }
        });

        VBox vBox = new VBox();
        vBox.getChildren().addAll(
                comboBoxPorts,
                textFieldOut,
                textFieldIn,
                btnOpenSerial,
                btnCloseSerial,
                btnSend);

        StackPane root = new StackPane();
        root.getChildren().add(vBox);

        Scene scene = new Scene(root, 300, 250);

        primaryStage.setOnCloseRequest(t -> closeSerialPort());

        primaryStage.setTitle("Hello World!");
        primaryStage.setScene(scene);
        primaryStage.show();
    }

    public static void main(String[] args) {
        launch(args);
    }

    private boolean openSerialPort() {
        boolean success = false;

        if (comboBoxPorts.getValue() != null
                && !comboBoxPorts.getValue().toString().isEmpty()) {
            try {
                _serialPort = new SerialPort(comboBoxPorts.getValue().toString());

                _serialPort.openPort();
                _serialPort.setParams(
                        SerialPort.BAUDRATE_9600,
                        SerialPort.DATABITS_8,
                        SerialPort.STOPBITS_1,
                        SerialPort.PARITY_NONE);

                _serialPort.addEventListener(new MySerialPortEventListener());

                success = true;
            } catch (SerialPortException ex) {
                Logger.getLogger("Test").log(Level.SEVERE, null, ex);
            }
        }
        return success;
    }

    private void closeSerialPort() {
        if (_serialPort != null && _serialPort.isOpened()) {
            try {
                _serialPort.closePort();
            } catch (SerialPortException ex) {
                Logger.getLogger("Test").log(Level.SEVERE, null, ex);
            }
        }

        _serialPort = null;
    }

    class MySerialPortEventListener implements SerialPortEventListener {

        @Override
        public void serialEvent(SerialPortEvent serialPortEvent) {

            if(serialPortEvent.isRXCHAR()){
                try {
                    int byteCount = serialPortEvent.getEventValue();
                    byte bufferIn[] = _serialPort.readBytes(byteCount);

                    String stringIn = "";
                    try {
                        stringIn = new String(bufferIn, "UTF-8");
                    } catch (UnsupportedEncodingException ex) {
                        Logger.getLogger("Test").log(Level.SEVERE, null, ex);
                    }
                    textFieldIn.setText(stringIn);

                } catch (SerialPortException ex) {
                    Logger.getLogger("Test").log(Level.SEVERE, null, ex);
                }

            }

        }

    }
}