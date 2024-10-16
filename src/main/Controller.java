package main;

import coppelia.*;

import javafx.fxml.FXML;
import javafx.application.Platform;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;
import javafx.scene.shape.Circle;
import javafx.stage.WindowEvent;

import javafx.scene.control.Label;
import javafx.scene.canvas.Canvas;
import javafx.scene.control.Button;
import javafx.scene.image.PixelWriter;
import javafx.scene.control.Dialog;

import javafx.concurrent.Task;

import utils.*;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.awt.image.DataBufferInt;
import java.nio.ByteBuffer;
import java.util.Arrays;

import static com.googlecode.javacv.cpp.opencv_core.*;
import static com.googlecode.javacv.cpp.opencv_highgui.cvLoadImage;
import static com.googlecode.javacv.cpp.opencv_imgproc.*;

/**
 * Created by Theo Theodoridis.
 * Class    : Controller
 * Version  : v1.0
 * Date     : © Copyright 01/11/2020
 * User     : ttheod
 * email    : ttheod@gmail.com
 * Comments : None.
 **/

public class Controller implements IController {
    //region Variables declaration
    //region GUI variables declaration
    @FXML
    private Button btnConnect;
    @FXML
    private Button btnRight;
    @FXML
    private Button btnLeft;
    @FXML
    private Button btnBack;
    @FXML
    private Button btnForward;
    @FXML
    private Button btnStop;
    @FXML
    private Canvas canvasCamera;
    @FXML
    private Label lblSensor0;
    @FXML
    private Label lblSensor1;
    @FXML
    private Label lblSensor2;
    @FXML
    private Label lblSensor3;
    @FXML
    private Label lblSensor4;
    @FXML
    private Label lblSensor5;
    @FXML
    private Label lblState;
    @FXML
    private Circle sonar0Led;
    @FXML
    private Circle sonar1Led;
    @FXML
    private Circle sonar2Led;
    @FXML
    private Circle sonar3Led;
    @FXML
    private Circle sonar4Led;
    @FXML
    private Circle sonar5Led;
    @FXML
    private Circle gpsLed;
    @FXML
    private Circle cameraLed;
    @FXML
    private Circle wheelsLed;
    @FXML
    private Circle rightWheelLed;
    @FXML
    private Circle leftWheelLed;
    @FXML
    private Label lblGpsX;
    @FXML
    private Label lblGpsY;
    @FXML
    private Label lblGpsZ;
    @FXML
    private Label lblRightWheel;
    @FXML
    private Label lblLeftWheel;
    @FXML
    private Button btnPositionChart;

    private String defaultButtonStyle;
    private final Circle[] sonarLeds = new Circle[6];
    private PixelWriter pw;

    private final static javafx.scene.paint.Color GREEN_LED = javafx.scene.paint.Color.CHARTREUSE;
    private final static javafx.scene.paint.Color RED_LED = javafx.scene.paint.Color.RED;
    private final static javafx.scene.paint.Color ORANGE_LED = javafx.scene.paint.Color.ORANGE;
    private final static javafx.scene.paint.Color GRAY_LED = javafx.scene.paint.Color.DARKGRAY;

    private Stage positionChartsStage;
    //endregion

    //region Sensors variables declaration
    private final int SONARS_NUMBER = 6;
    private final int WHEEL_NUMBER = 2; // LEFT-RIGHT
    private final int GPS_NUMBER = 6; // X, Y, Z, Rx, Ry et Rz
    private double[] gpsValues = new double[GPS_NUMBER];
    private double[] sonarValues = new double[SONARS_NUMBER];
    private final double[] encoderValues = new double[WHEEL_NUMBER];

    private boolean runLeftWheelEncoder;
    private boolean runRightWheelEncoder;
    private boolean runWheelEncoder;
    private boolean runMotion;
    private boolean runCamera;
    private boolean runGPS;
    private boolean runAtLeastOneSonar;

    private boolean runningStateLeftWheelIsOK;
    private boolean runningStateRightWheelIsOK;
    private boolean runningStateCameraIsOK;
    private boolean runningStateGPSIsOK;
    private final boolean[] runningStateSonars = new boolean[SONARS_NUMBER];

    private final IntW cameraHandle = new IntW(-1);
    private final IntW gpsHandle = new IntW(-1);
    private final IntW leftWheelHandle = new IntW(-1);
    private final IntW rightWheelHandle = new IntW(-1);
    private final IntW[] sonarsHandles = new IntW[SONARS_NUMBER];

    private boolean isPowerEmpty;
    //endregion

    //region Robot variables declaration
    private volatile boolean running = false;

    private MotionDirections dir = MotionDirections.Stop; // Direction.
    private final int vel = 5;    // Velocity.

    private boolean hasOrientationAim = false;
    private double orientationAim = 0.;
    //endregion

    //region Camera variables declaration
    private double targetMinScore = 0.0;
    private double targetMaxScore = 0.0;

    private final int resolutionCamera = 256;
    private final CvPoint target = new CvPoint();
    private final IntWA resolution = new IntWA(1);
    private final CharWA image = new CharWA(resolutionCamera * resolutionCamera * 3);
    private BufferedImage bufferedImageRGB;
    private volatile boolean isComputingTheImage = false;
    private volatile boolean isWrittingTheImage = false;

    private IplImage markerImage;
    //endregion

    //region Wheel encoders variables declaration
    private float dxRight = 0;
    private float totalRightJointPosition = 0;
    private float currentRightJointPosition = 0;
    private float previousRightJointPosition = 0;
    private FloatW robotRightJointPosition = new FloatW(3);

    private float dxLeft = 0;
    private float totalLeftJointPosition = 0;
    private float currentLeftJointPosition = 0;
    private float previousLeftJointPosition = 0;
    private FloatW robotLeftJointPosition = new FloatW(3);
    //endregion

    //region V-rep communication and simulation variables declaration
    private remoteApi vRep;
    private int clientID = -1;

    private final int    VREP_SENS_AXIS_X = -1;     // To reverse the x axis (-1) or not (1) in VREP simulation
    private final int    VREP_SENS_AXIS_Y = -1;     // To reverse the y axis (-1) or not (1) in VREP simulation
    private final double VREP_START_POS_X = +2.130; // The robot start x pos in the VREP simulation
    private final double VREP_START_POS_Y = -0.775; // The robot start y pos in the VREP simulation

    private final double CHARGER_XCOORD = 0.; // The charger X coordinate. Ori : 1.78
    private final double CHARGER_YCOORD = 0.; // The charger Y coordinate. Ori : -0.78
    private final double MAX_GPS_DIST = 5.0;  // The max Euclidean distance from the charger.

    private final String VREP_IPV4 = "127.0.0.1";
    private final int VREP_PORT = 20001;

    private final String VREP_ROBOT_NAME = "Roomba";
    private final String VREP_ROBOT_LEFT_WHEEL_NAME = "JointLeftWheel";
    private final String VREP_ROBOT_RIGHT_WHEEL_NAME = "JointRightWheel";//"FHkfhkf";
    private final String VREP_ROBOT_VISION_SENSOR_NAME = "Vision_sensor";
    private final String VREP_ROBOT_BASE_SONAR_NAME = "Proximity_sensor";

    private final float MARKER_THRESHOLD = 0.6f;
    //endregion

    //region Timers variables declaration
    private int MAX_BATT_TIME = 60 * 20; // Default 20 mins battery time.
    private final int MAX_BATT_VOLT = 12;      // volts.

    private Timer motionTimer;
    private Timer batteryTimer;
    //endregion

    //region Threads variables declaration
    private Thread updateUIThread;
    private Thread updateSensorThread;
    private Thread mainThread;
    //endregion

    //region States variables declaration
    private States currentState = States.None;
    private States requestState = States.Initialize;
    //endregion

    private Stage primaryStage = null;

    //endregion

    //region Methods

    //region Battery methods
    private int getBatteryTime() {
        return (batteryTimer.getSec());
    }

    private void setBatteryTime(int min) {
        MAX_BATT_TIME = 60 * min;
        motionTimer.setSec(MAX_BATT_TIME);
        motionTimer.restart();
    }

    private double getBatteryCapacity() {
        double v = (double) MAX_BATT_VOLT - Utils.map(batteryTimer.getSec(), 0, (double) MAX_BATT_TIME, 0, (double) MAX_BATT_VOLT);
        return (Math.max(v, 0.0));
    }

    private double getBatteryPercentage() {
        double v = getBatteryCapacity();

        if ((v >= 9.6) && (v <= 12)) return (100.0);
        else if ((v >= 7.2) && (v < 9.6)) return (80.0);
        else if ((v >= 4.8) && (v < 7.2)) return (60.0);
        else if ((v >= 2.4) && (v < 4.8)) return (40.0);
        else if ((v > 1.0) && (v < 2.4)) return (20.0);
        else
            return (0.0);
    }

    private boolean getBatteryState() {
        double v = getBatteryCapacity();
        return (v > 0.0);
    }

    private boolean isBatteryLow(){
        return getBatteryPercentage() <= 20.0;
    }

    //endregion

    //region Wheel methods
    private double readRightWheelEnc() {
        int result = vRep.simxGetJointPosition(clientID, rightWheelHandle.getValue(), robotRightJointPosition, remoteApi.simx_opmode_buffer);
        runningStateRightWheelIsOK = result != remoteApi.simx_return_remote_error_flag;
        currentRightJointPosition = robotRightJointPosition.getValue();
        dxRight = getAngleMinusAlpha(currentRightJointPosition, previousRightJointPosition);
        totalRightJointPosition += dxRight;

        previousRightJointPosition = currentRightJointPosition;
        return (Math.round((totalRightJointPosition / (2 * Math.PI)) * 100d) / 100d);
    }

    private double readLeftWheelEnc() {
        int result = vRep.simxGetJointPosition(clientID, leftWheelHandle.getValue(), robotLeftJointPosition, remoteApi.simx_opmode_buffer);
        runningStateLeftWheelIsOK = result != remoteApi.simx_return_remote_error_flag;
        currentLeftJointPosition = robotLeftJointPosition.getValue();
        dxLeft = getAngleMinusAlpha(currentLeftJointPosition, previousLeftJointPosition);
        totalLeftJointPosition += dxLeft;

        previousLeftJointPosition = currentLeftJointPosition;
        return (Math.round((totalLeftJointPosition / (2 * Math.PI)) * 100d) / 100d);
    }

    private float getAngleMinusAlpha(float angle, float alpha) {
        double sinAngle0 = Math.sin(angle);
        double sinAngle1 = Math.sin(alpha);
        double cosAngle0 = Math.cos(angle);
        double cosAngle1 = Math.cos(alpha);
        double sin_da = sinAngle0 * cosAngle1 - cosAngle0 * sinAngle1;
        double cos_da = cosAngle0 * cosAngle1 + sinAngle0 * sinAngle1;
        return ((float) Math.atan2(sin_da, cos_da));
    }

    private double getLeftWheelEnc() {
        return (encoderValues[0]);
    }

    private double getRightWheelEnc() {
        return (encoderValues[1]);
    }

    private int getEncoderNo() {
        return WHEEL_NUMBER;
    }
    //endregion

    //region GPS methods
    private double[] readGPS() {
        FloatWA position = new FloatWA(3);
        FloatWA orientation = new FloatWA(3);
        int result = vRep.simxGetObjectPosition(clientID, gpsHandle.getValue(), -1, position, remoteApi.simx_opmode_streaming);
        vRep.simxGetObjectOrientation(clientID, gpsHandle.getValue(), -1, orientation, remoteApi.simx_opmode_streaming);
        runningStateGPSIsOK = result != remoteApi.simx_return_remote_error_flag;

        double[] positions = new double[GPS_NUMBER];
        positions[0] = (VREP_SENS_AXIS_X * (double) position.getArray()[0] + VREP_START_POS_X);
        positions[1] = (VREP_SENS_AXIS_Y * (double) position.getArray()[1] + VREP_START_POS_Y);
        positions[2] = position.getArray()[2];
        positions[3] = (orientation.getArray()[0]*180.)/Math.PI;
        positions[4] = (orientation.getArray()[1]*180.)/Math.PI;
        positions[5] = (orientation.getArray()[2]*180.)/Math.PI;

        return (positions);
    }

    private double getGPSX() {
        return (gpsValues[0]);
    }

    private double getGPSY() {
        return (gpsValues[1]);
    }

    private double getGPSZ() {
        return (gpsValues[2]);
    }

    private double getGPSRx() {
        return (gpsValues[3]);
    }

    private double getGPSRy() {
        return (gpsValues[4]);
    }

    private double getGPSRz() {
        return (gpsValues[5]);
    }


    private int getGPSNo() {
        return GPS_NUMBER;
    }
    //endregion

    //region Ultrasonic methods
    private double readSonarRange(int sensor) {
        BoolW detectionState = new BoolW(false);
        FloatWA detectedPoint = new FloatWA(1); //Coordinates relatives to the sensor's frame
        IntW detectedObjectHandle = new IntW(1);
        FloatWA detectedSurfaceNormalVector = new FloatWA(1);

        int result = vRep.simxReadProximitySensor(clientID, sonarsHandles[sensor].getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_buffer);
        runningStateSonars[sensor] = result != remoteApi.simx_return_remote_error_flag;

        float[] detectedPointXYZ = detectedPoint.getArray();
        double distance = Math.sqrt(Math.pow(detectedPointXYZ[0], 2) + Math.pow(detectedPointXYZ[1], 2) + Math.pow(detectedPointXYZ[2], 2));
        distance = Math.round(distance * 100.) / 100.;
        distance = Utils.getDecimal(distance, "0.0");
        if (distance >= 1. || distance == 0.)
        {
            distance = 1.;
        }

        return detectionState.getValue() ? distance : 1.;
    }

    private double[] readSonars() {
        for (int i = 0; i < getSonarNo(); i++)
        {
            if (sonarsHandles[i].getValue() != remoteApi.simx_return_remote_error_flag)
            {
                sonarValues[i] = readSonarRange(i);
            }
        }
        return sonarValues;
    }

    private double[] getSonarRanges() {
        return (sonarValues);
    }

    private double getSonarRange(int sensor) {
        return (sonarValues[sensor]);
    }

    private int getSonarNo() {
        return SONARS_NUMBER;
    }

    private boolean isObstacleDetected(){
        return sonarValues[2] <= 0.1 || sonarValues[3] <= 0.1;
    }
    //endregion

    //region Camera methods
    private void readCamera() {
        int result = vRep.simxGetVisionSensorImage(clientID, cameraHandle.getValue(), resolution, image, 2, remoteApi.simx_opmode_buffer);
        runningStateCameraIsOK = result != remoteApi.simx_return_remote_error_flag;
        if (!isComputingTheImage) {
            isWrittingTheImage = true;
                    bufferedImageRGB = charWAtoBufferedImage(image);
            isWrittingTheImage = false;
        }
    }

    private BufferedImage charWAtoBufferedImage(CharWA image) {
        char[] imageArray = image.getArray();
        BufferedImage bufferedImage = new BufferedImage(resolutionCamera, resolutionCamera, BufferedImage.TYPE_INT_RGB);
        int index = 0;
        int r, g, b;
        for (int y = 0; y < resolutionCamera; y++) {
            for (int x = 0; x < resolutionCamera; x++) {
                r = (int) imageArray[index] & 0xFF;
                g = (int) imageArray[index + 1] & 0xFF;
                b = (int) imageArray[index + 2] & 0xFF;
                int rgb = (r << 16) | (g << 8) | b;
                bufferedImage.setRGB(x, resolutionCamera - 1 - y, rgb);
                index += 3;
            }
        }
        return bufferedImage;
    }

    private BufferedImage bufferedImageToGray(BufferedImage image) {
        BufferedImage grayImage = new BufferedImage(image.getWidth(), image.getHeight(), BufferedImage.TYPE_BYTE_GRAY);
        for (int i = 0; i < image.getWidth(); i++) {
            for (int j = 0; j < image.getHeight(); j++) {
                int rgb = image.getRGB(i, j);
                Color color = new Color(rgb);
                int grayLevel = (int) (0.299 * color.getRed() + 0.587 * color.getGreen() + 0.114 * color.getBlue());
                int grayRgb = new Color(grayLevel, grayLevel, grayLevel).getRGB();
                grayImage.setRGB(i, j, grayRgb);
            }
        }

        return grayImage;  // Retourner l'image en niveaux de gris
    }

    private int getGrayscale(BufferedImage img, int x, int y) {
        java.awt.Color c = new java.awt.Color(img.getRGB(x, y));
        int r = (int) (c.getRed() * 0.299);
        int g = (int) (c.getGreen() * 0.587);
        int b = (int) (c.getBlue() * 0.114);
        return r + g + b;
    }

    private int getTargetX() {
        return (target.x());
    }

    private int getTargetY() {
        return (target.y());
    }

    private double getTargetMinScore() {
        return (targetMinScore);
    }

    private double getTargetMaxScore() {
        return (targetMaxScore);
    }

    private void displayBufferedImage(BufferedImage bufferedImage) {
        double width = bufferedImage.getWidth();
        double height = bufferedImage.getHeight();
        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                int rgb = bufferedImage.getRGB(x, y);
                int r = (rgb >> 16) & 0xFF;
                int g = (rgb >> 8) & 0xFF;
                int b = rgb & 0xFF;
                javafx.scene.paint.Color fxColor = javafx.scene.paint.Color.rgb(r, g, b);
                pw.setColor(x, y, fxColor);
            }
        }
    }

    private CvPoint findMarker(IplImage src) {
        IplImage result = cvCreateImage(cvSize(src.width() - markerImage.width() + 1, src.height() - markerImage.height() + 1), IPL_DEPTH_32F, 1);

        if (src.width() < markerImage.width() || src.height() < markerImage.height()) {
            System.out.println("Invalid dimensions.");
            return null;
        }

        cvMatchTemplate(src, markerImage, result, CV_TM_CCOEFF_NORMED);

        double[] minVal = new double[1];
        double[] maxVal = new double[1];
        CvPoint minLoc = new CvPoint();
        CvPoint maxLoc = new CvPoint();
        cvMinMaxLoc(result, minVal, maxVal, minLoc, maxLoc, null);

        if (maxVal[0] > MARKER_THRESHOLD) {
            target.x(maxLoc.x() + markerImage.width() / 2);
            target.y(maxLoc.y() + markerImage.height() / 2);
            return maxLoc;
        }
        return null;
    }

    private void drawRectangle(IplImage image, CvPoint matchLocation) {
        cvRectangle(image, matchLocation,
                cvPoint(matchLocation.x() + markerImage.width(), matchLocation.y() + markerImage.height()),
                CvScalar.RED, 2, 8, 0);
    }

    private IplImage bufferedImageToIplImage(BufferedImage bufferedImage) {
        int width = bufferedImage.getWidth();
        int height = bufferedImage.getHeight();
        int channels = (bufferedImage.getType() == BufferedImage.TYPE_INT_RGB || bufferedImage.getType() == BufferedImage.TYPE_INT_ARGB) ? 3 : 1;

        IplImage iplImage = IplImage.create(width, height, IPL_DEPTH_8U, channels);

        ByteBuffer buffer = iplImage.getByteBuffer();
        if (buffer.capacity() < width * height * channels) {
            System.out.println("Buffer capacity is not sufficient for the image.");
            return null;
        }

        if (bufferedImage.getType() == BufferedImage.TYPE_INT_RGB || bufferedImage.getType() == BufferedImage.TYPE_INT_ARGB) {
            int[] data = ((DataBufferInt) bufferedImage.getRaster().getDataBuffer()).getData();
            byte[] byteData = new byte[width * height * 3]; // pour RGB
            for (int i = 0; i < data.length; i++) {
                byteData[i * 3] = (byte) ((data[i] >> 16) & 0xFF);  // Red
                byteData[i * 3 + 1] = (byte) ((data[i] >> 8) & 0xFF);   // Green
                byteData[i * 3 + 2] = (byte) (data[i] & 0xFF);           // Blue
            }
            buffer.put(byteData);
        }
        else if (bufferedImage.getType() == BufferedImage.TYPE_3BYTE_BGR) {
            byte[] data = ((DataBufferByte) bufferedImage.getRaster().getDataBuffer()).getData();
            buffer.put(data);
        }
        else if (bufferedImage.getType() == BufferedImage.TYPE_BYTE_GRAY) {
            byte[] data = ((DataBufferByte) bufferedImage.getRaster().getDataBuffer()).getData();
            buffer.put(data);
        }
        else {
            System.out.println("Unsupported BufferedImage type: " + bufferedImage.getType());
            return null;
        }

        return iplImage;
    }

    private BufferedImage iplImageToBufferedImage(IplImage iplImage) {
        int width = iplImage.width();
        int height = iplImage.height();
        int channels = iplImage.nChannels();
        byte[] data = new byte[width * height * channels];
        iplImage.getByteBuffer().get(data);

        BufferedImage image = new BufferedImage(width, height, channels == 3 ? BufferedImage.TYPE_3BYTE_BGR : BufferedImage.TYPE_BYTE_GRAY);
        image.getRaster().setDataElements(0, 0, width, height, data);
        return image;
    }

    //endregion

    //region Motion Methods
    private void resetButtonsStyle() {
        btnRight.setStyle(defaultButtonStyle);
        btnStop.setStyle(defaultButtonStyle);
        btnLeft.setStyle(defaultButtonStyle);
        btnForward.setStyle(defaultButtonStyle);
        btnBack.setStyle(defaultButtonStyle);
    }

    private void setVel(float lVel, float rVel) {
        vRep.simxSetJointTargetVelocity(clientID, leftWheelHandle.getValue(), lVel, remoteApi.simx_opmode_oneshot);
        vRep.simxSetJointTargetVelocity(clientID, rightWheelHandle.getValue(), rVel, remoteApi.simx_opmode_oneshot);
    }

    private void move(float vel) {
        setVel(vel, vel);
    }

    private void move(float vel, int time) {
        motionTimer.setMs(time);
        motionTimer.restart();
        while (motionTimer.getState()) {
            move(vel);
        }
        btnStopPressed();
    }

    private void turnSpot(float vel) {
        setVel(vel, -vel);
    }

    private void turnSpot(float vel, int time) {
        motionTimer.setMs(time);
        motionTimer.restart();
        while (motionTimer.getState()) {
            turnSpot(vel);
        }
        btnStopPressed();
    }

    private void turnSharp(float vel) {
        if (vel > 0) setVel(vel, 0);
        else setVel(0, -vel);
    }

    private void turnSharp(float vel, int time) {
        motionTimer.setMs(time);
        motionTimer.restart();
        while (motionTimer.getState()) {
            turnSharp(vel);
        }
        btnStopPressed();
    }

    private void turnSmooth(float vel) {
        if (vel > 0) setVel(vel, vel / 2);
        else setVel(-vel / 2, -vel);
    }

    private void turnSmooth(float vel, int time) {
        motionTimer.setMs(time);
        motionTimer.restart();
        while (motionTimer.getState()) {
            turnSmooth(vel);
        }
        btnStopPressed();
    }

    private double generateRotationAim() {
        double result = getGPSRz() + -180 + (360 * Rand.getDouble());
        if (result > 180){
            result = result - 360;
        } else if (result < -180){
            result = result + 360;
        }
        return result;
    }

    private void teleoperate() {
        switch (dir) {
            case Stop:
                move(0);
                break;
            case Forward:
                move(+vel);
                break;
            case Backward:
                move(-vel);
                break;
            case Right:
                turnSpot(+(float)(vel / 2));
                break;
            case Left:
                turnSpot(-(float)(vel / 2));
                break;
        }
    }
    //endregion

    //region UI methods
    private void handleCloseEvent()
    {
        btnDisconnectPressed();
    }

    private void handlePositionChartsCloseEvent()
    {
        positionChartsStage.setOnCloseRequest((WindowEvent event) -> handlePositionChartsCloseEvent());
        positionChartsStage = null;
    }

    public void btnConnectPressed()
    {
        if (clientID == -1)
        {
            btnConnect.setStyle(defaultButtonStyle);
            btnConnect.setDisable(true);
            btnConnect.setText("Connecting...");

            // create a new task for to connection to not block the UI
            Task<Boolean> task = new Task<Boolean>() {
                @Override
                protected Boolean call() throws Exception {
                    return connectToVrep();
                }

                @Override
                protected void succeeded() {
                    boolean connected = getValue();
                    btnConnect.setDisable(false);
                    if (connected)
                    {
                        btnConnect.setText("Connected");
                        btnConnect.setStyle("-fx-background-color: #7FFF00; ");
                        setup();
                    }
                    else
                    {
                        btnConnect.setText("Failed");
                        btnConnect.setStyle("-fx-background-color: #FF0000; ");
                    }
                }

                @Override
                protected void failed() {
                    btnConnect.setDisable(false);
                    btnConnect.setText("Failed");
                    btnConnect.setStyle("-fx-background-color: #FF0000; ");
                }
            };

            new Thread(task).start(); // run the task asynchronously
        }
        else
        {
            btnDisconnectPressed();
        }
    }

    private void btnDisconnectPressed()
    {
        btnConnect.setText("Disconnecting...");
        btnConnect.setStyle(defaultButtonStyle);
        btnConnect.setDisable(true);
        setDisableAllMotionButtons(true);
        if (positionChartsStage != null)
        {
            positionChartsStage.close();
            handlePositionChartsCloseEvent();
        }

        // creation of a task to disconnect properly and not block the UI
        Task<Void> task = new Task<Void>() {
            @Override
            protected Void call() throws Exception {
                kill();
                return null;
            }

            @Override
            protected void succeeded() {
                resetUIGeneral();
            }

            @Override
            protected void failed() {
                resetUIGeneral();
            }
        };

        // run the task asynchronously to not block the UI
        new Thread(task).start();
    }

    public void btnForwardPressed() {
        resetButtonsStyle();
        btnForward.setStyle("-fx-background-color: #7FFF00; ");
        dir = MotionDirections.Forward;
    }

    public void btnBackwardPressed() {
        resetButtonsStyle();
        btnBack.setStyle("-fx-background-color: #7FFF00; ");
        dir = MotionDirections.Backward;
    }

    public void btnLeftPressed() {
        resetButtonsStyle();
        btnLeft.setStyle("-fx-background-color: #7FFF00; ");
        dir = MotionDirections.Left;
    }

    public void btnRightPressed() {
        resetButtonsStyle();
        btnRight.setStyle("-fx-background-color: #7FFF00; ");
        dir = MotionDirections.Right;
    }

    public void btnStopPressed() {
        resetButtonsStyle();
        btnStop.setStyle("-fx-background-color: #7FFF00; ");
        dir = MotionDirections.Stop;
    }

    private void resetUILabels()
    {
        lblSensor0.setText(" 0.0m");
        lblSensor1.setText(" 0.0m");
        lblSensor2.setText(" 0.0m");
        lblSensor3.setText(" 0.0m");
        lblSensor4.setText(" 0.0m");
        lblSensor5.setText(" 0.0m");

        lblGpsX.setText("X:");
        lblGpsY.setText("Y:");
        lblGpsZ.setText("Z:");

        lblRightWheel.setText(" Right:");
        lblLeftWheel.setText(" Left:");
    }

    private void resetUILeds()
    {
        cameraLed.setFill(GRAY_LED);
        gpsLed.setFill(GRAY_LED);
        rightWheelLed.setFill(GRAY_LED);
        leftWheelLed.setFill(GRAY_LED);
        wheelsLed.setFill(GRAY_LED);
        for (Circle sonarLed : sonarLeds) {
            sonarLed.setFill(GRAY_LED);
        }
    }

    private void resetUIGeneral()
    {
        btnConnect.setDisable(false);
        btnConnect.setText("Connect");
        resetUILabels();
        resetUILeds();
        setDisableAllMotionButtons(true);
        lblState.setText(States.None.name());
    }

    private void setDisableAllMotionButtons(boolean setDisable)
    {
        btnForward.setDisable(setDisable);
        btnBack.setDisable(setDisable);
        btnRight.setDisable(setDisable);
        btnLeft.setDisable(setDisable);
        btnStop.setDisable(setDisable);

        btnPositionChart.setDisable(setDisable);
    }

    private void updateLeds()
    {
        for (int i = 0; i < SONARS_NUMBER; i++) {
            sonarLeds[i].setFill(runningStateSonars[i] ? javafx.scene.paint.Color.CHARTREUSE : javafx.scene.paint.Color.RED);
        }
        rightWheelLed.setFill(runningStateRightWheelIsOK ? GREEN_LED :  RED_LED);
        leftWheelLed.setFill(runningStateLeftWheelIsOK ? GREEN_LED :  RED_LED);
        wheelsLed.setFill((runningStateRightWheelIsOK && runningStateLeftWheelIsOK) ? GREEN_LED : ((runningStateRightWheelIsOK || runningStateLeftWheelIsOK) ? ORANGE_LED : RED_LED));
        cameraLed.setFill(runningStateCameraIsOK ? GREEN_LED :  RED_LED);
        gpsLed.setFill(runningStateGPSIsOK ? GREEN_LED :  RED_LED);
    }

    public void btnPositionChartPressed()
    {
        if (positionChartsStage == null)
        {
            Parent root = null;
            try
            {
                FXMLLoader fxmlLoader = new FXMLLoader(getClass().getResource("PositionChartsGUI.fxml"));
                root = fxmlLoader.load();
            }
            catch (Exception e)
            {
                Dialog<Void> d = new Dialog<>();
                d.setTitle("Error");
                d.setContentText("Unable to open the Position Charts GUI.");
                d.showAndWait();
            }
            if (root != null)
            {
                positionChartsStage = new Stage();
                positionChartsStage.setTitle("Position Charts");
                positionChartsStage.setScene(new Scene(root));

                positionChartsStage.setOnCloseRequest((WindowEvent event) -> handlePositionChartsCloseEvent());
                positionChartsStage.show();
            }
        }
        else
        {
            positionChartsStage.requestFocus();
        }
    }
    //endregion

    //region Generic Methods
    @Override
    public void setStage(Stage stage)
    {
        if (primaryStage != null)
        {
            primaryStage.removeEventHandler(WindowEvent.WINDOW_CLOSE_REQUEST, (WindowEvent event) -> handleCloseEvent());
        }
        primaryStage = stage;
        if (primaryStage != null)
        {
            primaryStage.setOnCloseRequest((WindowEvent event) -> handleCloseEvent());
        }
    }

    @Override
    public void init()
    {
        defaultButtonStyle = btnForward.getStyle();
        markerImage = cvLoadImage("data/images/marker.jpg", 0);
        pw = canvasCamera.getGraphicsContext2D().getPixelWriter();
        vRep = new remoteApi();
        vRep.simxFinish(-1); // close all simulations... to be sure

        sonarLeds[0] = sonar0Led;
        sonarLeds[1] = sonar1Led;
        sonarLeds[2] = sonar2Led;
        sonarLeds[3] = sonar3Led;
        sonarLeds[4] = sonar4Led;
        sonarLeds[5] = sonar5Led;
        setDisableAllMotionButtons(true);
    }

    private boolean connectToVrep()
    {
        clientID = vRep.simxStart(VREP_IPV4, VREP_PORT, true, true, 5000, 5);
        return clientID != -1;
    }

    private void kill()
    {
        forceStop();
        killThreads();
        killTimers();
        forceStop();
        disconnectToVrep();
    }

    private void forceStop()
    {
        requestState = States.None;
        dir = MotionDirections.Stop;
        vRep.simxSetJointTargetVelocity(clientID, leftWheelHandle.getValue(), 0, remoteApi.simx_opmode_blocking);
        vRep.simxSetJointTargetVelocity(clientID, rightWheelHandle.getValue(), 0, remoteApi.simx_opmode_blocking);
    }

    private void killTimers()
    {
        if (motionTimer != null)
        {
            motionTimer.cancelAndPurge();
            motionTimer = null;
        }
        if (batteryTimer != null)
        {
            batteryTimer.cancelAndPurge();
            batteryTimer = null;
        }
    }

    private void killThreads()
    {
        running = false;
        while ((updateSensorThread != null && updateSensorThread.isAlive()) ||
                (updateUIThread != null && updateUIThread.isAlive()) ||
                (mainThread != null && (mainThread.isAlive() && !isPowerEmpty)));

        System.out.println("All threads stopped");
        updateUIThread = null;
        updateSensorThread = null;
        mainThread = null;
    }

    private void disconnectToVrep()
    {
        if (clientID != -1)
        {
            //vRep.simxStopSimulation(clientID, remoteApi.simx_opmode_blocking);
            vRep.simxFinish(clientID);
            clientID = -1;
        }
    }

    private void setup()
    {
        if (!running)
        {
            setupSensors();
            firstReadAllSensors();
            setDisableAllMotionButtons(!runMotion);

            forceStop();
            requestState = States.Initialize;
            running = true;

            initTimers(true);

            isPowerEmpty = false;
            setBatteryTime(10); // set battery for 20 minutes
            initThreads(true);
        }
    }

    private void setupSensors()
    {
        // reset handles
        gpsHandle.setValue(-1);
        cameraHandle.setValue(-1);
        leftWheelHandle.setValue(-1);
        rightWheelHandle.setValue(-1);
        for (int i = 0; i < SONARS_NUMBER; i++)
        {
            if (sonarsHandles[i] == null)
            {
                sonarsHandles[i] = new IntW(-1);
            }
            else
            {
                sonarsHandles[i].setValue(-1);
            }
        }
        runLeftWheelEncoder = false;
        runRightWheelEncoder = false;
        runWheelEncoder = false;
        runMotion = false;
        runCamera = false;
        runGPS = false;
        runAtLeastOneSonar = false;
        Arrays.fill(runningStateSonars, false);

        // connect handle and update config
        int result;
        result = vRep.simxGetObjectHandle(clientID, VREP_ROBOT_LEFT_WHEEL_NAME, leftWheelHandle, remoteApi.simx_opmode_blocking);
        runRightWheelEncoder = result == remoteApi.simx_return_ok;
        runningStateRightWheelIsOK = runRightWheelEncoder;

        result = vRep.simxGetObjectHandle(clientID, VREP_ROBOT_RIGHT_WHEEL_NAME, rightWheelHandle, remoteApi.simx_opmode_blocking);
        runLeftWheelEncoder = result == remoteApi.simx_return_ok;
        runningStateLeftWheelIsOK = runLeftWheelEncoder;

        runWheelEncoder = runRightWheelEncoder && runLeftWheelEncoder;
        runMotion = runWheelEncoder;

        result = vRep.simxGetObjectHandle(clientID, VREP_ROBOT_VISION_SENSOR_NAME, cameraHandle, remoteApi.simx_opmode_blocking);
        runCamera = result == remoteApi.simx_return_ok;
        runningStateCameraIsOK = runCamera;

        result = vRep.simxGetObjectHandle(clientID, VREP_ROBOT_NAME, gpsHandle, remoteApi.simx_opmode_blocking);
        runGPS = result == remoteApi.simx_return_ok;
        runningStateGPSIsOK = runGPS;

        for (int i = 0; i < SONARS_NUMBER; i++) {
            result = vRep.simxGetObjectHandle(clientID, VREP_ROBOT_BASE_SONAR_NAME + i, sonarsHandles[i], remoteApi.simx_opmode_blocking);
            runningStateSonars[i] = result == remoteApi.simx_return_ok;
            runAtLeastOneSonar |= runningStateSonars[i];
        }
    }

    private void firstReadAllSensors()
    {
        int result;
        // right wheel
        if (runRightWheelEncoder)
        {
            result = vRep.simxGetJointPosition(clientID, rightWheelHandle.getValue(), robotRightJointPosition, remoteApi.simx_opmode_streaming);
            runRightWheelEncoder = result != remoteApi.simx_return_remote_error_flag;
        }
        currentRightJointPosition = robotRightJointPosition.getValue();
        totalRightJointPosition = 0;

        // left wheel
        if (runLeftWheelEncoder)
        {
            result = vRep.simxGetJointPosition(clientID, leftWheelHandle.getValue(), robotLeftJointPosition, remoteApi.simx_opmode_streaming);
            runLeftWheelEncoder = result != remoteApi.simx_return_remote_error_flag;
        }
        currentLeftJointPosition = robotLeftJointPosition.getValue();
        totalLeftJointPosition = 0;
        runWheelEncoder = runRightWheelEncoder && runLeftWheelEncoder;
        runMotion = runWheelEncoder;

        // camera
        if (runCamera)
        {
            result = vRep.simxGetVisionSensorImage(clientID, cameraHandle.getValue(), resolution, image, 2, remoteApi.simx_opmode_streaming);
            runCamera = result != remoteApi.simx_return_remote_error_flag;
        }

        // gps
        if (runGPS)
        {
            FloatWA position = new FloatWA(3);
            result = vRep.simxGetObjectPosition(clientID, gpsHandle.getValue(), -1, position, remoteApi.simx_opmode_streaming);
            runGPS = result != remoteApi.simx_return_remote_error_flag;
        }

        // sonars
        BoolW detectionState = new BoolW(false);
        FloatWA detectedPoint = new FloatWA(1); //Coordinates relatives to the sensor's frame
        IntW detectedObjectHandle = new IntW(1);
        FloatWA detectedSurfaceNormalVector = new FloatWA(1);
        for (int i = 0; i < SONARS_NUMBER; i++) {
            if (runningStateSonars[i])
            {
                result = vRep.simxReadProximitySensor(clientID, sonarsHandles[i].getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_streaming);
                runningStateSonars[i] = result != remoteApi.simx_return_remote_error_flag;
                runAtLeastOneSonar |= runningStateSonars[i];
            }
        }
    }

    private void initTimers(boolean start)
    {
        if (motionTimer == null)
        {
            motionTimer = new Timer();
            motionTimer.setSec(1);
            if (start) motionTimer.start();
        }
        if (batteryTimer == null)
        {
            batteryTimer = new Timer();
            batteryTimer.setSec(MAX_BATT_TIME);
            if (start) batteryTimer.start();
        }
    }

    private void initThreads(boolean start)
    {
        if (updateSensorThread == null)
        {
            updateSensorThread = new Thread(this::updateSensorThreadRunnable);
            if (start) updateSensorThread.start();
        }
        if (mainThread == null)
        {
            mainThread = new Thread(this::mainThreadRunnable);
            if (start) mainThread.start();
        }
        if (updateUIThread == null)
        {
            updateUIThread = new Thread(this::updateUIThreadRunnable);
            if (start) updateUIThread.start();
        }
    }

    private void updateUIThreadRunnable()
    {
        System.out.println("start update UI");
        while (running) {
            Platform.runLater(new Runnable() {
                @Override
                public void run() {
                    if (running)
                    {
                        if (runGPS) {
                            lblGpsX.setText("X: " + Math.round(gpsValues[0]*100.)/100.);
                            lblGpsY.setText("Y: " + Math.round(gpsValues[1]*100.)/100.);
                            lblGpsZ.setText("Z: " + Math.round(gpsValues[2]*100.)/100.);
                        }
                        if (runAtLeastOneSonar) {
                            lblSensor0.setText(" " + sonarValues[0] + "m");
                            lblSensor1.setText(" " + sonarValues[1] + "m");
                            lblSensor2.setText(" " + sonarValues[2] + "m");
                            lblSensor3.setText(" " + sonarValues[3] + "m");
                            lblSensor4.setText(" " + sonarValues[4] + "m");
                            lblSensor5.setText(" " + sonarValues[5] + "m");
                            lblSensor5.setText(" " + sonarValues[5] + "m");
                        }
                        if (runWheelEncoder) {
                            lblRightWheel.setText(" Right: " + encoderValues[0]);
                            lblLeftWheel.setText(" Left: " + encoderValues[1]);
                        }
                        updateLeds();
                        //lblState.setText(currentState == requestState ? currentState.name() : currentState.name() + " (→ " + requestState.name() +")");
                        lblState.setText(currentState.name());

                        if (runCamera) {
                            if (bufferedImageRGB != null)
                            {
                                while(isWrittingTheImage);
                                isComputingTheImage = true;
                                IplImage rgbSrc = bufferedImageToIplImage(bufferedImageRGB);
                                IplImage graySrc = bufferedImageToIplImage(bufferedImageToGray(bufferedImageRGB));

                                if (rgbSrc != null && graySrc != null)
                                {
                                    CvPoint matchLocation = findMarker(graySrc);
                                    if (matchLocation != null) {
                                        drawRectangle(rgbSrc, matchLocation);
                                        displayBufferedImage(iplImageToBufferedImage(rgbSrc));
                                    }
                                    else
                                    {
                                        displayBufferedImage(bufferedImageRGB);
                                    }
                                }
                                isComputingTheImage = false;
                            }
                        }
                    }
                }
            });
            Delay.ms(33);
        }
        System.out.println("stop update UI");
    }

    private void updateSensorThreadRunnable()
    {
        System.out.println("start update sensors");
        while (running)
        {
            if (runGPS) {
                gpsValues = readGPS();
            }

            if (!running) break;
            if (runAtLeastOneSonar) {
                sonarValues = readSonars();
            }

            if (!running) break;
            if (runWheelEncoder) {
                encoderValues[0] = readLeftWheelEnc();
                encoderValues[1] = readRightWheelEnc();

                if (!runMotion) {
                    move(0);
                }
            }

            if (!running) break;
            if (runCamera) {
                readCamera();
            }

            if (!running) break;

            Delay.ms(1);
        }
        System.out.println("stop update sensors");
    }

    private void mainThreadRunnable()
    {
        System.out.println("start main");
        while (running) {
            requestAutomate();
            stateAutomate();
            teleoperate();
            if (!getBatteryState()) {
                System.err.println("Error: Robot out of battery...");
                isPowerEmpty = true;
                break;
            }
            Delay.ms(1);
        }
        if (isPowerEmpty)
        {
            setDisableAllMotionButtons(true);
            kill();
            resetUIGeneral();
        }
        System.out.println("stop main");
    }
    //endregion

    //region Automate Methods
    private void requestAutomate() {
        if (requestState != currentState) {
            switch (requestState){
                case None:
                    currentState = States.None;
                    break;
                case Initialize:
                    currentState = States.Initialize;
                    break;
                case Clean:
                    currentState = States.Clean;
                    break;
                case Avoid:
                    currentState = States.Avoid;
                    break;
                case Wander:
                    currentState = States.Wander;
                    break;
                case Track:
                    currentState = States.Track;
                    break;
            }
        }
    }

    private void stateAutomate() {
        switch (currentState) {
            case None:
                break;
            case Initialize:
                requestState = States.Clean;
                break;
            case Clean:
                if (checkBatteryLowRoutine()) {
                    break;
                }
                if (isObstacleDetected()) {
                    dir = MotionDirections.Stop;
                    requestState = States.Avoid;
                    break;
                }
                dir = MotionDirections.Forward;
                break;
            case Avoid:
                if (checkBatteryLowRoutine()) {
                    break;
                }
                if (hasOrientationAim) {
                    double orientation = getGPSRz();
                    double deltaOrientation = orientationAim - orientation;
                    if (deltaOrientation > 180) {
                        deltaOrientation -= 360;
                    }
                    else if (deltaOrientation < -180) {
                        deltaOrientation += 360;
                    }

                    if (deltaOrientation > 3) {
                        dir = MotionDirections.Left;
                    }
                    else if (deltaOrientation < -3) {
                        dir = MotionDirections.Right;
                    }
                    else {
                        dir = MotionDirections.Stop;
                        hasOrientationAim = false;
                        requestState = States.Clean;
                    }
                }
                else {
                    dir = MotionDirections.Stop;
                    orientationAim = generateRotationAim();
                    hasOrientationAim = true;
                }
                break;
            case Wander:
                break;
            case Track:
                break;
        }
    }

    private boolean checkBatteryLowRoutine() {
        boolean bLow = isBatteryLow();
        if (bLow) {
            System.out.println("Battery Low !");
            requestState = States.Wander;
            dir = MotionDirections.Stop;
        }
        return bLow;
    }
    //endregion

    //endregion
}
