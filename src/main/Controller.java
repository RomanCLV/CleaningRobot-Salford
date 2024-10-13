package main;

import coppelia.*;
import com.googlecode.javacv.cpp.opencv_core;

import javafx.fxml.FXML;
import javafx.application.Platform;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import javafx.scene.control.Label;
import javafx.scene.canvas.Canvas;
import javafx.scene.control.Button;
import javafx.scene.image.PixelWriter;
import javafx.concurrent.Task;

import utils.*;

import java.awt.*;
import java.awt.image.BufferedImage;

import static com.googlecode.javacv.cpp.opencv_core.*;
import static com.googlecode.javacv.cpp.opencv_highgui.cvDestroyAllWindows;
import static com.googlecode.javacv.cpp.opencv_highgui.cvDestroyWindow;
import static com.googlecode.javacv.cpp.opencv_highgui.cvLoadImage;
import static com.googlecode.javacv.cpp.opencv_imgproc.cvMatchTemplate;
import static com.googlecode.javacv.cpp.opencv_imgproc.CV_TM_CCOEFF_NORMED;

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
    PixelWriter pw;
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
    private Label lbl;
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

    private String defaultButtonStyle;

    //endregion

    //region Sensors variables declaration
    private Color[][] imageCamera;
    private double[] gpsValues = new double[3];
    private double[] sonarValues = new double[6];
    private final double[] encoderValues = new double[2];

    private boolean runGPS = true;
    private boolean runCamera = true;
    private boolean runMotion = true;
    private boolean runSensors = true;
    private boolean runWheelEncoder = true;
    //endregion

    //region Robot variables declaration
    private final IntW cameraHandle = new IntW(1);
    private final IntW leftWheelHandle = new IntW(1);
    private final IntW rightWheelHandle = new IntW(1);

    private volatile boolean running = false;
    private boolean firstCameraRead = true;
    private final boolean[] firstSensorRead = new boolean[] { true, true, true, true, true, true };
    private boolean firstLeftWheelCall = true;
    private boolean firstRightWheelCall = true;

    private MotionDirections dir = MotionDirections.None; // Direction.
    private final int vel = 5;    // Velocity.
    //endregion

    //region Camera variables declaration
    private double targetMinScore = 0.0;
    private double targetMaxScore = 0.0;

    private int resolutionCamera = 256;
    private CvPoint point = new CvPoint();
    private IntWA resolution = new IntWA(1);
    private CharWA image = new CharWA(resolutionCamera * resolutionCamera * 3);
    private char[] imageArray = new char[resolutionCamera * resolutionCamera * 3];
    private Color[][] colorMatrix = new Color[resolutionCamera][resolutionCamera];
    private BufferedImage bufferedImage = new BufferedImage(resolutionCamera, resolutionCamera, BufferedImage.TYPE_INT_RGB);

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
    private final String VREP_ROBOT_RIGHT_WHEEL_NAME = "JointRightWheel";
    private final String VREP_ROBOT_VISION_SENSOR_NAME = "Vision_sensor";
    private final String CAMERA_WND_NAME = "Camera";
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
    States currentState = States.None;
    States requestState = States.Initialize;
    //endregion

    private Stage primaryStage = null;

    //endregion

    //region Methods
    //region Stage & close events method
    @Override
    public void setStage(Stage stage) {
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

    private void handleCloseEvent() {
        kill();
    }
    //endregion

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
    //endregion

    //region Wheel methods
    private double readRightWheelEnc() {
        if (firstRightWheelCall) {
            vRep.simxGetJointPosition(clientID, rightWheelHandle.getValue(), robotRightJointPosition, remoteApi.simx_opmode_streaming);
            currentRightJointPosition = robotRightJointPosition.getValue();
            firstRightWheelCall = false;
            totalRightJointPosition = 0;
        } else {
            vRep.simxGetJointPosition(clientID, rightWheelHandle.getValue(), robotRightJointPosition, remoteApi.simx_opmode_buffer);
            currentRightJointPosition = robotRightJointPosition.getValue();
            dxRight = getAngleMinusAlpha(currentRightJointPosition, previousRightJointPosition);
            totalRightJointPosition += dxRight;
        }
        previousRightJointPosition = currentRightJointPosition;
        return (Math.round((totalRightJointPosition / (2 * Math.PI)) * 100d) / 100d);
    }

    private double readLeftWheelEnc() {
        if (firstLeftWheelCall) {
            vRep.simxGetJointPosition(clientID, leftWheelHandle.getValue(), robotLeftJointPosition, remoteApi.simx_opmode_streaming);
            currentLeftJointPosition = robotLeftJointPosition.getValue();
            firstLeftWheelCall = false;
            totalLeftJointPosition = 0;
        } else {
            vRep.simxGetJointPosition(clientID, leftWheelHandle.getValue(), robotLeftJointPosition, remoteApi.simx_opmode_buffer);
            currentLeftJointPosition = robotLeftJointPosition.getValue();
            dxLeft = getAngleMinusAlpha(currentLeftJointPosition, previousLeftJointPosition);
            totalLeftJointPosition += dxLeft;
        }
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
        return (2);
    }
    //endregion

    //region GPS methods
    private double[] readGPS() {
        IntW baseHandle = new IntW(1);
        FloatWA position = new FloatWA(3);
        vRep.simxGetObjectHandle(clientID, VREP_ROBOT_NAME, baseHandle, remoteApi.simx_opmode_streaming);
        vRep.simxGetObjectPosition(clientID, baseHandle.getValue(), -1, position, remoteApi.simx_opmode_streaming);
        double[] positions = new double[position.getArray().length];

        positions[0] = Math.round((VREP_SENS_AXIS_X * (double) position.getArray()[0] + VREP_START_POS_X) * 100.0) / 100.0;
        positions[1] = Math.round((VREP_SENS_AXIS_Y * (double) position.getArray()[1] + VREP_START_POS_Y) * 100.0) / 100.0;
        positions[2] = Math.round((double) position.getArray()[2] * 100.0) / 100.0;
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

    private int getGPSNo() {
        return (3);
    }
    //endregion

    //region Ultrasonic methods
    private double readSonarRange(int sensor) {
        BoolW detectionState = new BoolW(false);
        FloatWA detectedPoint = new FloatWA(1); //Coordinates relatives to the sensor's frame
        IntW detectedObjectHandle = new IntW(1);
        FloatWA detectedSurfaceNormalVector = new FloatWA(1);

        IntW sensorHandle = new IntW(1);
        String objectName = "Proximity_sensor" + sensor;
        vRep.simxGetObjectHandle(clientID, objectName, sensorHandle, remoteApi.simx_opmode_blocking);
        if (firstSensorRead[sensor])
        {
            vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_streaming);
            firstSensorRead[sensor] = false;
        }
        else {
            vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_buffer);
        }

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
            sonarValues[i] = readSonarRange(i);
        return (sonarValues);
    }

    private double[] getSonarRanges() {
        return (sonarValues);
    }

    private double getSonarRange(int sensor) {
        return (sonarValues[sensor]);
    }

    private int getSonarNo() {
        return (sonarValues.length);
    }
    //endregion

    //region Camera methods
    private Color[][] readCamera() {
        if (firstCameraRead) {
            vRep.simxGetVisionSensorImage(clientID, cameraHandle.getValue(), resolution, image, 2, remoteApi.simx_opmode_streaming);
            firstCameraRead = false;
        }
        else {
            vRep.simxGetVisionSensorImage(clientID, cameraHandle.getValue(), resolution, image, 2, remoteApi.simx_opmode_buffer);
        }
        return (imageToColor(image));
    }

    private Color[][] imageToColor(CharWA image) {
        imageArray = image.getArray();
        int index = 0;
        int r, g, b;
        Color color;

        for (int i = 0; i < resolutionCamera; i++)
            for (int j = 0; j < resolutionCamera; j++) {
                // Retrieve the RGB Values:
                r = (int) imageArray[index];
                g = (int) imageArray[index + 1];
                b = (int) imageArray[index + 2];
                color = new Color(r, g, b);
                colorMatrix[i][j] = color;

                bufferedImage.setRGB(i, j, new Color(r, g, b).getRGB());
                index += 3;
            }
        return (colorMatrix);
    }

    private int getGrayscale(BufferedImage img, int x, int y) {
        Color c = new Color(img.getRGB(x, y));
        int r = (int) (c.getRed() * 0.299);
        int g = (int) (c.getGreen() * 0.587);
        int b = (int) (c.getBlue() * 0.114);
        return ((r + g + b));
    }

    private static BufferedImage rotate(BufferedImage bimg, double angle, boolean color) {
        // [1]Get image dimensions:
        int imageType = -1;
        int w = bimg.getWidth();
        int h = bimg.getHeight();

        // [2]Select image type: color/grayscale
        if (color) imageType = bimg.getType();
        else imageType = BufferedImage.TYPE_BYTE_GRAY;

        // [3]Rotate and draw:
        BufferedImage rotated = new BufferedImage(w, h, imageType);
        Graphics2D graphic = rotated.createGraphics();
        graphic.rotate(Math.toRadians(angle), (double)w / 2, (double)h / 2);
        graphic.drawImage(bimg, null, 0, 0);
        graphic.dispose();

        return (rotated);
    }

    private BufferedImage getImage() {
        return (rotate(bufferedImage, -90, false));
    }

    private int getImageWidth() {
        return (bufferedImage.getWidth());
    }

    private int getImageHeight() {
        return (bufferedImage.getHeight());
    }

    private int getImagePixel(int x, int y) {
        return (getGrayscale(bufferedImage, x, y));
    }

    private void setImagePixel(int x, int y, int rgb) {
        bufferedImage.setRGB(x, y, rgb + (rgb << 8) + (rgb << 16));
    }

    private int getTargetX() {
        return (point.x());
    }

    private int getTargetY() {
        return (point.y());
    }

    private double getTargetMinScore() {
        return (targetMinScore);
    }

    private double getTargetMaxScore() {
        return (targetMaxScore);
    }

    private void displayImage() {
        ImageViewer.display(getImage());
    }

    private void templateMatchingCV(BufferedImage image) {
        // [1]Load source and template image files:
        IplImage src = IplImage.create(image.getWidth(), image.getHeight(), opencv_core.IPL_DEPTH_8U, 1);
        src.copyFrom(image);


        // [2]The Correlation Image Result:
        IplImage result = cvCreateImage(cvSize(src.width() - markerImage.width() + 1, src.height() - markerImage.height() + 1), IPL_DEPTH_32F, 1);

        // [3]Select a function template-match method:
        //cvMatchTemplate(src, tmp, result, CV_TM_CCORR_NORMED);  //1*
        cvMatchTemplate(src, markerImage, result, CV_TM_CCOEFF_NORMED);   //5*

        double min_val[] = new double[2];
        double max_val[] = new double[2];

        // [4]Max and Min correlation point locations:
        CvPoint minLoc = new CvPoint();
        CvPoint maxLoc = new CvPoint();

        // [5]Compute and print min-max value locations:
        cvMinMaxLoc(result, min_val, max_val, minLoc, maxLoc, null);
        targetMinScore = min_val[0]; // Min Score.
        targetMaxScore = max_val[0]; // Max Score.
        //System.out.println("Min: " + targetMin);
        //System.out.println("Max: " + targetMax);

        // [6]Mark at point the image template coordinates:
        point.x(maxLoc.x() + markerImage.width());
        point.y(maxLoc.y() + markerImage.height());

        // [7]Draw the rectangle result in source image:
        cvRectangle(src, maxLoc, point, CvScalar.GRAY, 2, 8, 0);

        // [8]Display the image:
        ImageViewer.display(src.getBufferedImage());
    }
    //endregion

    //region Motion Methods
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

    private void teleoperate(MotionDirections dir, int vel) {
        switch (dir) {
            case Stop:
                move(0);
                this.dir = MotionDirections.None;
                break;
            case Forward:
                move(+vel, 3000);
                break;
            case Backward:
                move(-vel, 3000);
                break;
            case Right:
                turnSpot(+(float)(vel / 2), 3000);
                break;
            case Left:
                turnSpot(-(float)(vel / 2), 3000);
                break;
        }
    }
    //endregion

    //region Generic Methods
    @Override
    public void init() {
        defaultButtonStyle = btnForward.getStyle();
        markerImage = cvLoadImage("data/images/marker.jpg", 0);
        vRep = new remoteApi();
        vRep.simxFinish(-1); // close all simulations... to be sure
    }

    public void btnConnectPressed() {
        if (clientID == -1) {
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
        else {
            btnConnect.setText("Disconnecting...");
            btnConnect.setStyle(defaultButtonStyle);
            btnConnect.setDisable(true);

            // creation of a task to disconnect properly and not block the UI
            Task<Void> task = new Task<Void>() {
                @Override
                protected Void call() throws Exception {
                    kill();
                    return null;
                }

                @Override
                protected void succeeded() {
                    btnConnect.setDisable(false);
                    btnConnect.setText("Connect");
                }

                @Override
                protected void failed() {
                    btnConnect.setDisable(false);
                    btnConnect.setText("Connect");
                }
            };

            // run the task asynchronously to not block the UI
            new Thread(task).start();
        }
    }


    private boolean connectToVrep()
    {
        clientID = vRep.simxStart(VREP_IPV4, VREP_PORT, true, true, 5000, 5);
        return clientID != -1;
    }

    private void kill()
    {
        killTimers();
        killThreads();
        disconnectToVrep();
        cvDestroyWindow(CAMERA_WND_NAME);
        cvDestroyAllWindows();
        ImageViewer.dispose();
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
        while (updateSensorThread.isAlive() ||
                updateUIThread.isAlive() ||
                mainThread.isAlive())
        {
            System.out.println("wait threads end");
        }
        System.out.println("All threads stopped");
        updateUIThread = null;
        updateSensorThread = null;
        mainThread = null;
    }

    private void disconnectToVrep()
    {
        //vRep.simxStopSimulation(clientID, remoteApi.simx_opmode_blocking);
        vRep.simxFinish(clientID);
        clientID = -1;
    }

    private boolean setup() {
        boolean result = true;
        if (running)
        {
            result = false;
        }
        else
        {

            vRep.simxGetObjectHandle(clientID, VREP_ROBOT_LEFT_WHEEL_NAME, leftWheelHandle, remoteApi.simx_opmode_blocking);
            vRep.simxGetObjectHandle(clientID, VREP_ROBOT_RIGHT_WHEEL_NAME, rightWheelHandle, remoteApi.simx_opmode_blocking);
            vRep.simxGetObjectHandle(clientID, VREP_ROBOT_VISION_SENSOR_NAME, cameraHandle, remoteApi.simx_opmode_blocking);

            pw = canvasCamera.getGraphicsContext2D().getPixelWriter();
            ImageViewer.open(resolutionCamera, resolutionCamera, CAMERA_WND_NAME);

            dir = MotionDirections.None;
            running = true;

            initTimers(true);
            initThreads(true);
        }
        return result;
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
                            lblGpsX.setText("X: " + gpsValues[0]);
                            lblGpsY.setText("Y: " + gpsValues[1]);
                            lblGpsZ.setText("Z: " + gpsValues[2]);
                        }
                        if (runSensors) {
                            lblSensor0.setText(sonarValues[0] + "m");
                            lblSensor1.setText(sonarValues[1] + "m");
                            lblSensor2.setText(sonarValues[2] + "m");
                            lblSensor3.setText(sonarValues[3] + "m");
                            lblSensor4.setText(sonarValues[4] + "m");
                            lblSensor5.setText(sonarValues[5] + "m");
                            lblSensor5.setText(sonarValues[5] + "m");
                        }
                        if (runWheelEncoder) {
                            lblRightWheel.setText("Right : " + encoderValues[0]);
                            lblLeftWheel.setText("Left : " + encoderValues[1]);
                        }
                        if (runCamera) {
                            templateMatchingCV(getImage());
                        }
                    }
                }
            });
            Delay.ms(20);
        }
        System.out.println("stop update UI");
    }

    private void updateSensorThreadRunnable()
    {
        System.out.println("start update sensors");

        setBatteryTime(20); // set battery for 20 minutes
        while (running)
        {
            if (runGPS) {
                gpsValues = readGPS();
            }

            if (!running) break;
            if (runSensors) {
                sonarValues = readSonars();
            }

            if (!running) break;
            if (runWheelEncoder) {
                encoderValues[0] = readLeftWheelEnc();
                encoderValues[1] = readRightWheelEnc();
            }

            if (!running) break;
            if (runCamera) {
                imageCamera = readCamera();
            }

            if (!running) break;
            if (runMotion) {
                teleoperate(dir, vel);
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

            if (!getBatteryState()) {
                System.err.println("Error: Robot out of battery...");
                move(0, 1000);
                kill();
            }
            Delay.ms(1);
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
        switch (currentState){
            case None:
                break;
            case Initialize:
                requestState = States.Clean ;
                break;
            case Clean:
                break;
            case Avoid:
                break;
            case Wander:
                break;
            case Track:
                break;
        }
    }

    //endregion

    //endregion
}
