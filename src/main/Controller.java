package main;

import coppelia.*;
import com.googlecode.javacv.cpp.opencv_core;

import javafx.fxml.FXML;
import javafx.application.Platform;
import javafx.scene.control.Label;
import javafx.scene.canvas.Canvas;
import javafx.scene.control.Button;
import javafx.scene.image.PixelWriter;

import utils.Delay;
import utils.ImageViewer;
import utils.Timer;
import utils.Utils;

import java.awt.*;
import java.util.Arrays;
import java.awt.image.BufferedImage;
import java.util.Collections;

import static com.googlecode.javacv.cpp.opencv_core.*;
import static com.googlecode.javacv.cpp.opencv_highgui.cvLoadImage;
import static com.googlecode.javacv.cpp.opencv_imgproc.cvMatchTemplate;
import static com.googlecode.javacv.cpp.opencv_imgproc.CV_TM_CCOEFF_NORMED;

/**
 * Created by Theo Theodoridis. | Updated by Roman CLAVIER.
 * Class    : Controller
 * Version  : v1.0 | v1.1
 * Date     : © Copyright 01/11/2020 | 08/10/2024
 * User     : ttheod
 * email    : ttheod@gmail.com | roman.clavier.2001@gmail.com
 * Comments : None. | I'm a big boss
 **/

public class Controller {
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

    private boolean running = false;
    private boolean firstCameraRead = true;
    private final boolean[] firstSensorRead = new boolean[] { true, true, true, true, true, true };
    private boolean firstLeftWheelCall = true;
    private boolean firstRightWheelCall = true;

    private char dir = 's'; // Direction.
    private int vel = 5;    // Velocity.
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
    //endregion

    //region Wheel encoders variables declaration
    /**
     * Wheel encoders:
     **/

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

    //region GPS variables declaration
    /**
     * GPS:
     **/

    public static final double CHARGER_XCOORD = 1.78;  // The charger X coordinate.
    public static final double CHARGER_YCOORD = -0.78; // The charger Y coordinate.
    public static final double MAX_GPS_DIST = 5.0;   // The max Euclidean distance from the charger.
    //endregion

    //region V-rep communication variables declaration
    private final remoteApi vRep = new remoteApi();
    private int clientID = -1;
    //endregion

    //region Timers variables declaration
    /**
     * Timers:
     **/

    private int MAX_BATT_TIME = 60 * 20; // Default 20 mins battery time.
    private final int MAX_BATT_VOLT = 12;      // volts.

    private final Timer motionTimer = new Timer();
    private final Timer batteryTimer = new Timer();
    //endregion
    //endregion

    //region Methods
    //region Battery methods
    public int getBatteryTime() {
        return (batteryTimer.getSec());
    }

    public void setBatteryTime(int min) {
        MAX_BATT_TIME = 60 * min;
        motionTimer.setSec(MAX_BATT_TIME);
        motionTimer.restart();
    }

    public double getBatteryCapacity() {
        double v = (double) MAX_BATT_VOLT - Utils.map(batteryTimer.getSec(), 0, (double) MAX_BATT_TIME, 0, (double) MAX_BATT_VOLT);
        return (Math.max(v, 0.0));
    }

    public double getBatteryPercentage() {
        double v = getBatteryCapacity();

        if ((v >= 9.6) && (v <= 12)) return (100.0);
        else if ((v >= 7.2) && (v < 9.6)) return (80.0);
        else if ((v >= 4.8) && (v < 7.2)) return (60.0);
        else if ((v >= 2.4) && (v < 4.8)) return (40.0);
        else if ((v > 1.0) && (v < 2.4)) return (20.0);
        else
            return (0.0);
    }

    public boolean getBatteryState() {
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

    public double getLeftWheelEnc() {
        return (encoderValues[0]);
    }

    public double getRightWheelEnc() {
        return (encoderValues[1]);
    }

    public int getEncoderNo() {
        return (2);
    }
    //endregion

    //region GPS methods
    public double[] readGPS() {
        IntW baseHandle = new IntW(1);
        FloatWA position = new FloatWA(3);
        vRep.simxGetObjectHandle(clientID, "Roomba", baseHandle, remoteApi.simx_opmode_streaming);
        vRep.simxGetObjectPosition(clientID, baseHandle.getValue(), -1, position, remoteApi.simx_opmode_streaming);
        double[] positions = new double[position.getArray().length];

        positions[0] = Math.round((-(double) position.getArray()[0] + 2.130) * 100.0) / 100.0;
        positions[1] = Math.round((-(double) position.getArray()[1] - 0.775) * 100.0) / 100.0;
        positions[2] = Math.round((double) position.getArray()[2] * 100.0) / 100.0;
        return (positions);
    }

    public double getGPSX() {
        return (gpsValues[0]);
    }

    public double getGPSY() {
        return (gpsValues[1]);
    }

    public double getGPSZ() {
        return (gpsValues[2]);
    }

    public int getGPSNo() {
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
        if (firstSensorRead[sensor]) {
            vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_streaming);
            firstSensorRead[sensor] = false;
        } else {
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

    public double getSonarRange(int sensor) {
        return (sonarValues[sensor]);
    }

    public int getSonarNo() {
        return (sonarValues.length);
    }
    //endregion

    //region Camera methods
    private Color[][] readCamera() {
        if (firstCameraRead) {
            vRep.simxGetVisionSensorImage(clientID, cameraHandle.getValue(), resolution, image, 2, remoteApi.simx_opmode_streaming);
            firstCameraRead = false;
        } else
            vRep.simxGetVisionSensorImage(clientID, cameraHandle.getValue(), resolution, image, 2, remoteApi.simx_opmode_buffer);
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

    public BufferedImage getImage() {
        return (rotate(bufferedImage, -90, false));
    }

    public int getImageWidth() {
        return (bufferedImage.getWidth());
    }

    public int getImageHeight() {
        return (bufferedImage.getHeight());
    }

    public int getImagePixel(int x, int y) {
        return (getGrayscale(bufferedImage, x, y));
    }

    public void setImagePixel(int x, int y, int rgb) {
        bufferedImage.setRGB(x, y, rgb + (rgb << 8) + (rgb << 16));
    }

    public int getTargetX() {
        return (point.x());
    }

    public int getTargetY() {
        return (point.y());
    }

    public double getTargetMinScore() {
        return (targetMinScore);
    }

    public double getTargetMaxScore() {
        return (targetMaxScore);
    }

    public void displayImage() {
        ImageViewer.display(getImage());
    }

    public void templateMatchingCV(BufferedImage image) {
        // [1]Load source and template image files:
        IplImage src = IplImage.create(image.getWidth(), image.getHeight(), opencv_core.IPL_DEPTH_8U, 1);
        src.copyFrom(image);
        IplImage tmp = cvLoadImage("data/images/marker.jpg", 0);

        // [2]The Correlation Image Result:
        IplImage result = cvCreateImage(cvSize(src.width() - tmp.width() + 1, src.height() - tmp.height() + 1), IPL_DEPTH_32F, 1);

        // [3]Select a function template-match method:
        //cvMatchTemplate(src, tmp, result, CV_TM_CCORR_NORMED);  //1*
        cvMatchTemplate(src, tmp, result, CV_TM_CCOEFF_NORMED);   //5*

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
        point.x(maxLoc.x() + tmp.width());
        point.y(maxLoc.y() + tmp.height());

        // [7]Draw the rectangle result in source image:
        cvRectangle(src, maxLoc, point, CvScalar.GRAY, 2, 8, 0);

        // [8]Display the image:
        ImageViewer.display(src.getBufferedImage());
    }
    //endregion

    //region Motion Methods
    public void forward() {
        resetButtonsStyle();
        btnForward.setStyle("-fx-background-color: #7FFF00; ");
        dir = 'f';
    }

    public void backward() {
        resetButtonsStyle();
        btnBack.setStyle("-fx-background-color: #7FFF00; ");
        dir = 'b';
    }

    public void left() {
        resetButtonsStyle();
        btnLeft.setStyle("-fx-background-color: #7FFF00; ");
        dir = 'l';
    }

    public void right() {
        resetButtonsStyle();
        btnRight.setStyle("-fx-background-color: #7FFF00; ");
        dir = 'r';
    }

    public void stop() {
        resetButtonsStyle();
        btnStop.setStyle("-fx-background-color: #7FFF00; ");
        dir = 's';
    }

    private void resetButtonsStyle() {
        btnRight.setStyle(defaultButtonStyle);
        btnStop.setStyle(defaultButtonStyle);
        btnLeft.setStyle(defaultButtonStyle);
        btnForward.setStyle(defaultButtonStyle);
        btnBack.setStyle(defaultButtonStyle);
    }

    public void setVel(float lVel, float rVel) {
        vRep.simxSetJointTargetVelocity(clientID, leftWheelHandle.getValue(), lVel, remoteApi.simx_opmode_oneshot);
        vRep.simxSetJointTargetVelocity(clientID, rightWheelHandle.getValue(), rVel, remoteApi.simx_opmode_oneshot);
    }

    public void move(float vel) {
        setVel(vel, vel);
    }

    public void move(float vel, int time) {
        motionTimer.setMs(time);
        motionTimer.restart();
        while (motionTimer.getState()) {
            move(vel);
        }
        stop();
    }

    public void turnSpot(float vel) {
        setVel(vel, -vel);
    }

    public void turnSpot(float vel, int time) {
        motionTimer.setMs(time);
        motionTimer.restart();
        while (motionTimer.getState()) {
            turnSpot(vel);
        }
        stop();
    }

    public void turnSharp(float vel) {
        if (vel > 0) setVel(vel, 0);
        else setVel(0, -vel);
    }

    public void turnSharp(float vel, int time) {
        motionTimer.setMs(time);
        motionTimer.restart();
        while (motionTimer.getState()) {
            turnSharp(vel);
        }
        stop();
    }

    public void turnSmooth(float vel) {
        if (vel > 0) setVel(vel, vel / 2);
        else setVel(-vel / 2, -vel);
    }

    public void turnSmooth(float vel, int time) {
        motionTimer.setMs(time);
        motionTimer.restart();
        while (motionTimer.getState()) {
            turnSmooth(vel);
        }
        stop();
    }

    public void teleoperate(char dir, int vel) {
        switch (dir) {
            case 's':
                move(vel = 0);
                break;
            case 'f':
                move(+vel);
                break;
            case 'b':
                move(-vel);
                break;
            case 'r':
                turnSpot(+(float)(vel / 2));
                break;
            case 'l':
                turnSpot(-(float)(vel / 2));
                break;
        }
    }
    //endregion

    //region Generic Methods
    public void connectToVrep() {
        clientID = vRep.simxStart("127.0.0.1", 20001, true, true, 5000, 5);
        if (clientID == -1)
        {
            running = false;
            btnConnect.setText("Failed");
            btnConnect.setStyle("-fx-background-color: #FF0000; ");
            vRep.simxFinish(clientID);
        }
        else {
            btnConnect.setStyle("-fx-background-color: #7FFF00; ");
            btnConnect.setText("Connected");
            running = true;
            setup();
        }
    }

    public void setup() {
        vRep.simxGetObjectHandle(clientID, "JointLeftWheel", leftWheelHandle, remoteApi.simx_opmode_blocking);
        vRep.simxGetObjectHandle(clientID, "JointRightWheel", rightWheelHandle, remoteApi.simx_opmode_blocking);
        vRep.simxGetObjectHandle(clientID, "Vision_sensor", cameraHandle, remoteApi.simx_opmode_blocking);

        defaultButtonStyle = btnForward.getStyle();
        pw = canvasCamera.getGraphicsContext2D().getPixelWriter();
        ImageViewer.open(resolutionCamera, resolutionCamera, "Camera");

        motionTimer.setSec(1);
        batteryTimer.setSec(MAX_BATT_TIME);
        motionTimer.start();
        batteryTimer.start();

        updateSensorThread.start();
        mainThread.start();
        updateUIThread.start();
    }

    private final Thread updateUIThread = new Thread(this::updateUIThreadRunnable);

    public void updateUIThreadRunnable() {
        while (running) {
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
            Delay.ms(20);
        }
    }

    private final Thread updateSensorThread = new Thread(this::updateSensorThreadRunnable);

    private void updateSensorThreadRunnable() {
        setBatteryTime(20); // set battery for 20 minutes
        while (running) {
            if (runGPS) {
                gpsValues = readGPS();
            }
            if (runSensors) {
                sonarValues = readSonars();
            }
            if (runWheelEncoder) {
                encoderValues[0] = readLeftWheelEnc();
                encoderValues[1] = readRightWheelEnc();
            }
            if (runCamera) {
                imageCamera = readCamera();
            }
            if (runMotion) {
                teleoperate(dir, vel);
            }
            // [3]Update battery:
            if (!getBatteryState()) {
                System.err.println("Error: Robot out of battery...");
                move(0, 1000);
                running = false;
            }
            Delay.ms(1);
        }
    }

    private final Thread mainThread = new Thread(this::mainThreadRunnable);

    private void mainThreadRunnable() {
        while (running) {
            templateMatchingCV(getImage());
            //        Integer[] priority = new Integer[2];
//        double cam = getTargetMaxScore();                                                      // Target horizontal detection (pixels).
//        double bat = getBatteryCapacity();                                                     // Battery capacity (volts).
//        double snr = Arrays.stream(getSonarRanges()).min().getAsDouble();                      // Min sonar range radius (meters).
//        double gps = Utils.getEuclidean(CHARGER_XCOORD, getGPSY(), getGPSX(), CHARGER_YCOORD); // GPS distance from charger (meters?).
//        double[] sensors = new double[]{bat, snr, cam, gps};                                   // Sensor vector.
//
//        double[] sonarData = new double[]
//                {
//                        getSonarRange(0),
//                        getSonarRange(1),
//                        getSonarRange(2),
//                        getSonarRange(3),
//                        getSonarRange(4),
//                        getSonarRange(5)
//                };

            //avoid();

            //double leftMinSonarRadius  = Arrays.stream(new double[]{getSonarRange(0), getSonarRange(1), getSonarRange(2)}).min().getAsDouble();
            //double rightMinSonarRadius = Arrays.stream(new double[]{getSonarRange(3), getSonarRange(4), getSonarRange(5)}).min().getAsDouble();

            //
            Delay.ms(1);
        }
    }
    //endregion
    //endregion
}