package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous (name = "autonthingyredlower", preselectTeleOp = "bradenscode")
public class autonthingyRedLowerPurpleOnly extends LinearOpMode {
    private int armPos, armPosMin, armPosMax;
    private int spikePos;
    private Servo autonServo;
    private Servo scooper;
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor motorArm;
    private int LmT = 0;
    private int RmT = 0;
    private int LbmT = 0;
    private int RbmT = 0;
    double integralSum = 0;

    double Kp = 2;
    double Ki = 0.0;
    double Kd = 0.0;
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    double cX = 0;
    double cY = 0;
    double width = 0;

    private static String side = "";
    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    /** MAKE SURE TO CHANGE THE FOV AND THE RESOLUTIONS ACCORDINGLY **/
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution
    private static final double FOV = 40;

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels

    @Override
    public void runOpMode() {
        armPos = 0;
        armPosMax = 4260;
        armPosMin = 0;

        motorFrontLeft = hardwareMap.dcMotor.get("motorLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorArm = hardwareMap.dcMotor.get("motorArm");
        scooper = hardwareMap.servo.get("scooper");
        autonServo = hardwareMap.servo.get("autonServo");

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);

        // "Zero Power Behavior Brake" stops the wheels from drifting when you have already stopped putting power.
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        initOpenCV();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        telemetry.setMsTransmissionInterval(50);


        while (!isStarted() && !isStopRequested()) {
            autonServo.setPosition(0.5);
            scooper.setPosition(0.3);

            //autonServo.setPosition(0.9);


            telemetry.addLine("Coordinate" + "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addLine("Distance in Inch" + (getDistance(width)));

            if ((int) cX <= 100) {
                telemetry.addLine("Side: Left");
                side = "left";
            } else if ((int) cX >= 100 && (int) cX < 350) {

                telemetry.addLine("Side: Center");
                side = "center";
            } else if ((int) cX >= 350) {

                telemetry.addLine("Side: Right");
                side = "right";
            } else {
                side = "";
                telemetry.addLine("Side: Not Found");
            }
            telemetry.update();
            sleep(20);

        }


        telemetry.update();
        telemetry.addData("Side", side);
        telemetry.update();


        setliftpos(150, 0.3);

        if (side.equals("right")) {
            setliftpos(100,0.3);
            driveForward(1100, 1100, 0.3);
            strafeRight(150,150,0.3);

            //place pixel here
            autonServo.setPosition(0.9);
            sleep(1000);
            autonServo.setPosition(0.5);
            driveForward(-150, -150, 0.3);
            setliftpos(0, 0.3);

        } else if (side.equals("left")) {


            driveForward(1000, 1000, 0.3);
            driveForward(-1000, 1000, 0.3);

            //place pixel here
            autonServo.setPosition(0.9);
            sleep(1000);
            autonServo.setPosition(0.5);
            driveForward(-100, -100, 0.3);
            setliftpos(0, 0.3);

        } else {
            driveForward(1250, 1250, 0.3);
            strafeLeft(250, 250, 0.3);
            //place pixel here
            autonServo.setPosition(0.9);
            sleep(1000);
            autonServo.setPosition(0.5);
            driveForward(-100, -100, 0.3);
            setliftpos(0, 0.3);

        }

        // Release resources
        controlHubCam.stopStreaming();
    }
    public void strafeLeft(int mltarget, int mrtarget, double speed) {
        // to reverse set the position as negative value for left front motor
        LmT += -mltarget;
        RmT += mrtarget;
        LbmT += mltarget;
        // to reverse set the position as negative value for right back motor
        RbmT += -mrtarget;

        motorFrontLeft.setTargetPosition(LmT);
        motorFrontRight.setTargetPosition(RmT);
        motorBackLeft.setTargetPosition(LbmT);
        motorBackRight.setTargetPosition(RbmT);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackRight.setPower(speed);
        motorBackLeft.setPower(speed);

        while (opModeIsActive() && motorFrontLeft.isBusy() && motorFrontRight.isBusy() &&
                motorBackLeft.isBusy() && motorBackRight.isBusy()) {

        }

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
    }
    public void strafeRight(int mltarget, int mrtarget, double speed) {
        LmT += mltarget;
        // to reverse set the position as negative value for right front motor
        RmT += -mrtarget;
        // to reverse set the position as negative value for left back motor
        LbmT += -mrtarget;
        RbmT += mltarget;

        motorFrontLeft.setTargetPosition(LmT);
        motorFrontRight.setTargetPosition(RmT);
        motorBackLeft.setTargetPosition(LbmT);
        motorBackRight.setTargetPosition(RbmT);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackRight.setPower(speed);
        motorBackLeft.setPower(speed);

        while (opModeIsActive() && motorFrontLeft.isBusy() && motorFrontRight.isBusy() &&
                motorBackLeft.isBusy() && motorBackRight.isBusy()) {

        }

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
    }
    public void driveForward(int mltarget, int mrtarget, double speed) {
        LmT += mltarget;
        RmT += mrtarget;
        LbmT += mltarget;
        RbmT += mrtarget;

        motorFrontLeft.setTargetPosition(LmT);
        motorFrontRight.setTargetPosition(RmT);
        motorBackLeft.setTargetPosition(LbmT);
        motorBackRight.setTargetPosition(RbmT);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackRight.setPower(speed);
        motorBackLeft.setPower(speed);
        while (opModeIsActive() && motorFrontLeft.isBusy() && motorFrontRight.isBusy() &&
                motorBackLeft.isBusy() && motorBackRight.isBusy()) {
            idle();
        }
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);


    }





    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
    public void setliftpos(int armTarget, double speed) {
        armPos = armTarget;

        // validate armPos is within Min and Max
        armPos = Range.clip(armPos, armPosMin, armPosMax);


        motorArm.setTargetPosition(armPos);

        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorArm.setPower(speed);
        while (opModeIsActive() && motorArm.isBusy()) {
            idle();

        }

    }
    class YellowBlobDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerYellow = new Scalar(100, 100, 100);
            Scalar upperYellow = new Scalar(180, 255, 255);


            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }
    private static double getAngleTarget(double objMidpoint){
        double midpoint = -((objMidpoint - (CAMERA_WIDTH/2))*FOV)/CAMERA_WIDTH;
        return midpoint;
    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }
    public double PIDControl(double refrence, double state) {
        double error = angleWrap(refrence - state);
        telemetry.addData("Error: ", error);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
    public double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }



}
