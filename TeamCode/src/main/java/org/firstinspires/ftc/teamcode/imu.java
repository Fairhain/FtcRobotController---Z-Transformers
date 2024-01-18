package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "PID Testing")
@Config
public class imu extends LinearOpMode {

    IMU imu;

    DcMotorEx FrontLeft;
    DcMotorEx FrontRight;
    DcMotorEx BackLeft;
    DcMotorEx BackRight;

    double integralSum = 0;
    public static double Kp = 1.8;
    public static double Ki = 0;
    public static double Kd = 0.031;
    double Kf = 10;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        imu = hardwareMap.get(IMU.class, "imu");

        FrontLeft = hardwareMap.get(DcMotorEx.class, "motorLeft");
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight = hardwareMap.get(DcMotorEx.class, "motorRight");
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                        )
                )
        );

        waitForStart();

        imu.resetYaw();
        double referenceAngle = Math.toRadians(90);
        while (opModeIsActive()) {

            turn(referenceAngle);


            telemetry.addData("Target IMU angle", referenceAngle);
            telemetry.addData("Current IMU angle", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.update();



        }



    }
    public void turn(double referenceAngle){
        double power = PIDControl(referenceAngle, imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        FrontLeft.setPower(-power);
        BackLeft.setPower(-power);
        FrontRight.setPower(power);
        BackRight.setPower(power);
    }


    public double PIDControl(double reference, double state) {
        double error = angleWrap(reference - state);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();


        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }

    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }
}
