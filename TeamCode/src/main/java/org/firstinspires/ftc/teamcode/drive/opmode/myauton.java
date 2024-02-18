package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Config
@Autonomous
public class myauton extends LinearOpMode {
    private int armPos, armPosMin, armPosMax;
    private DcMotorEx motorArm;
    private Servo autonServo;
    private Servo scooper;
    private Servo autonServo2;
    public void runOpMode() {
        armPos = 0;
        armPosMax = 4260;
        armPosMin = 0;
        motorArm = hardwareMap.get(DcMotorEx.class, "motorArm");
        scooper = hardwareMap.servo.get("scooper");
        autonServo = hardwareMap.servo.get("autonServo");
        autonServo2 = hardwareMap.servo.get("autonServo2");
        autonServo2.setPosition(0.8);
        autonServo.setPosition(0.4);
        scooper.setPosition(0.3);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setliftpos(150, 0.3);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Trajectory rightPurple = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(36, 0), 0)
                .build();
        Trajectory rightToWall = drive.trajectoryBuilder(rightPurple.end().plus(new Pose2d(0, 0, Math.toRadians(90))), true)
                .splineTo(new Vector2d(30, 29), Math.toRadians(-90))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(rightPurple);
        drive.turn(Math.toRadians(-90));
        autonServo.setPosition(1);
        sleep(1000);
        autonServo.setPosition(0.5);
        drive.followTrajectory(rightToWall);
        drive.turn(Math.toRadians(180));
        autonServo2.setPosition(0);
        sleep(1000);
        autonServo2.setPosition(0.7);
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
}
