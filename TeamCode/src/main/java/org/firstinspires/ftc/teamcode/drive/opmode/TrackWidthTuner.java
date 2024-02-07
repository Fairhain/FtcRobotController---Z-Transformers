package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.MovingStatistics;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This routine determines the effective track width. The procedure works by executing a point turn
 * with a given angle and measuring the difference between that angle and the actual angle (as
 * indicated by an external IMU/gyro, track wheels, or some other localizer). The quotient
 * given angle / actual angle gives a multiplicative adjustment to the estimated track width
 * (effective track width = estimated track width * given angle / actual angle). The routine repeats
 * this procedure a few times and averages the values for additional accuracy. Note: a relatively
 * accurate track width estimate is important or else the angular constraints will be thrown off.
 */
@Config
@Disabled
@Autonomous(group = "drive")
public class TrackWidthTuner extends LinearOpMode {
    private DcMotorEx motorArm;
    private int armPos, armPosMin, armPosMax;
    private Servo scooper;
    public static double ANGLE = 180; // deg
    public static int NUM_TRIALS = 5;
    public static int DELAY = 1000; // ms

    @Override
    public void runOpMode() throws InterruptedException {
        armPos = 0;
        armPosMax = 4260;
        armPosMin = 0;
        motorArm = hardwareMap.get(DcMotorEx.class, "motorArm");
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);
        scooper = hardwareMap.servo.get("scooper");
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // TODO: if you haven't already, set the localizer to something that doesn't depend on
        // drive encoders for computing the heading

        telemetry.addLine("Press play to begin the track width tuner routine");
        telemetry.addLine("Make sure your robot has enough clearance to turn smoothly");
        telemetry.update();

        waitForStart();
        scooper.setPosition(0.4);
        setliftpos(150, 0.3);
        if (isStopRequested()) return;

        telemetry.clearAll();
        telemetry.addLine("Running...");
        telemetry.update();

        MovingStatistics trackWidthStats = new MovingStatistics(NUM_TRIALS);
        for (int i = 0; i < NUM_TRIALS; i++) {
            drive.setPoseEstimate(new Pose2d());

            // it is important to handle heading wraparounds
            double headingAccumulator = 0;
            double lastHeading = 0;

            drive.turnAsync(Math.toRadians(ANGLE));

            while (!isStopRequested() && drive.isBusy()) {
                double heading = drive.getPoseEstimate().getHeading();
                headingAccumulator += Angle.normDelta(heading - lastHeading);
                lastHeading = heading;

                drive.update();
            }

            double trackWidth = DriveConstants.TRACK_WIDTH * Math.toRadians(ANGLE) / headingAccumulator;
            trackWidthStats.add(trackWidth);

            sleep(DELAY);
        }

        telemetry.clearAll();
        telemetry.addLine("Tuning complete");
        telemetry.addLine(Misc.formatInvariant("Effective track width = %.2f (SE = %.3f)",
                trackWidthStats.getMean(),
                trackWidthStats.getStandardDeviation() / Math.sqrt(NUM_TRIALS)));
        telemetry.update();

        while (!isStopRequested()) {
            idle();
        }
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
