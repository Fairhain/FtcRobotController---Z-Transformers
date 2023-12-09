package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name = "bradenscode")
public class bradenscode extends LinearOpMode {
    private int armPos, armPosMax, armPosMin;
    private int armLeftPos, slider;
    private int minPosLeft, minPosRight;
    private DcMotor motorArm;
    private Servo scooper, autonServo;
    private Servo droneServo;
    private int claw;
    private boolean clawStatus;



    @Override
    public void runOpMode() {
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorLeft");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorRight");


        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorArm = hardwareMap.dcMotor.get("motorArm");

        scooper = hardwareMap.get(Servo.class, "scooper");
        droneServo = hardwareMap.servo.get("droneServo");
        autonServo = hardwareMap.servo.get("autonServo");
        boolean servoButton = gamepad1.x;
        boolean clawStatus = true;
        boolean ranvar = true;

        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        armPos = 0;
        armLeftPos = 0;
        armPosMax = 4260;
        armPosMin = 0;



        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests



        // Left is negative in reverse, that means normal (forward)

        waitForStart();
        //scooper.setPosition(0.365);
        //raises the scoop a bit higher
        scooper.setPosition(0.4);
        droneServo.setPosition(0.55);
        autonServo.setPosition(0.5);

        if (isStopRequested()) return;




        while (opModeIsActive()) {
            autonServo.setPosition(0.5);
            telemetry.addData("opmodeisActive",slider);
            double y = -gamepad2.left_stick_y; // Remember, this is reversed!
            double x = gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad2.right_stick_x;



            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator/1.5;
            double backLeftPower = (y - x + rx) / denominator/1.5;
            double frontRightPower = (y - x - rx) / denominator/1.5;
            double backRightPower = (y + x - rx) / denominator/1.5;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);




            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]



            if (gamepad1.x) {
                //scooper.setPosition(0.365); // moves up
                scooper.setPosition(0.358);

            } else if (gamepad1.b) {
                scooper.setPosition(0.3); //open left
            }

            if (gamepad1.y && slider == 0) {
                setliftpos(150, 0.3);
                slider = 1;
                telemetry.addData("slider is 1", slider);
                telemetry.update();

            } else if (gamepad1.y && slider == 1) {
                setliftpos(1275, 0.2);
                slider = 2;
                telemetry.addData("slider is 3", slider);
                telemetry.update();
            }

            if (gamepad1.a && slider == 1) {
                setliftpos(0, 0.3);
                slider = 0;

            } else if (gamepad1.a && slider == 2) {
                setliftpos(150, 0.3);
                slider = 1;
            }
            if (gamepad1.right_bumper){
                droneServo.setPosition(0.05);
            } else if (gamepad1.left_bumper){
                droneServo.setPosition(0.55);
            }

            //minPosLeft= armMotorLeft.getCurrentPosition();
            //minPosRight= armMotorRight.getCurrentPosition();

            //telemetry.addData("left position", minPosLeft);
            //telemetry.addData("right position", minPosRight);
            //telemetry.update();


            //telemetry.addData("left position", minPosLeft);
            //telemetry.addData("right position", minPosRight);
            //telemetry.update();
            /* if (gamepad1.y){
                drive(150,0.2);
            }

            if (gamepad1.a){
                drive(-200,0.2);
            }
            */

            /* if (gamepad1.b)
            {
                clawLeft.setPosition(0.15); // close left
                clawRight.setPosition(0.71); //close right
            }
            if (gamepad1.x){
                clawLeft.setPosition(0.35); //open left
                clawRight.setPosition(0.5); //open right
            }
            */

            telemetry.update();
        } // END OF OPMODE -------------------------------------------------------------------------

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