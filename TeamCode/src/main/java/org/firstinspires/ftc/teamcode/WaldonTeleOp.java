package org.firstinspires.ftc.teamcode;
//package org.firstinspires.ftc.teamcode.tuning;

import android.graphics.Color;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="WaldonTeleOp")

public class WaldonTeleOp extends LinearOpMode {
    boolean flywheel = false;
    boolean intake = false;
    double lastPressedX = 0;
    double lastPressedY = 0;
    double lastPressedA = 0;
    double lastPressedB = 0;

    double slow_mode = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class,"leftFront");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class,"leftBack");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class,"rightBack");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class,"rightFront");

        // Adjust the orientation parameters to match your robot
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DigActions.Launcher launcher = new DigActions.Launcher(hardwareMap);
        DigActions.Intake intake = new DigActions.Intake(hardwareMap);
        DigActions.Hopper hopper = new DigActions.Hopper(hardwareMap);
        //DigActions.Sensors sensors = new DigActions.Sensors(hardwareMap);
        DigActions.Parking parking = new DigActions.Parking(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Drive
            Drive(frontLeftMotor,backLeftMotor, frontRightMotor, backRightMotor, imu);
            // Intake

            // Index / queue up next artifact

            // Launch

            // Park

        }
    }

    private void Intake(){
        if(gamepad2.dpad_up && !intake){
            intake = true;
            Actions.runBlocking(new SequentialAction(DigActions.Intake.intakeOn()));
        }
        if(gamepad2.dpad_down && intake){
            intake = false;
            Actions.runBlocking(new SequentialAction(DigActions.Intake.intakeOff()));
        }
        if(gamepad2.dpad_left){
            Actions.runBlocking(new SequentialAction(DigActions.Intake.spitOut()));
        }
    }

    private void Index (){
        if(gamepad2.left_bumper) {
            Actions.runBlocking(new SequentialAction(DigActions.Hopper.
            // Find green ball
        }
        if (gamepad2.right_bumper){
            //find Purple ball
        }
    }

    private void Launch(){
        // need a button to spin up the flywheel
        if(gamepad2.x && !flywheel){
            flywheel = true;
            Actions.runBlocking(new SequentialAction(DigActions.Launcher.motorOn(4000)));
        }
        if(gamepad2.x && flywheel){
            Actions.runBlocking(new SequentialAction(DigActions.Launcher.motorOn(0)));
            flywheel = false;
        }
        if(gamepad2.y){
            Actions.runBlocking(new SequentialAction(DigActions.Launcher.trigger()));
            // TODO go back and write a smelly trigger action
        }

        // need a button (or two) to launch an artifact
    }

    private void Park(){

    }

    private void Drive(DcMotorEx frontLeftMotor, DcMotorEx backLeftMotor, DcMotorEx frontRightMotor, DcMotorEx backRightMotor, IMU imu){
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // Rotate the movement direction counter to the bot's rotation
        if(gamepad1.right_bumper) {
            slow_mode = .5;
        } else {
            slow_mode = 1;
        }
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        if (gamepad1.a) {
            imu.resetYaw();
        }

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = ((rotY + rotX + rx) / denominator) * slow_mode;
        double backLeftPower = ((rotY - rotX + rx) / denominator) * slow_mode;
        double frontRightPower = ((rotY + rotX - rx) / denominator) * slow_mode;
        double backRightPower = ((rotY - rotX - rx) / denominator) * slow_mode;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

}
