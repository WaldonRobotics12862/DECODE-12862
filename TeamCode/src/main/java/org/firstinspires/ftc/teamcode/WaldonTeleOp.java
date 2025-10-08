package org.firstinspires.ftc.teamcode;

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
import android.graphics.Color;
import com.qualcomm.hardware.rev.RevColorSensorV3;
@TeleOp(name="WaldonTeleOp")
public class WaldonTeleOp extends LinearOpMode {
    boolean flywheel = false;
    boolean intake = false;
    double lastPressedX = 0;
    double lastPressedY = 0;
    double lastPressedA = 0;
    double lastPressedB = 0;
    double slow_mode = 1;

    // Declare the color sensor
    private RevColorSensorV3 colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightBack");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        // Initialize the color sensor
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

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
        DigActions.Parking parking = new DigActions.Parking(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            Drive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, imu);
            Intake();
            Index();
            Launch();
            Park();
        }
    }

    private void Intake() {
        if (gamepad2.dpad_up && !intake) {
            intake = true;
            Actions.runBlocking(new SequentialAction(DigActions.Intake.intakeOn()));
        }
        if (gamepad2.dpad_down && intake) {
            intake = false;
            Actions.runBlocking(new SequentialAction(DigActions.Intake.intakeOff()));
        }
        if (gamepad2.dpad_left) {
            Actions.runBlocking(new SequentialAction(DigActions.Intake.spitOut()));
        }
    }

    private void Index() {
        if (gamepad2.left_bumper) {
            // Check if the ball is green using hue detection
            double hue = getBallHue();
            if (hue >= 90 && hue <= 150) {
                telemetry.addData("Ball Color", "Green Detected (Hue: %.2f)", hue);
                // Example action: Accept the green ball
                // Actions.runBlocking(new SequentialAction(DigActions.Hopper.acceptBall()));
            } else {
                telemetry.addData("Ball Color", "Not Green (Hue: %.2f)", hue);
                // Example action: Reject or ignore non-green ball
                Actions.runBlocking(new SequentialAction(DigActions.Hopper.spintoSensor()));

            }
            telemetry.update();
        }
        if (gamepad2.right_bumper) {
            // Placeholder for purple ball detection
            double hue = getBallHue();
            telemetry.addData("Ball Color", "Checking for Purple (Hue: %.2f)", hue);
            // Implement purple detection (e.g., hue ~270-300) if needed
            telemetry.update();
        }
    }

    private double getBallHue() {
        // Get RGB values from the color sensor
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();
        int myColor = colorSensor.getNormalizedColors().toColor();
        double hue = JavaUtil.rgbToHue(Color.red(myColor), Color.green(myColor), Color.blue(myColor));
        // Log RGB and HSV for debugging
        telemetry.addData("RGB", "R: %d, G: %d, B: %d", red, green, blue);
        return hue;
    }

    private void Launch() {
        if (gamepad2.x && !flywheel) {
            flywheel = true;
            Actions.runBlocking(new SequentialAction(DigActions.Launcher.motorOn(4000)));
        }
        if (gamepad2.x && flywheel) {
            Actions.runBlocking(new SequentialAction(DigActions.Launcher.motorOn(0)));
            flywheel = false;
        }
        if (gamepad2.y) {
            Actions.runBlocking(new SequentialAction(DigActions.Launcher.trigger()));
        }
    }

    private void Park() {
        // Implement parking logic if needed
    }

    private void Drive(DcMotorEx frontLeftMotor, DcMotorEx backLeftMotor, DcMotorEx frontRightMotor, DcMotorEx backRightMotor, IMU imu) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (gamepad1.right_bumper) {
            slow_mode = 0.5;
        } else {
            slow_mode = 1;
        }

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        if (gamepad1.a) {
            imu.resetYaw();
        }

        rotX = rotX * 1.1; // Counteract imperfect strafing

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