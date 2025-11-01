package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="WaldonTeleOp")
public class WaldonTeleOp extends LinearOpMode {
    boolean flywheel = false;
    boolean intake = false;
    boolean robotEnabled = true;
    double lastPressedX = 0;
    double lastPressedY = 0;
    double lastPressedA = 0;
    double lastPressedB = 0;
    double lastPressedbumper = 0;
    double slow_mode = 1.0;
    double redDistance = 0;
    double blueDistance = 0;
    int goalDetected = 0;
    boolean spinSpindexer = false;
    boolean intakeSpindexer=false;
    long xButtonDebounce = 0;
    long r_bump_ButtonDebounce = 0;
    long l_bump_ButtonDebounce = 0;
    long yButtonDebounce = 0;
    int encoder_location = 0;
    double en_power=0;

    //DcMotorEx spin_encoder = hardwareMap.get(DcMotorEx.class, "spin_encoder");




    private RevColorSensorV3 colorSensor;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    //public DcMotorEx spinEncoder;

    TelemetryPacket packet = null;


    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initAprilTag();

        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightBack");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        IMU imu = hardwareMap.get(IMU.class, "imu");

        DcMotorEx spinEncoder = hardwareMap.get(DcMotorEx.class, "spin_encoder");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        frontLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        CRServo spindexer = hardwareMap.get(CRServo.class, "spin");

        DigActions.Launcher launcher = new DigActions.Launcher(hardwareMap);
        DigActions.Intake intake = new DigActions.Intake(hardwareMap);
        DigActions.Hopper hopper = new DigActions.Hopper(hardwareMap);
        DigActions.Parking parking = new DigActions.Parking(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            Drive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, imu);
            Intake();
            Index(spinEncoder, spindexer);
            WaldonAprilTag();
            Launch();
            Park();

            if(!spinSpindexer) {// && (System.currentTimeMillis() - yButtonDebounce > 300)){
                //manually spin the spindexer but only if not being told to do so by the bumper button
                spindexer.setPower((gamepad2.right_trigger - gamepad2.left_trigger)*.25);
                spinEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            telemetryAprilTag();

            //TelemetryPacket packet = new TelemetryPacket();
            //List<Action> newAction = new ArrayList<>();
            //for (Action action : runningActions) {
            //    action.preview(packet.fieldOverlay());
            //    if(action.run(packet)){
            //        newAction.add(action);
            //    }
            // }
            //runningActions = newAction;

            //dash.sendTelemetryPacket(packet);
        }
        visionPortal.close();

    }

    private void Intake() {
        if (gamepad2.dpad_up) {
            //intake = true;
            Actions.runBlocking(new SequentialAction(DigActions.Intake.intakeOn()));
        }
        if (gamepad2.dpad_down) {
            //intake = false;
            Actions.runBlocking(new SequentialAction(DigActions.Intake.intakeOff()));
        }
        if (gamepad2.dpad_left) {
            Actions.runBlocking(new SequentialAction(DigActions.Intake.spitOut()));
        }

    }

    private void Index(DcMotorEx spin_encoder, CRServo spindexer) {

        if (gamepad2.left_bumper && (System.currentTimeMillis() - l_bump_ButtonDebounce > 500)) {
            l_bump_ButtonDebounce = System.currentTimeMillis();
            //spin_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //intakeSpindexer = true;

            double hue = getBallHue();
            if (hue >= 90 && hue <= 150) {
                telemetry.addData("Ball Color", "Green Detected (Hue: %.2f)", hue);
                //Actions.runBlocking(new SequentialAction(DigActions.Hopper.acceptBall()));
            } else {
                telemetry.addData("Ball Color", "Not Green (Hue: %.2f)", hue);
                //Actions.runBlocking(new SequentialAction(DigActions.Hopper.spintoSensor()));
            }
            telemetry.update();
        }
        if (gamepad2.right_bumper && (System.currentTimeMillis() - r_bump_ButtonDebounce > 500)) {
            r_bump_ButtonDebounce = System.currentTimeMillis();
            spin_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            spinSpindexer=true;

            double hue = getBallHue();
            if (hue >= 270 && hue <= 300) {
                telemetry.addData("Ball Color", "Purple Detected (Hue: %.2f)", hue);
                // Add action for purple ball if needed
            } else {
                telemetry.addData("Ball Color", "Not Purple (Hue: %.2f)", hue);
            }
            telemetry.update();
        }

        if(intakeSpindexer){
            encoder_location = spin_encoder.getCurrentPosition();
            en_power = (1500 - encoder_location)*.00015;
            if(en_power < 0.075){en_power = 0.075;}

            spindexer.setPower(en_power);

            if (encoder_location > 1320) { // 2675 is too far, 2620 2731
                intakeSpindexer = false;
                spindexer.setPower(0);
                spin_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }

        if (spinSpindexer){
            encoder_location = spin_encoder.getCurrentPosition();
            en_power = (3000 - encoder_location)*.00015;
            if(en_power < 0.075){en_power = 0.075;}

            spindexer.setPower(en_power);

            if (encoder_location > 2640) { // 2675 is too far, 2620 2731
                spinSpindexer = false;
                spindexer.setPower(0);
                spin_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

        }

    }

    private double getBallHue() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();
        int myColor = colorSensor.getNormalizedColors().toColor();
        double hue = JavaUtil.rgbToHue(Color.red(myColor), Color.green(myColor), Color.blue(myColor));
        telemetry.addData("RGB", "R: %d, G: %d, B: %d", red, green, blue);
        telemetry.update();
        return hue;
    }

    private void Launch() {
        if(flywheel){
            //double targetFlywheelSpeed = (Math.pow(redDistance+blueDistance,3) * -0.0039) + (Math.pow(redDistance+blueDistance,2)*1.782) - ((redDistance+blueDistance)*92.01) + 5162.5;
            //double targetFlywheelSpeed = (redDistance+blueDistance)*25+1250;

            double targetFlywheelSpeed = 0.3448*(redDistance+blueDistance)*(redDistance+blueDistance) - 41.693*(redDistance+blueDistance) + 4400;
            if (targetFlywheelSpeed>3700){targetFlywheelSpeed=3700;}

            Actions.runBlocking(new SequentialAction(DigActions.Launcher.motorOn(targetFlywheelSpeed)));
        }

        if (gamepad2.x && !flywheel && (System.currentTimeMillis() - xButtonDebounce > 500)) {
            xButtonDebounce = System.currentTimeMillis();
            flywheel = true;
            // calculate the speed of the flywheel based on how far away we are
            //double targetFlywheelSpeed = (redDistance+blueDistance)*25+1250;
            //double targetFlywheelSpeed = (Math.pow(redDistance+blueDistance,3) * -0.0039) + (Math.pow(redDistance+blueDistance,2)*1.782) - ((redDistance+blueDistance)*92.01) + 5162.5;
            //Actions.runBlocking(new SequentialAction(DigActions.Launcher.motorOn(targetFlywheelSpeed)));
        }
        if (gamepad2.x && flywheel && (System.currentTimeMillis() - xButtonDebounce > 500)) {
            xButtonDebounce = System.currentTimeMillis();
            flywheel = false;
            Actions.runBlocking(new SequentialAction(DigActions.Launcher.motorOff()));
        }
        if (gamepad2.y) {
            yButtonDebounce = System.currentTimeMillis();
            Actions.runBlocking(new SequentialAction(DigActions.Launcher.pullTrigger()));
        }

        if(gamepad2.guide){
            //Launch all three in a sequence:
            Actions.runBlocking(new SequentialAction(
                    DigActions.Launcher.pullTrigger(),
                    DigActions.Hopper.spinToSensor(),
                    DigActions.Launcher.pullTrigger(),
                    DigActions.Hopper.spinToSensor(),
                    DigActions.Launcher.pullTrigger()
                )
            );
        }
    }

    private void Park() {
        // Implement parking logic if needed
    }

    private void Drive(DcMotorEx frontLeftMotor, DcMotorEx backLeftMotor, DcMotorEx frontRightMotor, DcMotorEx backRightMotor, IMU imu) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (gamepad1.a) {
            imu.resetYaw();
        }

        if (gamepad1.left_bumper && (System.currentTimeMillis() - lastPressedA > 250)) {
            slow_mode = (slow_mode == 1.0) ? 2.0 : 1.0; // Toggle Turbo
            lastPressedbumper = System.currentTimeMillis();
        }

        if (gamepad1.right_bumper && (System.currentTimeMillis() - lastPressedA > 250)) {
            slow_mode = (slow_mode == 1.0) ? 0.5 : 1.0; // Toggle Turbo
            lastPressedbumper = System.currentTimeMillis();
        }

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.right_stick_button && (System.currentTimeMillis() - lastPressedB > 250)) {
            robotEnabled = !robotEnabled;
            lastPressedB = System.currentTimeMillis();
            if (!robotEnabled) {
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
            }
        }

        if (!robotEnabled) return;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

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
    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */

    private void WaldonAprilTag(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if(detection.id == 20) {
                //this is a blue goal
                goalDetected = 1; // use this for the Blue Goal
                blueDistance = detection.ftcPose.y;
                redDistance = 0;

            } else if(detection.id == 24) {
                //this is a red goal
                goalDetected = 2; // use this for the Red Goal
                redDistance = detection.ftcPose.y;
                blueDistance = 0;
            }
            else {
                //don't care
            }
        }
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                //telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                //telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop
        telemetry.update();

        // Add "key" information to telemetry
        // telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        // telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        // telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

}   // end class

