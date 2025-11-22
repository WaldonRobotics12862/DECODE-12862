package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;


@Autonomous(name="Red Audience Auton", preselectTeleOp = "WaldonTeleOp")
@Config
public class RedAudienceAuton extends LinearOpMode {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public int OBELISK = 2;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx spinEcoder;
        spinEcoder = hardwareMap.get(DcMotorEx.class, "spin_encoder");
        spinEcoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //DigActions.Sensors Sensors = new DigActions.Sensors(hardwareMap);
        DigActions.Intake Intake = new DigActions.Intake(hardwareMap);
        DigActions.Launcher Launcher = new DigActions.Launcher(hardwareMap);
        DigActions.Parking Parking = new DigActions.Parking(hardwareMap);
        DigActions.Hopper Hopper = new DigActions.Hopper(hardwareMap);


        Pose2d beginPose = new Pose2d(62,12,Math.toRadians(180));
        Pose2d launchPose = new Pose2d(58,12,Math.toRadians(159));

        initAprilTag();

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        drive.localizer.setPose(beginPose);

        Action inital_move = drive.actionBuilder(beginPose)
                .splineToLinearHeading(launchPose,0)
                .build();

        Action intake_artifact = drive.actionBuilder(launchPose)
                //.setTangent(200)
                .splineTo(new Vector2d(36, 32), Math.toRadians(90))
                .lineToY(54)
                .waitSeconds(1)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(58,18,Math.toRadians(160)), Math.toRadians(-20))
                .build();

        Action park = drive.actionBuilder(launchPose)
                .splineTo(new Vector2d(36, 32), Math.toRadians(90))
                .build();

        spinEcoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /**
         * Auton START
         */
        waitForStart();
        AutonDetection();

        spinEcoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(OBELISK == 1){
            Actions.runBlocking(DigActions.Hopper.spinToSensor());
        } else if (OBELISK == 2){
            Actions.runBlocking(DigActions.Hopper.spinToSensor());
            Actions.runBlocking(DigActions.Hopper.spinToSensor());
        }

        Actions.runBlocking(
                new SequentialAction(
                        inital_move,
                        new DigActions.Launcher.MotorOn(3600),
                        new SleepAction(1), // let the motor get up to speed
                        new DigActions.Launcher.PullTrigger(),
                        new DigActions.Hopper.SpinToSensor(),
                        new DigActions.Launcher.PullTrigger(),
                        new DigActions.Hopper.SpinToSensor(),
                        new DigActions.Launcher.PullTrigger(),
                        new DigActions.Launcher.MotorOn(1500), // slow down to 50% to save battery
                        new DigActions.Intake.IntakeOn(),
                        new ParallelAction(
                                intake_artifact,
                                new SequentialAction(
                                        new DigActions.Hopper.SpinToSensor(),
                                        new DigActions.Hopper.SpinToSensor(),
                                        new DigActions.Hopper.SpinToSensor(),
                                        new DigActions.Hopper.SpinToSensor()
                                )
                        ),
                        new DigActions.Intake.IntakeOff(),
                        new DigActions.Hopper.MotorOff(),
                        new DigActions.Launcher.MotorOn(3600),
                        new DigActions.Hopper.SpinToSensor(),
                        new DigActions.Launcher.PullTrigger(),
                        new DigActions.Hopper.SpinToSensor(),
                        new DigActions.Launcher.PullTrigger(),
                        new DigActions.Hopper.SpinToSensor(),
                        new DigActions.Launcher.PullTrigger(),
                        new DigActions.Launcher.MotorOff(),
                        park

                )
        );
    }

    private void initAprilTag() {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

    }   // end method initAprilTag()

    private void AutonDetection(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if(detection.id == 21) {
                //GREEN PURPLE PURPLE
                // No Rotation
                OBELISK = 0;

            } else if(detection.id == 22) {
                //PURPLE GREEN PURPLE
                // Rotate 1
                OBELISK = 2;
            }
            else if(detection.id == 23)  {
                //PURPLE PURPLE GREEN
                // Rotate 2 or better yet, rotate backwards if we can
                OBELISK = 1;
            }
        }
    }
}