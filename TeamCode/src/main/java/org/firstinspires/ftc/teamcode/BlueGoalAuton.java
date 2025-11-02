package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="Blue Goal Auton", preselectTeleOp = "WaldonTeleOp")
@Config
public class BlueGoalAuton extends LinearOpMode {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public int OBELISK = 2;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx spinEcoder;
        spinEcoder = hardwareMap.get(DcMotorEx.class, "spin_encoder");
        spinEcoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DigActions.Intake Intake = new DigActions.Intake(hardwareMap);
        DigActions.Launcher Launcher = new DigActions.Launcher(hardwareMap);
        DigActions.Parking Parking = new DigActions.Parking(hardwareMap);
        DigActions.Hopper Hopper = new DigActions.Hopper(hardwareMap);


        Pose2d beginPose = new Pose2d(-49, -49, Math.toRadians(-125));
        Pose2d obeliskPose = new Pose2d(-26,-26,Math.toRadians(-210));
        Pose2d launchPose = new Pose2d(-24, -24, Math.toRadians(-135));
        Pose2d parkPose = new Pose2d(-33, -10, Math.toRadians(-180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        drive.localizer.setPose(beginPose);

        initAprilTag();

        Action drive_to_launch_1 = drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(35))
                .splineToLinearHeading(obeliskPose, Math.toRadians(-305))
                .build();

        Action AprilTagSpin = drive.actionBuilder(obeliskPose)
                .splineToLinearHeading(launchPose,Math.toRadians(-125),new AngularVelConstraint(Math.PI/2))
                //.turn(Math.toRadians(-80)) // this seems to work but I would rather use the spline to heading functions
                .build();

        Action drive_to_intake_1 = drive.actionBuilder(launchPose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-12, -20, Math.toRadians(-90)), Math.toRadians(90))
                .lineToY(-50)
                .setTangent(Math.toRadians(-270))
                .splineToLinearHeading(launchPose, Math.toRadians(-90))
                .build();

        Action park = drive.actionBuilder(launchPose)
                .splineToLinearHeading(parkPose, Math.toRadians(-180))
                .build();


        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        drive_to_launch_1
                )
        );

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
                        AprilTagSpin,
                        //spin back
                        new DigActions.Launcher.MotorOn(3000),
                        new SleepAction(2), // let the motor get up to speed
                        new DigActions.Launcher.PullTrigger(),
                        new DigActions.Hopper.SpinToSensor(),
                        new DigActions.Launcher.PullTrigger(),
                        new DigActions.Hopper.SpinToSensor(),
                        new DigActions.Launcher.PullTrigger(),
                        new DigActions.Launcher.MotorOff(),
                        new DigActions.Intake.IntakeOn(),

                        drive_to_intake_1,
                        new DigActions.Intake.IntakeOff(),
                        new DigActions.Launcher.MotorOn(3000),
                        new SleepAction(2), // let the motor get up to speed
                        new DigActions.Launcher.PullTrigger(),
                        new DigActions.Hopper.SpinToSensor(),
                        new DigActions.Launcher.PullTrigger(),
                        new DigActions.Hopper.SpinToSensor(),
                        new DigActions.Launcher.PullTrigger(),
                        new DigActions.Launcher.MotorOff(),

                        park
                        //new SleepAction(1000),
                        //launch three artifacts
                        //new ParallelAction(drive_to_intake_1,DigActions.Intake.intakeOn()),
                        //drive_to_launch_2,
                        //launch three artifacts
                        //park

                        //all_drives
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
                // Rotate 2 or better yet, rotate backwards if we can
                OBELISK = 2;
            }
            else if(detection.id == 23)  {
                //PURPLE PURPLE GREEN
                // Rotate 1
                OBELISK = 1;
            }
        }
    }
}