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

@Autonomous(name="Red Goal Auton", preselectTeleOp = "WaldonTeleOp")
@Config

public class RedGoalAuton extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d beginPose = new Pose2d(-49, 49, Math.toRadians(125));
        Pose2d launchPose = new Pose2d(-24, 24,Math.toRadians(135));
        Pose2d parkPose = new Pose2d(-60,24,Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        drive.localizer.setPose(beginPose);

        Action all_drives = drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(-35))
                .splineToLinearHeading(launchPose,Math.toRadians(-35))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-12,24,Math.toRadians(90)),Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(launchPose,Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-60,24,Math.toRadians(180)),Math.toRadians(180))
                .build();

        Action drive_to_launch_1 = drive.actionBuilder(beginPose)
                //.setTangent(Math.toRadians(-35))
                .splineToLinearHeading(launchPose,Math.toRadians(125))
                .build();

        Action drive_to_intake_1 = drive.actionBuilder(launchPose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-12,24,Math.toRadians(90)),Math.toRadians(90))
                .lineToY(54)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(launchPose,Math.toRadians(90))
                .build();

        Action park = drive.actionBuilder(launchPose)
                .splineToLinearHeading(parkPose,Math.toRadians(180))
                .build();


        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        drive_to_launch_1
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
}