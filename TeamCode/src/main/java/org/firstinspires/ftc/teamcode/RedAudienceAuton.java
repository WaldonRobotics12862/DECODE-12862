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


@Autonomous(name="Red Audience Auton")
@Config
public class RedAudienceAuton extends LinearOpMode {
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


        Pose2d beginPose = new Pose2d(62, 12, Math.toRadians(180));
        Pose2d launchPose =new Pose2d(58, 12, Math.toRadians(160));

        Pose2d newPose = new Pose2d(-12, 0,0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(62,12, Math.toRadians(0)));

        Action inital_move = drive.actionBuilder(new Pose2d(62,12,Math.toRadians(0))) //beginPose)
                .lineToX(58)
                .turn(Math.toRadians(-20))
                //.splineToLinearHeading(new Pose2d(-12, 2, Math.toRadians(20)),0)
                .build();

        Action intake_artifact = drive.actionBuilder(new Pose2d(58,12,Math.toRadians(160)))
                .splineTo(new Vector2d(36, 32), Math.toRadians(90))
                .lineToY(48)
                .setTangent(-90)
                .splineToLinearHeading(new Pose2d(58,12,Math.toRadians(160)),Math.toRadians(0))
                .build();

        Action park = drive.actionBuilder(launchPose)
                .splineTo(new Vector2d(36, 32), Math.toRadians(90))
                .build();

        spinEcoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        spinEcoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Actions.runBlocking(
                new SequentialAction(
                        inital_move,
                        //new DigActions.Launcher.MotorOn(3700),
                        new SleepAction(2), // let the motor get up to speed
                        //new DigActions.Launcher.PullTrigger(),
                        //new DigActions.Launcher.MotorOff(),
                        new DigActions.Hopper.SpinToSensor(),
                        //new DigActions.Launcher.MotorOn(3700),
                        //new SleepAction(2), // let the motor get up to speed
                        //new DigActions.Launcher.PullTrigger(),
                        //new DigActions.Launcher.MotorOff(),
                        new DigActions.Hopper.SpinToSensor(),
                        //new DigActions.Launcher.MotorOn(3700),
                        //new SleepAction(2), // let the motor get up to speed
                        //new DigActions.Launcher.PullTrigger(),
                        new DigActions.Launcher.MotorOff(),
                        new DigActions.Intake.IntakeOn(),
                        new DigActions.Hopper.MotorTurn(),
                        intake_artifact,
                        new DigActions.Intake.IntakeOff(),
                        new DigActions.Hopper.MotorOff(),
                        //new DigActions.Launcher.MotorOn(3700),
                        new SleepAction(2), // let the motor get up to speed
                        //new DigActions.Launcher.PullTrigger(),
                        //new DigActions.Launcher.MotorOff(),
                        new DigActions.Hopper.SpinToSensor(),
                        //new DigActions.Launcher.MotorOn(3700),
                        //new SleepAction(2), // let the motor get up to speed
                        //new DigActions.Launcher.PullTrigger(),
                        //new DigActions.Launcher.MotorOff(),
                        new DigActions.Hopper.SpinToSensor(),
                        //new DigActions.Launcher.MotorOn(3700),
                        //new SleepAction(2), // let the motor get up to speed
                        //new DigActions.Launcher.PullTrigger(),
                        new DigActions.Launcher.MotorOff(),
                        park

        // launch 3 artifacts
                        //intake_artifact,
                        //launch 3 artifacts
                        //park

                )
        );
    }
}