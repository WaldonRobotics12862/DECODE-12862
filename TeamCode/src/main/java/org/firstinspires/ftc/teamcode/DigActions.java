package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class DigActions {

    public static class Launcher {
        private static DcMotorEx flywheelmotor;
        private static CRServo turret;
        private static Servo trigger;

        public Launcher(HardwareMap hardwareMap) {
            // Launcher initialization, e.g., configuring motors or sensors
            flywheelmotor = hardwareMap.get(DcMotorEx.class, "flywheel");
            turret = hardwareMap.get(CRServo.class, "turret");
            trigger = hardwareMap.get(Servo.class, "trigger");

            flywheelmotor.setDirection(DcMotorEx.Direction.REVERSE);
            trigger.setPosition(0);
        }

        public static class MotorOn implements Action {
            private final double rpm;
            private boolean initialized = false;
            public MotorOn(double rpm) {
                this.rpm = rpm;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    flywheelmotor.setVelocityPIDFCoefficients(100,20,20,1);
                    flywheelmotor.setVelocity(rpm * 28.0 / 60.0);
                    initialized = true;
                }
                double vel = flywheelmotor.getVelocity() * 60.0 / 28.0;
                //packet.put("Target rpm: ", rpm);
                //packet.put("shooterVelocity", vel);
                return vel < (rpm * 0.95);
                //return false;
            }
        }
        public static Action motorOn(double rpm) {
            return new MotorOn(rpm);
        }

        public static class ReadAprilTag implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false;
            }
        }

        public static Action readAprilTag() {
            return new ReadAprilTag();
        }

        public static class TurretTurn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false;
            }
        }

        public static Action turretTurn() {
            return new TurretTurn();
        }

        public static class MotorOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                flywheelmotor.setVelocity(0);
                flywheelmotor.setPower(0);
                return false;
            }
        }
        public static Action motorOff() {
            return new MotorOff();
        }

        public static class DetectDistance implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false;
            }
        }

        public static Action detectDistance() {
            return new DetectDistance();
        }

        public static class PullTrigger implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                trigger.setPosition(1);
                try {
                    sleep(750);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                trigger.setPosition(0);
                try {
                    sleep(250);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                return false;
            }
        }
        public static Action pullTrigger() {
            return new PullTrigger();
        }
    }

    public static class Sensors {

    }

    public static class Intake {
        private static DcMotorEx intake;
        private static CRServo spin;

        public Intake(HardwareMap hardwareMap) {
            // 1 Motor, 2 Direction
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            spin = hardwareMap.get(CRServo.class, "spin");
        }

        public static class IntakeOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(1);
                //spin.setPower(0.25);
                return false;
            }
        }
        public static Action intakeOn() {
            return new IntakeOn();
        }

        public static class IntakeOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(0);
                //spin.setPower(0);
                return false;
            }
        }
        public static Action intakeOff() {
            return new IntakeOff();
        }

        public static class SpitOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(-1);
                return false;
            }
        }
        public static Action spitOut() {
            return new SpitOut();
        }
    }

    public static class Hopper {
        static DigitalChannel magSensor;
        private static CRServo spin;
        private static DcMotor spin_encoder;
        private static DcMotorEx intake;



        public Hopper(HardwareMap hardwareMap) {
            // Hopper initialization, e.g., configuring motors or sensors
            magSensor = hardwareMap.get(DigitalChannel.class, "mag");
            spin = hardwareMap.get(CRServo.class, "spin");
            spin_encoder = hardwareMap.get(DcMotor.class, "spin_encoder");

        }

        public static class SpinToSensor implements Action {
            private long startTime = 0; // New: Variable to store the start time
            private boolean initialized = false; // New: Flag to check if we've started

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // New: Initialize the timer on the first run
                if (!initialized) {
                    startTime = System.currentTimeMillis();
                    initialized = true;
                }
                int encoder_location = spin_encoder.getCurrentPosition();

                double en_power = 0.3 - (encoder_location * 0.0003);
                if (en_power < 0.09) {
                    en_power = 0.09;
                }
                spin.setPower(en_power);

                double TargetLocation = 2200;

                if((encoder_location > TargetLocation && !magSensor.getState()) || System.currentTimeMillis() - startTime > 2000) {
                    spin.setPower(0);
                    spin_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    return false;
                } else {
                    return true;
                }

            }
        }
        public static Action spinToSensor(){
            return new SpinToSensor();
        }

        public static class MotorTurn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                spin.setPower(0.75);

                if(spin_encoder.getCurrentPosition() > 2600){
                    spin_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                return false;
            }
        }
        public static Action motorTurn() {
            return new MotorTurn();
        }

        public static class  MotorOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                spin.setPower(0);
                return false;
            }
        }
        public static Action motorOff() {
            return new MotorOff();
        }

        public static class IdentifyBall implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                return false;
            }
        }
        public static Action identifyBall() {
            return new IdentifyBall();
        }

        public static class IdentifyOrder implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false;
            }
        }
        public static Action identifyOrder() {
            return new IdentifyOrder();
        }

        public static class Sequencing implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false;
            }
        }
        public static Action sequencing() {
            return new Sequencing();
        }
    }

    public static class Parking {
        public Parking(HardwareMap hardwareMap) {
            // Parking initialization, e.g., configuring motors or sensors
        }

        public static class Touch implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false;
            }
        }

        public static Action touch() {
            return new Touch();
        }
    }
}

// TEST