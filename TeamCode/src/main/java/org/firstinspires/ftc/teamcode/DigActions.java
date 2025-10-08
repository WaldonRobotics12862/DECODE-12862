package org.firstinspires.ftc.teamcode;
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

        public Launcher(HardwareMap hardwareMap) {
            // Launcher initialization, e.g., configuring motors or sensors
            flywheelmotor = hardwareMap.get(DcMotorEx.class, "flywheel");
            turret = hardwareMap.get(CRServo.class, "turret");

        }

        public static class MotorOn implements Action {
            public Integer rpm;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                flywheelmotor.setVelocity(rpm);
                double flyWheelSpeed = flywheelmotor.getVelocity();
                if (flyWheelSpeed > (rpm*0.9)) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public static Action motorOn(Integer rpm) {
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
    }

    public static class Sensors {

    }

    public static class Intake {
        public Intake(HardwareMap hardwareMap) {
            // 1 Motor, 2 Direction
        }

        public static class IntakeOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false;
            }
        }

        public static Action intakeOn() {
            return new IntakeOn();
        }

        public static class IntakeOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false;
            }
        }

        public static Action intakeOff() {
            return new IntakeOff();
        }

        public static class SpitOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false;
            }
        }

        public static Action spitOut() {
            return new SpitOut();
        }
    }

    public static class Hopper {
        private static DigitalChannel magSensor;
        private static DcMotorEx spin;

        public Hopper(HardwareMap hardwareMap) {
            // Hopper initialization, e.g., configuring motors or sensors
            magSensor = hardwareMap.get(DigitalChannel.class, "mag");
            spin = hardwareMap.get(DcMotorEx.class, "spin");

        }

        public static class MotorTurn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false;
            }
        }

        public static Action motorTurn() {
            return new MotorTurn();
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

        public static class SpinToSensor implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                spin.setPower(0.1);
                // add sleep maybe????
                if (magSensor.getState()) {
                    return true;
                } else {
                    spin.setPower(0);
                    return false;
                }
            }
        }

        public static Action spintoSensor() {
            return new SpinToSensor();
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