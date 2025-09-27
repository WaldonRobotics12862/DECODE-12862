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

import org.firstinspires.ftc.robotcore.external.JavaUtil;
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
            return new MotorOn();
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
        public static DcMotorEx SpindexMotor;
        private static RevColorSensorV3 Color;
        private static Servo SpindexTrigger;
        private static DigitalChannel MagnetSensor;

        public Hopper(HardwareMap hardwareMap) {
            // Hopper initialization, e.g., configuring motors or sensors
            SpindexMotor = hardwareMap.get(DcMotorEx.class, "spindex");
            Color = hardwareMap.get(RevColorSensorV3.class, "color");
            SpindexTrigger = hardwareMap.get(Servo.class, "trigger");
            MagnetSensor = hardwareMap.get(DigitalChannel.class, "mag");


        }
        public static class RotateToNext implements Action{
            private static DcMotorEx SpindexMotor;
            private static DigitalChannel MagnetSensor;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                SpindexMotor.setPower(0.1);
                if(MagnetSensor.getState()) {
                    SpindexMotor.setPower(0);
                    return false;
                }else {return true}
                return false;
        }
        public static Action rotateToNext(){
                return new RotateToNext();
        }
            public static class IdentifyBall implements Action {
                private static RevColorSensorV3 Color;
                @Override
                public int run(@NonNull TelemetryPacket packet) {
                    int myColor = color.getNormalizedColors().toColor();
                    double hue = JavaUtil.rgbToHue(Color.red(myColor), Color.green(myColor), Color.blue(myColor));
                    //Purple
                    if (hue > 75 && hue < 95) {
                        return 1;
                    //Green
                    } else if (hue > 35 && hue < 55) {
                        return 2;
                    }else {return 0};
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


        public static class MotorTurn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false;
            }
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