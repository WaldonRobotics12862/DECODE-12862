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

public class DigActions {
    public static class Launcher {

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

    }
    public static class Parking {
        public static class ParkServo {
            public ParkServo(HardwareMap hardwareMap) {
                // Servo initialization, e.g., configuring servo motor
            }
            public static class ServoPark implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    return false;
                }
            }
            public static Action servoPark() {
                return new ServoPark();
            }
        }
    }
}