package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

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
            private boolean initialized  = false;
            private final Integer targetRPM;
            private final double targetVelocityUnits;

            public MotorOn(Integer rpm) {
                this.targetRPM = rpm;
                this.targetVelocityUnits = rpm * 28.0 / 60.0;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    flywheelmotor.setVelocity(targetVelocityUnits);
                    initialized = true;
                    packet.put("Target RPM", targetRPM); // Useful telemetry
                    packet.put("Target Vel Units", targetVelocityUnits); // Useful telemetry
                }

                double vel = flywheelmotor.getVelocity();
                packet.put("RPM: ", vel);
                return vel < targetVelocityUnits * 0.9;
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
                packet.addLine("Flywheel Motor Off");
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

        public static class RotateToNext implements Action {
            private static DcMotorEx SpindexMotor;
            private static DigitalChannel MagnetSensor;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                SpindexMotor.setPower(0.1);
                if (MagnetSensor.getState()) {
                    SpindexMotor.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }

            public static Action rotateToNext() {
                return new RotateToNext();
            }
        }

        public static class IdentifyBall implements Action {
            private static RevColorSensorV3 Color;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                int myColor = Color.getNormalizedColors().toColor();
                double hue = JavaUtil.rgbToHue(Color.red(), Color.green(), Color.blue());
                //TODO: figure out how to return a integer
                if (hue > 75 && hue < 95) {
                    return true;//1; // Purple
                } else if (hue > 35 && hue < 55) {
                    return true; //2; // Green
                } else {
                    return true; //0;
                }
            }

            public static Action identifyBall() {
                return new IdentifyBall();
            }
        }

        public static class IdentifyOrder implements Action {
            private int state = 0;
            private int[] colors = new int[3];
            private long lastTime = 0;
            private boolean rotating = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                long currentTime = System.currentTimeMillis();
                if (state == 0) {
                    // Start
                    state = 1;
                    rotating = true;
                    SpindexMotor.setPower(0.1);
                    lastTime = currentTime;
                    return true;
                }

                if (rotating) {
                    if (MagnetSensor.getState()) {
                        SpindexMotor.setPower(0);
                        rotating = false;
                        //TODO: get the color sensor above to work and then call the function from here.
                        //colors[state - 1] = new IdentifyBall().run(packet);
                        packet.addLine("Ball " + (state) + " color: " + colors[state - 1]);
                        lastTime = currentTime;
                        return true;
                    }
                    return true;
                } else {
                    if (currentTime - lastTime > 500) { // Small delay
                        state++;
                        if (state > 3) {
                            // Check order, for example purple-green-other
                            boolean isGoodOrder = (colors[0] == 1 && colors[1] == 2 && colors[2] == 0);
                            packet.addLine("Order good: " + isGoodOrder);
                            return false; // Done
                        }
                        rotating = true;
                        SpindexMotor.setPower(0.1);
                        lastTime = currentTime;
                        return true;
                    }
                    return true;
                }
            }
        }

        public static Action identifyOrder() {
            return new IdentifyOrder();
        }

        public static class Sequencing implements Action {
            private int shotCount = 0;
            private long lastTime = 0;
            private int subState = 0; // 0: rotate, 1: trigger, 2: reset

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                long currentTime = System.currentTimeMillis();
                if (shotCount >= 3) {
                    return false; // Done after 3 shots
                }

                if (subState == 0) {
                    // Rotate to next
                    SpindexMotor.setPower(0.1);
                    if (MagnetSensor.getState()) {
                        SpindexMotor.setPower(0);
                        subState = 1;
                        lastTime = currentTime;
                    }
                    return true;
                } else if (subState == 1) {
                    // Trigger fire
                    if (currentTime - lastTime > 500) {
                        SpindexTrigger.setPosition(1.0); // Fire position
                        subState = 2;
                        lastTime = currentTime;
                    }
                    return true;
                } else if (subState == 2) {
                    // Reset trigger
                    if (currentTime - lastTime > 500) {
                        SpindexTrigger.setPosition(0.0); // Reset position
                        shotCount++;
                        subState = 0;
                        lastTime = currentTime;
                    }
                    return true;
                }
                return true;
            }
        }

        public static Action sequencing() {
            return new Sequencing();
        }

        public static class MotorTurn implements Action {
            private boolean started = false;
            private long startTime;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                long currentTime = System.currentTimeMillis();
                if (!started) {
                    SpindexMotor.setPower(0.5);
                    startTime = currentTime;
                    started = true;
                    return true;
                }
                if (currentTime - startTime > 2000) {
                    SpindexMotor.setPower(0);
                    return false;
                }
                return true;
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

// made by alix and vrishank barry did no work lol