package org.firstinspires.ftc.teamcode;

public class DigActions {
    public static class Launcher{

    }
    public static class Sensors{

    }
    public static class Intake{
        public Intake (HardwareMap hardwareMap){
            // 1 Motor, 2 Direction
        }
        public static class IntakeOn implements Actions {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false;

            }
        }
            public static Action intakeOn(){
                return new IntakeOn();
            }
        }
        public static class IntakeOff implements Actions {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false;

            }
        }
            public static Action intakeOff(){
                return new IntakeOff();
            }
        }
        public static class SpitOut implements Actions {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false;

            }

            public static Action spitOut() {
                return new SpitOut();
            }
        }
    }
    public static class Hopper{

    }
    public static class Parking{

    }
}
