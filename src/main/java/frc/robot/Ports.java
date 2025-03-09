package frc.robot;

public class Ports {
    /*** SUBSYSTEM IDS ***/

    // Controllers
    public static class Joysticks {
        public static final int DRIVER = 0;
        public static final int OPERATOR = 1;
    }

    // LED
    public static class LED {
        public static final int PORT = 0;
    }

    // Elevator
    public static class Elevator {
        public static final int ELEVATOR_MASTER = 10;
        public static final int ELEVATOR_FOLLOWER = 11;
        public static final int LIMIT_SWITCH_TOP = 0;
        public static final int LIMIT_SWITCH_BOTTOM = 1;
    }

    // End effector
    public static class EndEffector {
        public static final int CORAL_MOTOR = 12;
        public static final int INDEX_BEAM_BREAK = 2;
        public static final int ALGAE_MOTOR = 13;
    }

    // Algae Slapdown
    public static class AlgaeSlapdown {
        public static final int ALGAE_PIVOT = 14;
        public static final int ALGAE_ROLLER = 15;
        public static final int ALGAE_ENCODER = 69; // change
    }
}