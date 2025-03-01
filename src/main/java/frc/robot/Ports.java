package frc.robot;

public class Ports {
     /*** SWERVE MODULE PORTS ***/

    /*  
    Swerve Modules go:
        0 1
        2 3
    */
    public class Swerve {
        public static final int FL_DRIVE = 0; 
        public static final int FL_ROTATION = 0;
        public static final int FL_ENCODER = 0; 

        public static final int FR_DRIVE = 0; 
        public static final int FR_ROTATION = 0;
        public static final int FR_ENCODER = 0; 

        public static final int BL_DRIVE = 0;
        public static final int BL_ROTATION = 0;
        public static final int BL_ENCODER = 0; 

        public static final int BR_DRIVE = 0;
        public static final int BR_ROTATION = 0;
        public static final int BR_ENCODER = 0; 
    }

    /*** SUBSYSTEM IDS ***/

    //Controllers
    public static class Joysticks {
        public static final int DRIVER = 0;
        public static final int OPERATOR = 1;
    }

    // Elevator
    public static class Elevator {
        public static final int ELEVATOR_MASTER = 2;
        public static final int ELEVATOR_FOLLOWER = 1;
        public static final int LIMIT_SWITCH_TOP = 0;
        public static final int LIMIT_SWITCH_BOTTOM = 1;
        public static final int ELEVATOR_CANCODER = -1;
    }

    // End effector
    public static class EndEffector {
        public static final int CORAL_MOTOR = 3;
        public static final int INDEX_BEAM_BREAK = 2;
        public static final int ALGAE_MOTOR = 4;
        public static final int ALGAE_CANCODER = -1;
    }

    // Algae Slapdown
    public static class AlgaeSlapdown {
        public static final int ALGAE_PIVOT = -1;
        public static final int ALGAE_ROLLER = -1;
        public static final int ALGAE_ENCODER = -1;
    }

    // Pigeon
    public static class Pigeon {
        public static final int PIGEON = -1;
    }
}