package frc.robot;

public class Constants {
    public static class LoggedDashboard {
        public static boolean tuningMode = false;
    }

    public static class Mapping {
        public static final int controllerPort = 1; //xbox controller port, change if needed
    }

    public static class CoralRoller {
        public static final int coralRollerMotor = 0;
        //divided by 5 for safety @owen
        public static final double coralRollerSpeedL23 = 0.2;
        public static final double coralRollerSpeedL4 = 0.4;
    }

    public static class AlgaePivot {
        //Motor ID
        public static final int pivMotor = 1; //change if needed

        //Positions
        public static final double resting = 0.0; //pointing straight down
        public static final double fullyUp = 0.4; //estimate

        //PID
        public static final double kP = 0.0001; //NOT TUNED
        public static final double kI = 0.0;
        public static final double kD = 0.0001; //NOT TUNED
    }
}
