package frc.robot;

public class Constants {
    public static class LoggedDashboard {
        public static boolean tuningMode = false;
    }

    public static class Slapdown {
        // The angle that the pivot motor should be at when the arm is at the top.
        public static final double PIVOT_TOP_ANGLE = 0.0; // revolutions

        // The angle that the pivot motor should be at when the arm is at the bottom.
        public static final double PIVOT_BOTTOM_ANGLE = 0.2; // revolutions

        // The rotation rate of the intake motor while it is running.
        public static final double INTAKE_SPEED = 0.2; // rotations per second
    }
}
