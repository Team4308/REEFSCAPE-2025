package frc.robot;

public class Constants {
    public static class LoggedDashboard {
        public static boolean tuningMode = false;
    }

    public static class Mapping {
        public static class Index {
            public static final int indexMotor = 1;
        }

        public static class Shooter {
            // these numbers are wrong
            public static final int motor = 13;
            public static final int beambrake = 0;
            public static final int encoder = 1;
            public static final int limitSwitch1 = 2;
            public static final int limitSwitch2 = 3;
        }
    }

    public static class Shooter {
        public static final int shooterStartDegree = 18;
        public static final int shooterEndDegree = 72;
        public static final double encoderStartRevolutions = 0.0;
        public static final double encoderEndRevolutions = -0.69;
        public static final class AngleControl {
            // these numbers are also wrong
            public static final double kP = 0.01;
            public static final double kI = 0;
            public static final double kD = 0.0001;
          }
    }
}
