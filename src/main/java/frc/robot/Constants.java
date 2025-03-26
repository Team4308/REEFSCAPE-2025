package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import swervelib.math.Matter;

public final class Constants {

    public static final double ROBOT_MASS = 123 * 0.453592;
    public static final Matter CHASSIS = new Matter(
            new Translation3d(Units.inchesToMeters(27), Units.inchesToMeters(27), Units.inchesToMeters(40)),
            ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED = Units.feetToMeters(12.5);
    // Maximum speed of the robot in meters per second, used to limit acceleration.

    public static final class LoggedDashboard {
        public static final boolean TUNING_MODE = false;
    }

    public static final class Swerve {
        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds

        public static final class Align {
            public static final class Heading {
                public static final double TOLERANCE = 2.5; // in degrees
            }

            public static final class Translation {
                public static final double TOLERANCE = Units.inchesToMeters(1.0);
            }
        }

        public static class Auton {
            public static class Angle {
                public static final double kP = 5.0;
                public static final double kI = 0.0;
                public static final double kD = 0.0;
            }

            public static class Translation {
                public static final double kP = 2.5;
                public static final double kI = 0.0;
                public static final double kD = 0.0;
            }
        }
    }

    public static class Driver {
        // Joystick Deadband
        public static final double DEADBAND = 0.0;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
    }

    public static class constLED {
        public static final int LED_PORT = 9;
        public static final int Elevator_Length = 48;
        public static final int Funnel_Vert_Length = 46;
        public static final int Funnel_Hori_Length = 22;
        public static final boolean debugMode = true; // Allows AbosoluteLib to print debug messages, And uses smart dash 
        public static final int LED_LENGTH = Elevator_Length + Funnel_Vert_Length + Funnel_Hori_Length;

        public static final Pair<Integer, Integer> Elevator_Ends = new Pair<Integer, Integer>(0, Elevator_Length - 1);
        public static final Pair<Integer, Integer> Funnel_Vert_Ends = new Pair<Integer, Integer>(Elevator_Length,
                Elevator_Length + Funnel_Vert_Length - 1);
        public static final Pair<Integer, Integer> Funnel_Hori_Ends = new Pair<Integer, Integer>(
                Elevator_Length + Funnel_Vert_Length, LED_LENGTH - 1);

        // Idle pattern speficly
        public static final int SCROLL_SPEED = 1;
        public static final int PATTERN_LENGTH = 4;
        public static final int TRAIL_LENGTH = 2;
        // Sim
        public static final double SIM_UPDATE_RATE = 0.02;
        public static final double STATUS_LIGHT_DURATION = 2.0;
    }

    public static class constElevator {
        public static TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();
        static {
            ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            ELEVATOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        public static final double TOLERANCE = 0.015; // m
        public static final double MAX_VELOCITY = 5.0; // m/s
        public static final double MAX_ACCELERATION = 10.0; // m/s^2

        // Controllers
        public static final ProfiledPIDController PID_CONTROLLER = new ProfiledPIDController(2.6, 0.0, 0.0,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
        public static final ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(0.0, 0.38, 2.3, 0.0);

        // Elevator physical constants
        public static final double GEAR_RATIO = 175 / 36;
        public static final double SPOOL_CIRCUMFERENCE = Units.inchesToMeters(Math.PI * 2.53); // Random number that
                                                                                               // works
        public static double MAX_HEIGHT = Units.inchesToMeters(46.875);
        public static double MIN_HEIGHT = Units.inchesToMeters(4.875);

        // Preset heights in inches
        public static final double L1 = Units.inchesToMeters(6.5);
        public static final double L2 = Units.inchesToMeters(23.0);
        public static final double L3 = Units.inchesToMeters(39.0);
        public static final double ALGAE1 = Units.inchesToMeters(30.0);
        public static final double ALGAE2 = Units.inchesToMeters(43.0);

        public static final double ALGAE1_PREMOVE = Units.inchesToMeters(28);
        public static final double ALGAE2_PREMOVE = Units.inchesToMeters(41);

        // Speed constants (in meters per second)
        public static final double ALGAE_REMOVAL_SPEED = 3;

        public static final double TIMEOUT_SECONDS = 3;
    }

    public static class constEndEffector {
        public static class algaePivot {
            public static final double TOLERANCE = 20.0;

            public static final double ROTATION_TO_ANGLE_RATIO = 18.3333333;

            // Controllers
            public static final ProfiledPIDController PID_CONTROLLER = new ProfiledPIDController(0.00, 0.0, 0.0,
                    new TrapezoidProfile.Constraints(1000, 7200)); // degrees
            public static final ArmFeedforward FEEDFORWARD = new ArmFeedforward(0.0, 0.12, 0.33, 0.0);

            // Angles (degrees)
            public static final double REST_ANGLE = 90.0; // Straight Up
            public static final double REMOVAL_ANGLE_TOP = 330;
            public static final double REMOVAL_ANGLE_BOTTOM = 60;

            public static final double TIMEOUT_SECONDS = 3;
        }

        public static class rollerSpeeds { // m/s
            public static final double L1 = 3;
            public static final double L23 = 80;

            public static final double DEFAULT_CORAL = 10;
            public static final double ALGAE_REMOVAL_BOTTOM = 50;
            public static final double ALGAE_REMOVAL_TOP = -50;
        }
    }
}