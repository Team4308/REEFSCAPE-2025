package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import swervelib.math.Matter;

public final class Constants {

    public static final double ROBOT_MASS = 100 * 0.453592;
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED = Units.feetToMeters(15.1);
    // Maximum speed of the robot in meters per second, used to limit acceleration.

    public static final class LoggedDashboard {
        public static final boolean TUNING_MODE = false;
    }

    // public static final class AutonConstants
    // {
    //
    //  public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    //  public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
    // }
    public static final class Swerve {
        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds

        public static final class ReefHeadingAlign {
            public static double kP = 0.0;
            public static double kI = 0.0;
            public static double kD = 0.0;

            public static final double TOLERANCE = 1.0;
        }
    }

    public static class Operator {
        // Joystick Deadband
        public static final double DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
    }

    public static class GamePieces {
        public static final Pose3d kReefCenterBlue = new Pose3d(0.0, 0.0, 0.0, new Rotation3d());
        public static final Pose3d kReefCenterRed = new Pose3d(0.0, 0.0, 0.0, new Rotation3d());
    }

    public static class constLED {
        public static final int LED_LENGTH = 56;
        public static final int SCROLL_SPEED = 1;
        public static final int PATTERN_LENGTH = 7; 
        public static final int TRAIL_LENGTH = 3;
        public static final double SIM_UPDATE_RATE = 0.02;
    }

    public static class constElevator {
        public static TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();
        static {
            ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            ELEVATOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        public static final double TOLERANCE = 0.01; // m
        public static final double MAX_VELOCITY = 5.42; // m/s
        public static final double MAX_ACCELERATION = 11.91; // m/s^2

        // Tuning
        public static final ProfiledPIDController PID_CONTROLLER = new ProfiledPIDController(1.5, 0.0, 0.00,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION), 0.02);
        public static final ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(0.0, 0.40, 2.0, 0.0);

        // Elevator physical constants
        public static final double GEAR_RATIO = 175/36;
        public static final double SPOOL_CIRCUMFERENCE = Units.inchesToMeters(2 * Math.PI * 1.757);
        public static double MAX_HEIGHT = Units.inchesToMeters(64);
        public static double MIN_HEIGHT = Units.inchesToMeters(4.875);

        // Preset heights in inches
        public static final double L1 = Units.inchesToMeters(6.0);
        public static final double L2 = Units.inchesToMeters(20.0);
        public static final double L3 = Units.inchesToMeters(39.0);
        public static final double ALGAE1 = Units.inchesToMeters(39.0);
        public static final double ALGAE2 = Units.inchesToMeters(60.0);

        // Speed constants (in meters per second)
        public static final double ALGAE_REMOVAL_SPEED = 1;
    }

    public static class constEndEffector {
        public static class algaePivot {
            public static final ProfiledPIDController PID_CONTROLLER = new ProfiledPIDController(0.005, 0.0, 0.0,
                    new TrapezoidProfile.Constraints(180, 360), 0.02);
            public static final ArmFeedforward FEEDFORWARD = new ArmFeedforward(0.0, 0.32, 0.0035, 0.0);

            public static final double TOLERANCE = 15.0;

            public static final double ROTATION_TO_ANGLE_RATIO = 40.0;

            public static final double MIN_ANGLE = -90.0;
            public static final double MAX_ANGLE = 120;
            public static final double REMOVAL_ANGLE = -30;
        }

        public static class rollerSpeeds {    // m/s
            public static final double L1 = 3;
            public static final double L23 = 50;
            public static final double L4 = 10;

            public static final double CORAL_INTAKE = 10;
            public static final double ALGAE_REMOVAL = -50;
        }
    }

    public static class constSlapdown {
        // The angle that the pivot motor should be at when the arm is at the top.
        public static final double PIVOT_TOP_ANGLE = 90.0; // degrees

        // The angle that the pivot motor should be at when the arm is at the bottom.
        public static final double PIVOT_BOTTOM_ANGLE = 45.0; // degrees

        // The precentage output of the intake motor while it is running.
        public static final double INTAKE_SPEED = 0.2; // precentage
    }
}