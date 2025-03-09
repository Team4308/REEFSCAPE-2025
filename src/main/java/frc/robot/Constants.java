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

    public static final double ROBOT_MASS = 125 * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED = Units.feetToMeters(14.5);
    // Maximum speed of the robot in meters per second, used to limit acceleration.

    public static final class LoggedDashboard {
        public static final boolean tuningMode = false;
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
        }

        public static final double alignTolerance = 1.0;
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
        public static final int LED_PORT = 0;
        public static final int LED_LENGTH = 56;
        public static final double SIM_UPDATE_RATE = 0.02;
    }

    public static class constElevator {
        public static TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();
        static {
            ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            ELEVATOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        public static final double tolerance = 0.01; // meters
        public static final double maxVelocity = 5.42; // m/s
        public static final double maxAcceleration = 11.91; // m/s^2

        // Tunin
        public static final ProfiledPIDController pidController = new ProfiledPIDController(1.5, 0.0, 0.00,
                new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration), 0.02);
        public static final ElevatorFeedforward feedforward = new ElevatorFeedforward(0.0, 0.40, 2.0, 0.0);

        // Elevator physical constants
        public static final double GEAR_RATIO = 175/36;
        public static final double SPOOL_CIRCUMFERENCE = Units.inchesToMeters(2 * Math.PI * 1.757);
        public static double MAX_HEIGHT = Units.inchesToMeters(64);
        public static double MIN_HEIGHT = Units.inchesToMeters(4.875);
        // Reef Zone heights inches
        public static final double L1 = Units.inchesToMeters(6.0);
        public static final double L2 = Units.inchesToMeters(20.0);
        public static final double L3 = Units.inchesToMeters(39.0);
        public static final double L4 = MAX_HEIGHT;
        public static final double ALGAE1 = Units.inchesToMeters(39.0);
        public static final double ALGAE2 = Units.inchesToMeters(60.0);

        // Speed constants (in meters per second)
        public static final double ALGAE_REMOVAL_SPEED = 1;
    }

    public static class EndEffector {

        public static final ProfiledPIDController algaePID = new ProfiledPIDController(0.005, 0.0, 0.0,
            new TrapezoidProfile.Constraints(180, 360), 0.02);
        public static final ArmFeedforward algaeFeedforward = new ArmFeedforward(0.0, 0.32, 0.0035, 0.0);

        // Positions
        public static class algaePositions {
            public static final double minPosition = -90.0;
            public static final double maxPosition = 120;
            public static final double removeAlgaePosition = -30;
        }

        public static final double algaeArmTolerance = 15;

        public static class speeds {
            // arbitray values
            public static final double L1 = 3;
            public static final double L23 = 50;
            public static final double L4 = 10;

            public static final double intake = 10;
            public static final double removeAlgae = -50;
        }
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