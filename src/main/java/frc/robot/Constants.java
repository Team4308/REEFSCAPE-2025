package frc.robot;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
    // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
    // 0);
    // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
    // }
    public static final class Swerve {
        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds

        public static final class ReefHeadingAlign {
            public static double kP = 0.0;
            public static double kI = 0.0;
            public static double kD = 0.0;
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
        public static final int LED_PORT = 5;
        public static final int LED_LENGTH = 120;
        public static final double SIM_UPDATE_RATE = 0.02;  
    }

    public static class constElevator {
        public static TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();
        static {
            ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            ELEVATOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.radiansPerSecondToRotationsPerMinute(20);
            ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.rotationsToRadians(3);
            ELEVATOR_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;
            ELEVATOR_CONFIG.Slot0.kG = 0.3;
            ELEVATOR_CONFIG.Slot0.kS = 0.4;
            ELEVATOR_CONFIG.Slot0.kP = 1;
            ELEVATOR_CONFIG.Slot0.kI = 0.0;
            ELEVATOR_CONFIG.Slot0.kD = 0.0;
        }
      
        // Tunin
        public static final PIDController pidController = new PIDController(0.1, 0.0, 0.00);
        public static final ElevatorFeedforward feedforward = new ElevatorFeedforward(1, 0.05, 18.05, 0.01);

        // Elevator physical constants
        public static final double GEAR_RATIO = 6.222222;
        public static final double SPOOL_RADIUS = 1.76; // INCHES ( CHANGE )
        public static final double floorToEvevatorHeight = 10.0; // INCHES (CHANGE )
        public static double MAX_HEIGHT = Units.inchesToMeters(50.0);
        public static double MIN_HEIGHT = Units.inchesToMeters(0.1);
        // Reef Zone heights inches
        public static final double L1 = Units.inchesToMeters(18.0 - floorToEvevatorHeight + 2);
        public static final double L2 = Units.inchesToMeters(31.875 - floorToEvevatorHeight + 2);
        public static final double L3 = Units.inchesToMeters(47.652 - floorToEvevatorHeight + 2);
        public static double L4 = Units.inchesToMeters(MAX_HEIGHT);
        public static double ALGAE1 =  Units.inchesToMeters(1); // idk bro
        public static double ALGAE2 =  Units.inchesToMeters(1); // idk bro

        // Speed constants (in meters per second)
        public static final double L1Velocity = 1;    //Custom speed for L1
        public static final double MAX_SPEED = 0.02;    // Maximum safe speed: 0.05 m/s
        public static final double NORMAL_SPEED = 0.01;  // Normal operation: 0.03 m/s
        public static final double SLOW_SPEED = 0.005;    // Precise movement: 0.01 m/s
        
        // Convert speeds to motor RPS using spool circumference
        public static final double SPOOL_CIRCUMFERENCE = Math.PI * SPOOL_RADIUS * 0.0254; 
        public static final double MAX_MOTOR_RPS = (MAX_SPEED / SPOOL_CIRCUMFERENCE) * GEAR_RATIO;
        public static final double NORMAL_MOTOR_RPS = (NORMAL_SPEED / SPOOL_CIRCUMFERENCE) * GEAR_RATIO;
        public static final double SLOW_MOTOR_RPS = (SLOW_SPEED / SPOOL_CIRCUMFERENCE) * GEAR_RATIO;

        // Calibration constants - also slowed down
        public static final double CALIBRATION_VOLTAGE_DOWN = -0.3;  
        public static final double CALIBRATION_VOLTAGE_UP = 0.4;     

        // Calibration constants
        public static final double CURRENT_THRESHOLD = 20.0;         // Current spike threshold for detecting limits
    }

    public static class EndEffector {
        public static final double algaeDeadZone = 3;

        //Positions
        public static class algaePositions {
            public static final double minPosition = -90.0;
            public static final double maxPosition = 90;
            public static final double restPosition = -90;
            public static final double removeAlgaePosition = -45;
        }

        public static final double algaeArmTolerance = 3;

        public static class speeds {
            //arbitray values
            public static final double L1 = 5;
            public static final double L23 = 50;
            public static final double L4 = 25;

            public static final double maxAlgaeVelocity = 15;
            public static final double intake = 10;
            public static final double removeAlgae = -50;
        }

        //PID
        public static class PID {
            public static final double kP = 1;
            public static final double kI = 0;
            public static final double kD = 0;
        }

        //FeedForward
        public static class FeedForward {
            public static final double kS = 1;
            public static final double kG = 0.27;
            public static final double kV = 0.18;
            public static final double kA = 0.0;
        }
    }
}
