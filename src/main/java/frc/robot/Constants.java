package frc.robot;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class Constants {
    public static class LoggedDashboard {
        public static boolean tuningMode = false;
    }

    public static class constLED {
        public static final int LED_PORT = 5;
        public static final int LED_LENGTH = 120;
        public static final double SIM_UPDATE_RATE = 0.02;  
    }

    class Mapping {
    public static class Controllers {
        public static final int kStick = 0;
        public static final int kStick1 = 1;
    }
    }
    public static class constElevator {
        public static TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();
        static {
            ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            ELEVATOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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
        // Device Ids
        public static final int ELEVATOR_LEADER = 1;
        public static final int ELEVATOR_FOLLOWER = 2;
        // Tunin
        public static final PIDController pidController = new PIDController(0.1, 0.0, 0.00);
        public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 1, 0);

        // Elevator physical constants
        public static final double GEAR_RATIO = 6.222222;
        public static final double SPOOL_RADIUS = 1.76; // INCHES ( CHANGE )
        public static final double floorToEvevatorHeight = 8.0; // INCHES (CHANGE )
        // Reef Zone heights inches
        public static final double L1 = Units.inchesToMeters(18.0 - floorToEvevatorHeight); ;
        public static final double L2 = Units.inchesToMeters(31.875 - floorToEvevatorHeight);
        public static final double L3 = Units.inchesToMeters(47.652 - floorToEvevatorHeight);
        public static double L4 = Units.inchesToMeters(72 - floorToEvevatorHeight);

        public static final double MAX_SPEED = 2.0;  // Max 
        public static final double NORMAL_SPEED = 1.0;  // Normal 
        public static final double SLOW_SPEED = 0.5;   // Slow 
        
        public static final double MAX_MOTOR_RPS = MAX_SPEED * GEAR_RATIO;
        public static final double NORMAL_MOTOR_RPS = NORMAL_SPEED * GEAR_RATIO;
        public static final double SLOW_MOTOR_RPS = SLOW_SPEED * GEAR_RATIO;

        // Calibration constants
        public static final double CALIBRATION_VOLTAGE_DOWN = -0.5;  // Slow downward voltage for calibration
        public static final double CALIBRATION_VOLTAGE_UP = 0.75;    // Slow upward voltage for calibration
        public static final double CURRENT_THRESHOLD = 20.0;         // Current spike threshold for detecting limits

    }
}