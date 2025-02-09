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

    public static class constElevator {
        public static TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();
        public static final PIDController ELEVATOR_PID = new PIDController(1.0, 0.0, 0.0);
        public static final SimpleMotorFeedforward ELEVATOR_FEEDFORWARD = new SimpleMotorFeedforward(1.0, 1.0);

        // Device Ids
        public static final int ELEVATOR_LEADER = 1;
        public static final int ELEVATOR_FOLLOWER = 2;


        // Elevator physical constants 
        public static final double GEAR_RATIO = 6.2222222;
        public static double MAX_HEIGHT = Units.inchesToMeters(72.0);
        
        // Reef Zone heights (arbitrary values do not use real inchs)
        public static final double L1 = Units.inchesToMeters(0.0);
        public static final double L2 = Units.inchesToMeters(24.0);
        public static final double L3 = Units.inchesToMeters(48.0);
        public static double L4 = Units.inchesToMeters(72.0);

        // Calibration 
        public static final double CALIBRATION_VOLTAGE_DOWN = -0.5;
        public static final double CALIBRATION_VOLTAGE_UP = 1.0;
        public static final double CURRENT_THRESHOLD = 20.0;

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
        }
    }
}
