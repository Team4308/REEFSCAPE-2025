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
