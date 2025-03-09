package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Constants.EndEffector;

public class AlgaeArmSubsystem extends LogSubsystem {
    private TalonFX algaeMotor = new TalonFX(Ports.EndEffector.ALGAE_MOTOR);

    private double targetAngle = Constants.EndEffector.algaePositions.minPosition;

    private double encoderOffset = 0.0;

    public AlgaeArmSubsystem() {
        var config = new Slot0Configs();
        config.kS = 0.1; // Add 0.1 V output to overcome static friction
        config.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        config.kP = 0.11; // An error of 1 rps results in 0.11 V output
        config.kI = 0; // no output for integrated error
        config.kD = 0;
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        algaeMotor.getConfigurator().apply(configuration);
        algaeMotor.getConfigurator().apply(config);

        encoderOffset = -algaeMotor.getPosition().getValueAsDouble();

        EndEffector.algaePID.setTolerance(EndEffector.algaeArmTolerance);

        stopControllers();
    }

    public double getAlgaePosition() {
        return (algaeMotor.getPosition().getValueAsDouble() + encoderOffset) * EndEffector.rotationToAngleRatio - 90;
    }

    public void goToTargetPosition() {
        double currentAngle = getAlgaePosition();

        double motorVoltage = EndEffector.algaePID.calculate(currentAngle, targetAngle);

        double feedforwardOutput = EndEffector.algaeFeedforward.calculate(Math.toRadians(currentAngle),
                EndEffector.algaePID.getSetpoint().velocity);

        SmartDashboard.putNumber("Algae Arm Angle", currentAngle);
        SmartDashboard.putNumber("Algae Arm Target", EndEffector.algaePID.getSetpoint().position);

        algaeMotor.setVoltage(DoubleUtils.clamp(feedforwardOutput + motorVoltage, -12, 12));
    }

    public void setAlgaePosition(double degree) {
        targetAngle = DoubleUtils.clamp(degree, Constants.EndEffector.algaePositions.minPosition,
                Constants.EndEffector.algaePositions.maxPosition);
    }

    public double getCurrentTarget() {
        return targetAngle;
    }

    @Override
    public void periodic() {
        goToTargetPosition();
    }

    public void stopControllers() {
        algaeMotor.set(0);
    }

    public Sendable log() {
        return this;
    }
}