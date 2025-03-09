package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.constEndEffector;
import frc.robot.Ports.EndEffector;

public class AlgaeArmSubsystem extends LogSubsystem {
    private TalonFX algaeMotor = new TalonFX(EndEffector.ALGAE_MOTOR);

    private double targetAngle = constEndEffector.algaePositions.minPosition;

    private double encoderOffset = 0.0;

    public AlgaeArmSubsystem() {
        algaeMotor.setNeutralMode(NeutralModeValue.Brake);

        encoderOffset = -algaeMotor.getPosition().getValueAsDouble();

        constEndEffector.algaePID.setTolerance(constEndEffector.algaeArmTolerance);

        stopControllers();
    }

    public double getAlgaePosition() {
        return (algaeMotor.getPosition().getValueAsDouble() + encoderOffset) * constEndEffector.rotationToAngleRatio + constEndEffector.algaePositions.minPosition;
    }

    public void goToTargetPosition() {
        double currentAngle = getAlgaePosition();

        double motorVoltage = constEndEffector.algaePID.calculate(currentAngle, targetAngle);

        double feedforwardOutput = constEndEffector.algaeFeedforward.calculate(Math.toRadians(currentAngle),
                constEndEffector.algaePID.getSetpoint().velocity);

        SmartDashboard.putNumber("Algae Arm Angle", currentAngle);
        SmartDashboard.putNumber("Algae Arm Target", constEndEffector.algaePID.getSetpoint().position);

        algaeMotor.setVoltage(DoubleUtils.clamp(feedforwardOutput + motorVoltage, -12, 12));
    }

    public void setAlgaePosition(double degree) {
        targetAngle = DoubleUtils.clamp(degree, constEndEffector.algaePositions.minPosition,
                constEndEffector.algaePositions.maxPosition);
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