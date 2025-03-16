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

    public double targetAngle = constEndEffector.algaePivot.MAX_ANGLE;

    private double encoderOffset;

    public AlgaeArmSubsystem() {
        algaeMotor.setNeutralMode(NeutralModeValue.Brake);

        encoderOffset = -algaeMotor.getPosition().getValueAsDouble();

        stopControllers();
    }

    public double getAlgaePosition() {
        return (algaeMotor.getPosition().getValueAsDouble() + encoderOffset) * constEndEffector.algaePivot.ROTATION_TO_ANGLE_RATIO + constEndEffector.algaePivot.MAX_ANGLE;
    }

    public void goToTargetPosition() {
        double currentAngle = getAlgaePosition();

        double motorVoltage = constEndEffector.algaePivot.PID_CONTROLLER.calculate(currentAngle, targetAngle);

        double feedforwardOutput = constEndEffector.algaePivot.FEEDFORWARD.calculate(Math.toRadians(currentAngle),
                                                                                constEndEffector.algaePivot.PID_CONTROLLER.getSetpoint().velocity);

        // SmartDashboard.putNumber("Algae Arm Angle", currentAngle);
        // SmartDashboard.putNumber("Algae Arm Target", constEndEffector.algaePivot.PID_CONTROLLER.getSetpoint().position);

        algaeMotor.setVoltage(DoubleUtils.clamp(feedforwardOutput + motorVoltage, -12, 12));
    }

    public void setAlgaePosition(double degree) {
        targetAngle = DoubleUtils.clamp(degree, constEndEffector.algaePivot.MIN_ANGLE, constEndEffector.algaePivot.MAX_ANGLE);
    }

    public boolean isAtPosition() {
        return Math.abs(getAlgaePosition() - targetAngle) < constEndEffector.algaePivot.TOLERANCE;
    }

    @Override
    public void periodic() {
        goToTargetPosition();
        SmartDashboard.putBoolean("Pivot At Position", isAtPosition());
    }

    public void stopControllers() {
        algaeMotor.set(0);
    }

    public Sendable log() {
        return this;
    }
}