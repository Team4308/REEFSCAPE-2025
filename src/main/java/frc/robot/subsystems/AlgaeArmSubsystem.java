package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.util.sendable.Sendable;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.constEndEffector;
import frc.robot.Ports.EndEffector;

public class AlgaeArmSubsystem extends LogSubsystem {
    private TalonFX algaeMotor = new TalonFX(EndEffector.ALGAE_MOTOR);

    public double targetAngle = constEndEffector.algaePivot.REST_ANGLE;

    private double encoderOffset;

    public AlgaeArmSubsystem() {
        algaeMotor.setNeutralMode(NeutralModeValue.Brake);

        constEndEffector.algaePivot.PID_CONTROLLER.enableContinuousInput(0, 360);

        resetSensors();
        stopControllers();
    }

    public double getAlgaePosition() {
        return (algaeMotor.getPosition().getValueAsDouble() + encoderOffset)
                * constEndEffector.algaePivot.ROTATION_TO_ANGLE_RATIO + constEndEffector.algaePivot.REST_ANGLE;
    }

    public void goToTargetPosition() {
        double currentAngle = getAlgaePosition();

        double motorVoltage = constEndEffector.algaePivot.PID_CONTROLLER.calculate(currentAngle, targetAngle);

        double feedforwardOutput = constEndEffector.algaePivot.FEEDFORWARD.calculate(Math.toRadians(currentAngle),
                constEndEffector.algaePivot.PID_CONTROLLER.getSetpoint().velocity);

        double totalVoltage = feedforwardOutput + motorVoltage;
        algaeMotor.setVoltage(totalVoltage);

        Logger.recordOutput("Subsystems/Algae/Target Angle", targetAngle);
        Logger.recordOutput("Subsystems/Algae/Current Angle", getAlgaePosition());
        Logger.recordOutput("Subsystems/Algae/Is At Angle", isAtPosition());
        Logger.recordOutput("Subsystems/Algae/Voltage", totalVoltage);
    }

    public void setAlgaePosition(double degree) {
        // continous rotation, pushes back to 360 if goes below 0
        targetAngle = (3600 + degree) % 360;
    }

    public boolean isAtPosition() {
        return Math.abs(getAlgaePosition() - targetAngle) < constEndEffector.algaePivot.TOLERANCE;
    }

    @Override
    public void periodic() {
        goToTargetPosition();
    }

    public void stopControllers() {
        algaeMotor.set(0);
    }

    public void resetSensors() {
        stopControllers();
        encoderOffset = -algaeMotor.getPosition().getValueAsDouble();
    }

    public Sendable log() {
        return this;
    }
}