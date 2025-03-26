package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.constEndEffector;
import frc.robot.Ports.EndEffector;
import frc.robot.Robot;

public class AlgaeArmSubsystem extends LogSubsystem {
    private TalonFX algaeMotor = new TalonFX(EndEffector.ALGAE_MOTOR);

    public double targetAngle = constEndEffector.algaePivot.REST_ANGLE;

    private double encoderOffset;
    public double totalVoltage;

    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                    null, // Use default timeout (10 s)
                          // Log state with Phoenix SignalLogger class
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> algaeMotor.set(totalVoltage),
                    null,
                    this));

    public AlgaeArmSubsystem() {
        algaeMotor.setNeutralMode(NeutralModeValue.Brake);

        constEndEffector.algaePivot.PID_CONTROLLER.enableContinuousInput(0, 360);

        resetSensors();
        stopControllers();
    }

    public double getAlgaePosition() {
        if (Robot.isSimulation()) {
            return Simulation.algaeAngleSimulation;
        }
        double value = (algaeMotor.getPosition().getValueAsDouble() + encoderOffset)
                * constEndEffector.algaePivot.ROTATION_TO_ANGLE_RATIO - 90;
        value += 360000000;
        return value % 360;
    }

    public void goToTargetPosition() {
        double currentAngle = getAlgaePosition();

        double pidOutput = constEndEffector.algaePivot.PID_CONTROLLER.calculate(currentAngle, targetAngle);

        double feedforwardOutput = constEndEffector.algaePivot.FEEDFORWARD.calculate(Math.toRadians(targetAngle),
                constEndEffector.algaePivot.PID_CONTROLLER.getSetpoint().velocity);

        totalVoltage = feedforwardOutput + pidOutput;
        totalVoltage = DoubleUtils.clamp(totalVoltage, -12, 12);
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
        if (Robot.isSimulation()) {
            return Simulation.algaeAtAngleSimulation;
        }
        return Math.abs(getAlgaePosition() - targetAngle) < constEndEffector.algaePivot.TOLERANCE;
    }

    public boolean isAtPosition2(double check) {
        if (Robot.isSimulation()) {
            return (Math.abs(Simulation.algaeAngleSimulation - check) < constEndEffector.algaePivot.TOLERANCE);
        }
        return Math.abs(getAlgaePosition() - check) < constEndEffector.algaePivot.TOLERANCE;
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
        encoderOffset = -algaeMotor.getPosition().getValueAsDouble(); // Starts & resets at straight town
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    public Sendable log() {
        return this;
    }
}