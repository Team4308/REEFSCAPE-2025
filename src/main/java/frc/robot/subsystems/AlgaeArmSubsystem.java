package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.control.XBoxWrapper;
import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;

public class AlgaeArmSubsystem extends LogSubsystem {
    private TalonFX algaeMotor = new TalonFX(Ports.EndEffector.ALGAE_MOTOR);

    private PIDController algaepidController = new PIDController(Constants.EndEffector.PID.kP,
            Constants.EndEffector.PID.kI,
            Constants.EndEffector.PID.kD);
    private ArmFeedforward algaeFeedForward = new ArmFeedforward(Constants.EndEffector.FeedForward.kS,
            Constants.EndEffector.FeedForward.kG, Constants.EndEffector.FeedForward.kV,
            Constants.EndEffector.FeedForward.kA);

    private CANcoder canCoder = new CANcoder(Ports.EndEffector.ALGAE_CANCODER);

    private double targetAngle = Constants.EndEffector.algaePositions.restPosition;

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

        stopControllers();
    }

    double tempSim = 0;

    public double getAlgaePosition() {
        if (Robot.isSimulation()) {
            tempSim -= new XBoxWrapper(1).getRightY() / 10;
            return tempSim;
        }
        return algaeMotor.getPosition().getValueAsDouble() * 360d / 5;
        // return canCoder.getPosition().getValueAsDouble() * 360d;
    }

    public void goToTargetPosition() {
        // 0 Degrees is always parallel to the ground
        double currentAngle = getAlgaePosition();

        double motorVoltage = algaepidController.calculate(currentAngle, targetAngle);
        motorVoltage = DoubleUtils.clamp(motorVoltage, -Constants.EndEffector.speeds.maxAlgaeVelocity,
                Constants.EndEffector.speeds.maxAlgaeVelocity);

        double feedforwardOutput = algaeFeedForward.calculate(Math.toRadians(currentAngle),
                Constants.EndEffector.speeds.maxAlgaeVelocity);

        SmartDashboard.putNumber("algaepid", motorVoltage);
        SmartDashboard.putNumber("algaefeedforward", feedforwardOutput);
        SmartDashboard.putNumber("totalvoltage", feedforwardOutput + motorVoltage);
        SmartDashboard.putNumber("encoderalgae", currentAngle);
        SmartDashboard.putNumber("targetalgae", targetAngle);

        // algaeMotor.setVoltage(feedforwardOutput + motorVoltage);
    }

    public void setAlgaePosition(double degree) {
        targetAngle = DoubleUtils.clamp(degree, Constants.EndEffector.algaePositions.minPosition,
                Constants.EndEffector.algaePositions.maxPosition);
    }

    public boolean isAtPosition() {
        return Math.abs(getAlgaePosition() - targetAngle) < Constants.EndEffector.algaeArmTolerance;
    }

    public void editAlgaePosition(double difference) {
        targetAngle += difference;
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