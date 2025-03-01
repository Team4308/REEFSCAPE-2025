package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import ca.team4308.absolutelib.wrapper.LogSubsystem;
import frc.robot.Constants;
import frc.robot.Ports;

public class EndEffectorSubsystem extends LogSubsystem {

    private final TalonFX rollerMotor = new TalonFX(Ports.EndEffector.CORAL_MOTOR);
    private final TalonFX algaeMotor = new TalonFX(Ports.EndEffector.ALGAE_MOTOR);

    VelocityVoltage rollerVelocity = new VelocityVoltage(0);
    VelocityVoltage algaeVelocity = new VelocityVoltage(0);

    DigitalInput beamBreak = new DigitalInput(Ports.EndEffector.INDEX_BEAM_BREAK);

    public EndEffectorSubsystem() {
        var config = new Slot0Configs();
        config.kS = 0.1; // Add 0.1 V output to overcome static friction
        config.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        config.kP = 0.11; // An error of 1 rps results in 0.11 V output
        config.kI = 0; // no output for integrated error
        config.kD = 0;
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rollerMotor.getConfigurator().apply(configuration);
        rollerMotor.getConfigurator().apply(config);
        algaeMotor.getConfigurator().apply(configuration);
        algaeMotor.getConfigurator().apply(config);
        rollerVelocity.Slot = 0;
        algaeVelocity.Slot = 0;

        stopControllers();
    }

    public void setRollerOutput(double velocity){

        rollerMotor.setControl(rollerVelocity.withVelocity(velocity));
    }

    public void runBeamBreak() {
        if (!beamBreak.get()) {
            rollerMotor.setControl(algaeVelocity.withVelocity(5));
        }
    }

    public void setArmOutput(double velocity){
        algaeMotor.setControl(algaeVelocity.withVelocity(velocity));
    }

    public void stopControllers() {
        rollerMotor.set(0.0);
        algaeMotor.set(0);
    }

    public Sendable log() {
        return this;
    }
}