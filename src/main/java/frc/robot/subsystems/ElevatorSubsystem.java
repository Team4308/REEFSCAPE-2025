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

public class ElevatorSubsystem extends LogSubsystem {

    private final TalonFX motor1 = new TalonFX(Constants.Mapping.Elevator.motor1);
    private final TalonFX motor2 = new TalonFX(Constants.Mapping.Elevator.motor2);

    VelocityVoltage rollerVelocity = new VelocityVoltage(0);
    VelocityVoltage algaeVelocity = new VelocityVoltage(0);

    DigitalInput topBeamBreak = new DigitalInput(Constants.Mapping.Elevator.topBeamBreak);
    DigitalInput botBeamBreak = new DigitalInput(Constants.Mapping.Elevator.botBeamBreak);

    public ElevatorSubsystem() {
        var config = new Slot0Configs();
        config.kS = 0.1; // Add 0.1 V output to overcome static friction
        config.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        config.kP = 0.11; // An error of 1 rps results in 0.11 V output
        config.kI = 0; // no output for integrated error
        config.kD = 0;
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor1.getConfigurator().apply(configuration);
        motor1.getConfigurator().apply(config);
        motor2.getConfigurator().apply(configuration);
        motor2.getConfigurator().apply(config);
        rollerVelocity.Slot = 0;
        algaeVelocity.Slot = 0;

        stopControllers();
    }

    public void setMotorSpeed(double velocity){
        if (botBeamBreak.get() && velocity > 0) {
            velocity = 0;
        }
        if (topBeamBreak.get() && velocity < 0) {
            velocity = 0;
        }
        
        motor1.setControl(rollerVelocity.withVelocity(-velocity));
        motor2.setControl(rollerVelocity.withVelocity(-velocity));
    }

    public void stopControllers() {
        motor1.set(0);
        motor2.set(0);
    }

    public Sendable log() {
        return this;
    }
}