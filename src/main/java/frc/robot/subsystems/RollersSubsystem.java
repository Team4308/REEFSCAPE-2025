package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class RollersSubsystem extends LogSubsystem {

    public final TalonFX motor;
    public boolean state;

    final VelocityVoltage velocity;
    private final TalonFXConfiguration configuration;

    public double maxSpeed;
    public double multiplier;

    public final DigitalInput limitSwitch1;
    public final DigitalInput limitSwitch2;

    public RollersSubsystem() {
        // Setup Controllers
        motor = new TalonFX(Constants.Mapping.Index.indexMotor);
        
        motor.setNeutralMode(NeutralModeValue.Brake);

        limitSwitch1 = new DigitalInput(Constants.Mapping.Shooter.limitSwitch1);
        limitSwitch2 = new DigitalInput(Constants.Mapping.Shooter.limitSwitch2);

        stopControllers();

        velocity = new VelocityVoltage(0);

        configuration = new TalonFXConfiguration();

        motor.getConfigurator().apply(configuration);

        maxSpeed = 100;
        multiplier = 1;
    }

    public void setMotorOutput(double rps) {
        velocity.Slot = 0;
        motor.setControl(velocity.withVelocity(rps*multiplier));

    }

    public void stopControllers() {
        motor.set(0.0);
    }

    @Override
    public Sendable log() {
        return this;
    }
}


