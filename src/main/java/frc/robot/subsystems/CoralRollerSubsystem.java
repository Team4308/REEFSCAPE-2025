package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.wrapper.MotoredSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class CoralRollerSubsystem extends MotoredSubsystem {
    public final TalonFX motor;
    final VelocityVoltage velocity = new VelocityVoltage(0);
    final DutyCycleOut motorOut = new DutyCycleOut(0);
    final TalonFXConfiguration config = new TalonFXConfiguration();

    public CoralRollerSubsystem() {
        motor = new TalonFX(Constants.Mapping.coralRollerMotor);
        motor.setNeutralMode(NeutralModeValue.Brake);

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        motor.getConfigurator().apply(config);
    }

    @Override
    public void stopControllers() {
        motorOut.Output = 0;
        motor.setControl(motorOut);
    }

    public void setOutput(double rps) {
        motor.setControl(velocity.withVelocity(rps));
    }

    @Override
    public Sendable log() {
        return this;
    }

    public Command runRollerCommand(double rps) {
        return this.run(() -> setOutput(rps));
    }
    
}
