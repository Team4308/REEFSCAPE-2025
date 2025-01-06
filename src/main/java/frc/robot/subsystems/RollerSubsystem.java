package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import ca.team4308.absolutelib.wrapper.MotoredSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Constants;

public class RollerSubsystem extends MotoredSubsystem {
    public final TalonSRX motor;
    public boolean state;

    public RollerSubsystem() {
        // Setup Controllers
        motor = new TalonSRX(Constants.Mapping.Roller.motor);
        
        motor.setNeutralMode(NeutralMode.Brake);

        stopControllers();
    }

    public void setOutput(double output) {
        motor.set(TalonSRXControlMode.PercentOutput, output);
    }

    public void stopControllers() {
        motor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

    @Override
    public Sendable log() {
        return this;
    }
}