package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.wrapper.MotoredSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class AlgaePivotSubsystem extends MotoredSubsystem {
    public final TalonFX pivotMotor;
    public final PIDController pidController;
    final VelocityVoltage velocity = new VelocityVoltage(0);
    final DutyCycleOut motorOut = new DutyCycleOut(0);
    final TalonFXConfiguration config = new TalonFXConfiguration();

    public AlgaePivotSubsystem() {
        pivotMotor = new TalonFX(Constants.AlgaePivot.pivMotor);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        pivotMotor.getConfigurator().apply(config);
        pidController = new PIDController(Constants.AlgaePivot.kP, Constants.AlgaePivot.kI, Constants.AlgaePivot.kD);

        //pivotMotor.setPosition(Constants.AlgaePivot.resting);
    }

    public double getPosition() {
        return pivotMotor.getPosition().getValueAsDouble() * 360d;
    }

    public void setPosition(double degree) {
        double target = DoubleUtils.clamp(degree, Constants.AlgaePivot.resting, Constants.AlgaePivot.fullyUp);
        double current = getPosition();

        double output = DoubleUtils.clamp(pidController.calculate(current, target), -1, 1);

        SmartDashboard.putNumber("Target Degrees", target);
        SmartDashboard.putNumber("Current Degrees", current);
        SmartDashboard.putNumber("Pivot Output", output);
        setOutput(output);
    }


    @Override
    public void stopControllers() {
        motorOut.Output = 0;
        pivotMotor.setControl(motorOut);
    }

    public void setOutput(double rps) {
        pivotMotor.setControl(velocity.withVelocity(rps));
    }

    @Override
    public Sendable log() {
        return this;
    }
 
    public Command runPivotCommand(double rps) {
        return this.run(() -> setOutput(rps));
    }
    
}
