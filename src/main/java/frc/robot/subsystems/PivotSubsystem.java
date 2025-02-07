package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;;
public class PivotSubsystem extends LogSubsystem {

    public final TalonFX motor;
    public final PIDController pidController;
    private final CANcoder canCoder; 


    public PivotSubsystem() {
        motor = new TalonFX(Constants.Mapping.Shooter.motor);
        
        canCoder = new CANcoder(Constants.Mapping.Shooter.encoder);

        motor.setNeutralMode(NeutralModeValue.Brake);

        pidController = new PIDController(Constants.Shooter.AngleControl.kP, Constants.Shooter.AngleControl.kI, Constants.Shooter.AngleControl.kD);//pid not tuned

        stopControllers();
    }

    public void setMotorOutput(double percent){
        motor.set(percent);
    }

    public void setMotorPosition(double degree) { 
        SmartDashboard.putNumber("encoderDegree", canCoder.getPosition().getValueAsDouble());
        
        double wantedDegree = DoubleUtils.clamp(degree, Constants.Shooter.shooterStartDegree, Constants.Shooter.shooterEndDegree);

        double shooterDegree = DoubleUtils.mapRangeNew(canCoder.getPosition().getValueAsDouble(), Constants.Shooter.encoderStartRevolutions, Constants.Shooter.encoderEndRevolutions, Constants.Shooter.shooterStartDegree, Constants.Shooter.shooterEndDegree);

        double motorOutput = -DoubleUtils.clamp(pidController.calculate(shooterDegree, wantedDegree), -1.0, 1.0);

        SmartDashboard.putNumber("shooterDegree", shooterDegree);
        SmartDashboard.putNumber("wantedDegree", wantedDegree);
        SmartDashboard.putNumber("motorOutput", motorOutput);

        setMotorOutput(motorOutput);
    }

        public void stopControllers() {
        motor.set(0.0);
    }

    public Sendable log() {
        return this;
    }
}
