package frc.robot.subsystems;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constElevator;

public class ElevatorSubsystem extends SubsystemBase {

  private TalonFX leftMotorFollower;
  private TalonFX rightMotorLeader;

  // Reef Zone 
  public final double L1 = Units.inchesToMeters(0.0); 
  public final double L2 = Units.inchesToMeters(0.0);
  public final double L3 = Units.inchesToMeters(0.0);
  public final double L4 = Units.inchesToMeters(0.0);

  // Current level of the elevator (L1, L2, L3, L4)
  public static int Current_Level = 1;
  
    public ElevatorSubsystem() {
      leftMotorFollower = new TalonFX(1); // Change id's ltr
      rightMotorLeader = new TalonFX(2); // ^ 
  
      rightMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
      leftMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
  
    }
    
    public Command zero() { //  Current zeroing to Home the elevator
      return this.run(() -> rightMotorLeader.setVoltage(-1.0))
        .until(() -> rightMotorLeader.getStatorCurrent().getValueAsDouble() > 20.0)
        .finallyDo(
          (interrupted) -> {
            rightMotorLeader.setVoltage(0.0);
            if (!interrupted) rightMotorLeader.setPosition(0.0);
          }
        );
    }
  

    
    public void setPosition(double setpoint) { // Set the position of the elevator (closed loop)
   
      double pidOutput = constElevator.ELEVATOR_PID.calculate(rightMotorLeader.getPosition().getValueAsDouble(), setpoint);
      double feedforwardOutput = constElevator.ELEVATOR_FEEDFORWARD.calculate(setpoint);
  
      double output = pidOutput + feedforwardOutput;
  
      rightMotorLeader.setControl(new PositionVoltage(output));
      leftMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), true));
    }
  

  // Stop the controllers
  public void stopControllers() {
    rightMotorLeader.set(0.0);
    leftMotorFollower.set(0.0);
  }

  // Reset the sensor position of the elevator
  public void resetSensorPosition(double setpoint) { 
    rightMotorLeader.setPosition(setpoint);
    leftMotorFollower.setPosition(setpoint);
  }

  @Override
  public void periodic() {
    //  nothin
  }

}

