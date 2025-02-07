package frc.robot.subsystems;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constElevator;

public class ElevatorSubsystem extends SubsystemBase {

  private TalonFX leftMotorFollower;
  private TalonFX rightMotorLeader;

  // Constants for the elevator
  public static final double gearRatio = 6.222222; // Change to proper gear ratio please
  public static final double maxHeight = Units.inchesToMeters(72.0);  // Not final! Please change later

  // Reef Zone 
  public final double L1 = Units.inchesToMeters(0.0); 
  public final double L2 = Units.inchesToMeters(24.0);  // Not real heights please change later ( I just guessed )
  public final double L3 = Units.inchesToMeters(48.0);
  public final double L4 = Units.inchesToMeters(72.0);

  
  public ElevatorSubsystem() {
    leftMotorFollower = new TalonFX(1); // Change later to the correct 
    rightMotorLeader = new TalonFX(2); // 

    rightMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    leftMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
  }
    
  public Command zero() { //  zeroing to Home the elevator
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

    if (setpoint < 0 || setpoint > maxHeight) {
      throw new IllegalArgumentException("Setpoint out of range"); // Ensure we dont overextend the elevator
    }

    double adjustedSetpoint = setpoint / gearRatio; // Not 100% sure if this is the right way of doing it
    double pidOutput = constElevator.ELEVATOR_PID.calculate(rightMotorLeader.getPosition().getValueAsDouble(), adjustedSetpoint);
    double feedforwardOutput = constElevator.ELEVATOR_FEEDFORWARD.calculate(adjustedSetpoint);

    double output = pidOutput + feedforwardOutput;

    rightMotorLeader.setControl(new PositionVoltage(output));
    leftMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), true));
  }

  // Returns pos of leading motor 
  public Pose3d getPosition() {
    double currentPosition = rightMotorLeader.getPosition().getValueAsDouble() * gearRatio; 
    return new Pose3d(currentPosition, 0, 0, new Rotation3d());
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
    // Nothin
  }

}

