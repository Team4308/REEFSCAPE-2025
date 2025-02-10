package frc.robot.subsystems;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constElevator;

public class ElevatorSubsystem extends SubsystemBase {

  private TalonFX leftMotorFollower;
  private TalonFX rightMotorLeader;
  private double currentVelocityLimit = constElevator.NORMAL_MOTOR_RPS;

  public ElevatorSubsystem() {
    leftMotorFollower = new TalonFX(constElevator.ELEVATOR_LEADER); 
    rightMotorLeader = new TalonFX(constElevator.ELEVATOR_FOLLOWER); 

    rightMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    leftMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
  }
    
  public Command zero() { //  Zeroing to Home the elevator
    return this.run(() -> rightMotorLeader.setVoltage(-1.0))
      .until(() -> rightMotorLeader.getStatorCurrent().getValueAsDouble() > 20.0)
      .finallyDo(
        (interrupted) -> {
          rightMotorLeader.setVoltage(0.0);
          if (!interrupted) rightMotorLeader.setPosition(0.0);
        }
      );
  }
  
  public void setPosition(double setpointMeters) {
    setPosition(setpointMeters, currentVelocityLimit);
  }

  public void setPosition(double setpointMeters, double velocityRPS) {
    if (setpointMeters < 0 || setpointMeters > constElevator.MAX_HEIGHT) {
        throw new IllegalArgumentException("Setpoint out of range");
    }

    double motorRotations = DoubleUtils.mapRange(
        setpointMeters,
        0,
        constElevator.MAX_HEIGHT,
        0,
        constElevator.MAX_HEIGHT * constElevator.GEAR_RATIO
    );

    var positionVoltage = new PositionVoltage(motorRotations).withVelocity(velocityRPS);
    rightMotorLeader.setControl(positionVoltage);
    leftMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), true));
  }

  // Preset position commands
  public Command goToL1() {
    return this.runOnce(() -> setPosition(constElevator.L1));
  }

  public Command goToL2() {
    return this.runOnce(() -> setPosition(constElevator.L2));
  }

  public Command goToL3() {
    return this.runOnce(() -> setPosition(constElevator.L3));
  }

  public Command goToL4() {
    return this.runOnce(() -> setPosition(constElevator.L4));
  }

  // Speed control
  public void setNormalSpeed() {
    currentVelocityLimit = constElevator.NORMAL_MOTOR_RPS;
  }

  public void setSlowSpeed() {
    currentVelocityLimit = constElevator.SLOW_MOTOR_RPS;
  }

  public void setMaxSpeed() {
    currentVelocityLimit = constElevator.MAX_MOTOR_RPS;
  }

  public void setCustomSpeed(double speedMetersPerSecond) {
    currentVelocityLimit = speedMetersPerSecond * constElevator.GEAR_RATIO;
  }

  // Optional: Modified position commands with speed control
  public Command goToL1WithSpeed(double speed) {
    return this.runOnce(() -> setPosition(constElevator.L1, speed * constElevator.GEAR_RATIO));
  }

public double getPosition() {
    double motorRotations = rightMotorLeader.getPosition().getValueAsDouble();
    return motorRotations / constElevator.GEAR_RATIO;  
}

  public double getHeight() {
    return getPosition();  // Already returns in meters
  }

  public double getMaxHeight() {
    return constElevator.MAX_HEIGHT;
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

