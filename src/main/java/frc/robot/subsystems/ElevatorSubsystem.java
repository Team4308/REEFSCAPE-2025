package frc.robot.subsystems;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constElevator;
import frc.robot.subsystems.LEDSystem;
public class ElevatorSubsystem extends SubsystemBase {

  private final LEDSystem m_ledSystem;
  private TalonFX leftMotorFollower;
  private TalonFX rightMotorLeader;
  private double currentVelocityLimit = constElevator.NORMAL_MOTOR_RPS;

  public ElevatorSubsystem(LEDSystem ledSystem) {
    m_ledSystem = ledSystem;
    leftMotorFollower = new TalonFX(constElevator.ELEVATOR_LEADER); 
    rightMotorLeader = new TalonFX(constElevator.ELEVATOR_FOLLOWER); 
    rightMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    leftMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
  }
    

  public void setPosition(double setpointMeters) {
    setPosition(setpointMeters, currentVelocityLimit);
  }

  public void setPosition(double setpointMeters, double velocityRPS) {
    if (setpointMeters < 0 || setpointMeters > constElevator.MAX_HEIGHT) {
        m_ledSystem.setLedState("Fault");
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
    m_ledSystem.setLedState("Teleop");  // Show progress during movement
  }

  // Preset position commands

  public Command goToLevel(String lvl) {
    switch (lvl) {
      case "L1":
        return  this.runOnce(() -> setPosition(constElevator.L1));
      case "L2":
        return  this.runOnce(() -> setPosition(constElevator.L1));
      case "L3":
        return  this.runOnce(() -> setPosition(constElevator.L1));
      case "L4":
        return  this.runOnce(() -> setPosition(constElevator.L1));
      default:
        return null;
    }
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
public double getPosition() {
    double motorRotations = rightMotorLeader.getPosition().getValueAsDouble();
    return motorRotations / constElevator.GEAR_RATIO;  
}

  public double getHeight() {
    return getPosition();  
  }

  public double getMaxHeight() {
    return constElevator.MAX_HEIGHT;
  }

  // Stop the controllers
  public void stopControllers() {
    rightMotorLeader.set(0.0);
    leftMotorFollower.set(0.0);
    m_ledSystem.setLedState("Idle");
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

  public Command homeElevator() {
    String previousLedState = m_ledSystem.getLedState();
    m_ledSystem.setLedState("Zeroing");

    return run(() -> {
      // Go down until bottom limit
      rightMotorLeader.setVoltage(constElevator.CALIBRATION_VOLTAGE_DOWN);
      leftMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), true));
    })
    .until(() -> rightMotorLeader.getStatorCurrent().getValueAsDouble() > constElevator.CURRENT_THRESHOLD)
    .beforeStarting(() -> {
      setSlowSpeed();
      m_ledSystem.setLedState("Zeroing");
      System.out.println("Starting elevator homing sequence...");
    })
    .andThen(() -> {
      stopControllers();
      resetSensorPosition(0.0);
      System.out.println("Found bottom, moving up...");
    })
    .andThen(
      run(() -> {
        // Go up until top limit
        rightMotorLeader.setVoltage(constElevator.CALIBRATION_VOLTAGE_UP);
        leftMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), true));
      })
      .until(() -> rightMotorLeader.getStatorCurrent().getValueAsDouble() > constElevator.CURRENT_THRESHOLD)
    )
    .finallyDo((interrupted) -> {
      stopControllers();
      setNormalSpeed();
      if (!interrupted) {
        double foundMaxHeight = getPosition();
        constElevator.MAX_HEIGHT = foundMaxHeight;
        System.out.println("Homing complete. New max height: " + foundMaxHeight);
        m_ledSystem.setLedState(previousLedState);

      } else {
        m_ledSystem.setLedState("Fault");
        System.out.println("Homing sequence interrupted!");

      }
    });
  }

}

