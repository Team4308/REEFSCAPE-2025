package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constElevator;
import frc.robot.Ports;

public class ElevatorSubsystem extends SubsystemBase {

  private static final double POSITION_TOLERANCE = 0.01; // meters
  private double targetPosition = 0.0;
  private Double foundMaxHeight = null;
  private Double foundBottemHeight = null;
  private TalonFX leftMotorFollower;
  private TalonFX rightMotorLeader;
  private DigitalInput topLimitSwitch;
  private DigitalInput bottemLimitSwitch;
  private CANcoder cancoder;

  private double currentVelocityLimit = constElevator.NORMAL_MOTOR_RPS;

  public ElevatorSubsystem() {
    leftMotorFollower = new TalonFX(Ports.Elevator.ELEVATOR_FOLLOWER);
    rightMotorLeader = new TalonFX(Ports.Elevator.ELEVATOR_MASTER);
    topLimitSwitch = new DigitalInput(Ports.Elevator.LIMIT_SWITCH_TOP);
    bottemLimitSwitch = new DigitalInput(Ports.Elevator.LIMIT_SWITCH_BOTTOM);
    rightMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    leftMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    cancoder = new CANcoder(Ports.Elevator.ELEVATOR_CANCODER);

    cancoder.setPosition(0);
  }

  /**
   * Sets the elevators position to the desired setpoint
   * 
   * @param double   The Desired position in meters
   * @param velocity The desired target velosity(default is the
   *                 currentVelocityLimit)
   * @return Null
   */
  public Command setPositionCommand(double setpointMeters, double... velocitys) {
    return run(() -> {
      assert velocitys.length <= 1;
      double velocity = currentVelocityLimit;
      velocity = velocitys.length > 0 ? velocitys[0] : 0.0;

      targetPosition = DoubleUtils.clamp(setpointMeters,
          (!Double.isNaN(foundBottemHeight)) ? foundBottemHeight
              : constElevator.MIN_HEIGHT /* One Liner thats scuffed !? */,
          (!Double.isNaN(foundMaxHeight)) ? foundMaxHeight : constElevator.MAX_HEIGHT);

      double setpointRotations = targetPosition / (Math.PI * constElevator.SPOOL_RADIUS);
      double motorRotations = setpointRotations * constElevator.GEAR_RATIO;
      double currentMotorRotations = getPosition();

      double pidOutput = constElevator.pidController.calculate(currentMotorRotations, motorRotations);

      double requestedVelocity = DoubleUtils.clamp(
          velocity,
          -constElevator.MAX_MOTOR_RPS,
          constElevator.MAX_MOTOR_RPS);

      double feedforwardVoltage = constElevator.feedforward.calculate(requestedVelocity);

      double totalVoltage = DoubleUtils.clamp(
          pidOutput + feedforwardVoltage,
          -6.0,
          6.0);

      rightMotorLeader.setVoltage(totalVoltage);
      leftMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), false));
    })
        .until(() -> isAtPosition())
        .withName("SetPosition " + setpointMeters);
  }

  /**
   * Checks if the elevator is at the desired position
   * 
   * @return Boolean
   */
  public boolean isAtPosition() {
    return Math.abs(getPositionInMeters() - targetPosition) < POSITION_TOLERANCE;
  }

  // Preset position commands
  /**
   * Sets the elevators position to the desired setpoint
   * 
   * @param int Level
   * @return Null
   */
  public Command goToLevel(int lvl) {
    System.out.println("Setting level to: " + lvl);
    switch (lvl) {
      case 0:
        return setPositionCommand(0.0);
      case 1:
        return setPositionCommand(constElevator.L1);
      case 2:
        return setPositionCommand(constElevator.L2);
      case 3:
        return setPositionCommand(constElevator.L3);
      case 4:
        return setPositionCommand(constElevator.L4);
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

  // Elevator data
  /**
   * Gets the elevators position in motorRotations / GEAR_RATIO
   * 
   * @return Double
   */
  public double getPosition() {
    // double motorRotations = rightMotorLeader.getPosition().getValueAsDouble();
    // return motorRotations / constElevator.GEAR_RATIO;
    double encoder = cancoder.getPosition().getValueAsDouble() * 360d;
    return encoder / constElevator.GEAR_RATIO * 4;// needs to be changed
  }

  /**
   * Gets the elevators position in meters
   * 
   * @return Double
   */
  public double getPositionInMeters() {
    return getPosition() * (Math.PI * constElevator.SPOOL_RADIUS);
  }

  /**
   * Gets the elevators max meight
   * 
   * @return Double
   */
  public double getMaxHeight() {
    return constElevator.MAX_HEIGHT;
  }

  // Stop the controllers
  /**
   * Stops the controllers
   * 
   * @return Null
   */
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

    // Check if the top top limit switch is hit then set that to the new height
    if (topLimitSwitch.get()) {
      foundMaxHeight = getPositionInMeters();

    }
    if (bottemLimitSwitch.get()) {
      foundBottemHeight = getPositionInMeters();
    }

    SmartDashboard.putNumber("Elevator Position", getPositionInMeters());
    SmartDashboard.putNumber("Elevator Target", targetPosition);
    SmartDashboard.putNumber("Elevator Error", targetPosition - getPositionInMeters());
    SmartDashboard.putBoolean("At Position", isAtPosition());
    SmartDashboard.putNumber("Elevator Current", rightMotorLeader.getStatorCurrent().getValueAsDouble());
  }

  /**
   * Homes the elevator
   * 
   * @return Null
   */
  public Command homeElevator() {

    return run(() -> {
      // Go down until bottom limit
      rightMotorLeader.setVoltage(constElevator.CALIBRATION_VOLTAGE_DOWN);
      leftMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), true));
    })
        .until(() -> rightMotorLeader.getStatorCurrent().getValueAsDouble() > constElevator.CURRENT_THRESHOLD)
        .beforeStarting(() -> {
          setSlowSpeed();
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
                .until(() -> rightMotorLeader.getStatorCurrent().getValueAsDouble() > constElevator.CURRENT_THRESHOLD))
        .finallyDo((interrupted) -> {
          stopControllers();
          setNormalSpeed();
          if (!interrupted) {
            double foundMaxHeight = getPositionInMeters();
            /// MAX_HEIGHT = foundMaxHeight;
            System.out.println("Homing done. New max height: " + foundMaxHeight);

          } else {
            System.out.println("Homing sequence failed!");

          }
        });
  }

}
