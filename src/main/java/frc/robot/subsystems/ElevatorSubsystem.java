package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constElevator;

public class ElevatorSubsystem extends SubsystemBase {

  private static final double POSITION_TOLERANCE = 0.01; // meters
  private double targetPosition = 0.0;
  private Double maxHeight = constElevator.MAX_HEIGHT;
  private Double botHeight = constElevator.MIN_HEIGHT;
  private TalonFX leftMotorFollower;
  private TalonFX rightMotorLeader;
  private DigitalInput topLimitSwitch;
  private DigitalInput bottemLimitSwitch;

  private double currentVelocityLimit = constElevator.NORMAL_MOTOR_RPS;

  public ElevatorSubsystem() {
    leftMotorFollower = new TalonFX(constElevator.ELEVATOR_LEADER);
    rightMotorLeader = new TalonFX(constElevator.ELEVATOR_FOLLOWER);
    topLimitSwitch = new DigitalInput(constElevator.top_Limit);
    bottemLimitSwitch = new DigitalInput(constElevator.bottem_Limit);
    rightMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    leftMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
  }

  /**
   * Sets the elevators position to the desired setpoint
   * 
   * @param double The Desired position in meters
   * @return Null
   */
  public void setPosition(double setpointMeters) {
    targetPosition = setpointMeters;
  }

  private double calculateVoltage() {
    targetPosition = DoubleUtils.clamp(targetPosition,
        botHeight, maxHeight);

    double setpointRotations = targetPosition / (Math.PI *
        constElevator.SPOOL_RADIUS);
    double motorRotations = setpointRotations * constElevator.GEAR_RATIO;
    double currentMotorRotations = getPosition();

    double pidOutput = constElevator.pidController.calculate(currentMotorRotations, motorRotations);
    System.out.print("pid output: ");
    System.out.println(pidOutput);
    double requestedVelocity = DoubleUtils.clamp(
        currentVelocityLimit,
        -constElevator.MAX_MOTOR_RPS,
        constElevator.MAX_MOTOR_RPS);

    double feedforwardVoltage = constElevator.feedforward.calculate(requestedVelocity);
    System.out.print("feedforward output: ");
    System.out.println(feedforwardVoltage);
    double totalVoltage = DoubleUtils.clamp(
        pidOutput + feedforwardVoltage,
        -6.0,
        6.0);

    return totalVoltage;
  }

  /**
   * Checks if the elevator is at the desired position
   * 
   * @return Boolean
   */
  private boolean isAtPosition() {
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
    return runOnce(() -> {
      System.out.println("Setting level to: " + lvl);
      switch (lvl) {
        case 0:
          setPosition(0.0);
          break;
        case 1:
          setPosition(constElevator.L1);
          break;
        case 2:
          setPosition(constElevator.L2);
          break;
        case 3:
          setPosition(constElevator.L3);
          break;
        case 4:
          setPosition(constElevator.L4);
          break;
        default:
          setPosition(0.0);
          break;
      }
    });
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
    double motorRotations = rightMotorLeader.getPosition().getValueAsDouble();
    return motorRotations / constElevator.GEAR_RATIO;
  }

  /**
   * Gets the elevators position in meters
   * 
   * @return Double
   */
  public double getPositionInMeters() {

    return getPosition() * constElevator.SPOOL_CIRCUMFERENCE;
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
    if (!isAtPosition()) {
      double voltage = calculateVoltage();
      rightMotorLeader.setVoltage(voltage);
      leftMotorFollower.setVoltage(voltage);
    }

    // Check if the top top limit switch is hit then set that to the new height
    if (topLimitSwitch.get()) {
      // System.out.println("ADSFDGHJHFAGSHNDFH");
      // foundMaxHeight = getPositionInMeters();
    }
    if (bottemLimitSwitch.get()) {
      // foundBottemHeight = getPositionInMeters();
      // System.out.println("ADSFDGHJHFAGSHNDFH");
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