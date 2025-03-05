package frc.robot.subsystems;

import javax.security.auth.x500.X500Principal;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import ca.team4308.absolutelib.control.JoystickHelper;
import ca.team4308.absolutelib.control.XBoxWrapper;
import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.robot.Constants.constElevator;
import frc.robot.Ports;
import frc.robot.Robot;

public class ElevatorSubsystem extends SubsystemBase {
  private static double kDt = 0.02;

  private static final double POSITION_TOLERANCE = 0.01; // meters
  private double targetPosition = 0.0;
  private Double maxHeight = constElevator.MAX_HEIGHT;
  private Double botHeight = constElevator.MIN_HEIGHT;
  private TalonFX leftMotorFollower;
  private TalonFX rightMotorLeader;
  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;
  private CANcoder cancoder;
  private Double encoderOffset = 0.0;

  private double currentVelocityLimit = constElevator.NORMAL_MOTOR_RPS;

  public ElevatorSubsystem() {
    leftMotorFollower = new TalonFX(Ports.Elevator.ELEVATOR_FOLLOWER);
    rightMotorLeader = new TalonFX(Ports.Elevator.ELEVATOR_MASTER);
    topLimitSwitch = new DigitalInput(Ports.Elevator.LIMIT_SWITCH_TOP);
    bottomLimitSwitch = new DigitalInput(Ports.Elevator.LIMIT_SWITCH_BOTTOM);
    rightMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    leftMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    cancoder = new CANcoder(Ports.Elevator.ELEVATOR_CANCODER);

    cancoder.setPosition(0);
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

    double setpointRotations = targetPosition / (constElevator.SPOOL_CIRCUMFERENCE);
    double motorRotations = setpointRotations * constElevator.GEAR_RATIO;
    double currentMotorRotations = getPosition();

    double pidOutput = constElevator.pidController.calculate(currentMotorRotations, motorRotations);

    double feedforwardVoltage = constElevator.feedforward.calculate(constElevator.pidController.getSetpoint().velocity);

    double totalVoltage = DoubleUtils.clamp(
        pidOutput + feedforwardVoltage,
        -12.0,
        12.0);

    SmartDashboard.putNumber("elevatorFeedforward", feedforwardVoltage);
    SmartDashboard.putNumber("elevatorFeedback", pidOutput);

    if (topLimitSwitch.get()) {
      return Math.max(0, totalVoltage);
    } else if (bottomLimitSwitch.get()) {
      return Math.min(0, totalVoltage);
    }

    return totalVoltage;
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

  public double getCurrentSpeed() {
    return currentVelocityLimit;
  }

  // Elevator data
  /**
   * Gets the elevators position in motorRotations / GEAR_RATIO
   * 
   * @return Double
   */
  double tempSim = 0;

  public double getPosition() {
    if (Robot.isSimulation()) {
      XBoxWrapper afsd = new XBoxWrapper(1);
      tempSim += afsd.getLeftY() / 5;
      return tempSim;
    }

    double motorRotations = rightMotorLeader.getPosition().getValueAsDouble() + encoderOffset;
    return (motorRotations / constElevator.GEAR_RATIO);
    // double encoder = cancoder.getPosition().getValueAsDouble() * 360d;
    // return encoder / constElevator.GEAR_RATIO * 4 ;// needs to be changed
  }

  /**
   * Gets the elevators position in meters
   * 
   * @return Double
   */
  public double getPositionInMeters() {
    return getPosition() * constElevator.SPOOL_CIRCUMFERENCE;
  }

  public double getTarget() {
    return targetPosition;
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
    double voltage = calculateVoltage();
    rightMotorLeader.setVoltage(voltage);
    leftMotorFollower.setVoltage(voltage);

    // Check if the top top limit switch is hit then set that to the new height
    if (topLimitSwitch.get()) {
      encoderOffset = -rightMotorLeader.getPosition().getValueAsDouble() + maxHeight;
    }
    if (bottomLimitSwitch.get()) {
      encoderOffset = -rightMotorLeader.getPosition().getValueAsDouble();
    }
    SmartDashboard.putNumber("Elevator Encoder Offset", encoderOffset);
    SmartDashboard.putNumber("Elevator Position", getPositionInMeters());
    SmartDashboard.putNumber("Elevator Target", targetPosition);
    SmartDashboard.putNumber("Elevator Error", targetPosition - getPositionInMeters());
    SmartDashboard.putBoolean("At Position", isAtPosition());
    SmartDashboard.putNumber("Elevator Current", rightMotorLeader.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Voltage", voltage);
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