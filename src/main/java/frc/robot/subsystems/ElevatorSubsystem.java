package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.constElevator;
import frc.robot.Ports.Elevator;
import frc.robot.Robot;

public class ElevatorSubsystem extends SubsystemBase {
  public double targetPosition = constElevator.MIN_HEIGHT;
  private TalonFX leftMotorFollower;
  private TalonFX rightMotorLeader;
  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;
  private double encoderOffset;

  public ElevatorSubsystem() {
    leftMotorFollower = new TalonFX(Elevator.ELEVATOR_FOLLOWER);
    rightMotorLeader = new TalonFX(Elevator.ELEVATOR_MASTER);
    topLimitSwitch = new DigitalInput(Elevator.LIMIT_SWITCH_TOP);
    bottomLimitSwitch = new DigitalInput(Elevator.LIMIT_SWITCH_BOTTOM);
    rightMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    leftMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);

    encoderOffset = -rightMotorLeader.getPosition().getValueAsDouble();

    stopControllers();
  }

  /**
   * Sets the elevators position to the desired setpoint
   * 
   * @param double The Desired position in meters
   * @return Null
   */
  public void setPosition(double setpointMeters) {
    targetPosition = DoubleUtils.clamp(setpointMeters, constElevator.MIN_HEIGHT, constElevator.MAX_HEIGHT);
  }

  private double calculateVoltage() {
    double pidOutput = constElevator.PID_CONTROLLER.calculate(getPositionInMeters(), targetPosition);

    double feedforwardVoltage = constElevator.FEEDFORWARD
        .calculate(constElevator.PID_CONTROLLER.getSetpoint().velocity);

    double totalVoltage = DoubleUtils.clamp(pidOutput + feedforwardVoltage, -12.0, 12.0);

    SmartDashboard.putNumber("Setpoint Position", constElevator.PID_CONTROLLER.getSetpoint().position);

    return totalVoltage;
  }

  public boolean isAtPosition() {
    if (Robot.isSimulation()) {
      return true;
    }
    return Math.abs(getPositionInMeters() - targetPosition) < constElevator.TOLERANCE;
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
      switch (lvl) {
        default:
        case 0:
          setPosition(constElevator.MIN_HEIGHT);
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
          setPosition(constElevator.MAX_HEIGHT);
          break;
      }
    });
  }

  // Speed Control
  public void setConstraints(double velocity, double acceleration) {
    constElevator.PID_CONTROLLER.setConstraints(new TrapezoidProfile.Constraints(velocity, acceleration));
  }

  // Elevator data
  /**
   * Gets the elevators position in motorRotations / GEAR_RATIO
   * 
   * @return Double
   */
  public double getPosition() {
    double motorRotations = rightMotorLeader.getPosition().getValueAsDouble() + encoderOffset;
    return (motorRotations / constElevator.GEAR_RATIO);
  }

  /**
   * Gets the elevators position in meters
   * 
   * @return Double
   */
  public double getPositionInMeters() {
    return (getPosition() * constElevator.SPOOL_CIRCUMFERENCE) + constElevator.MIN_HEIGHT;
  }

  /**
   * Stops the controllers
   * 
   * @return Null
   */
  public void stopControllers() {
    rightMotorLeader.set(0.0);
    leftMotorFollower.set(0.0);
  }

  @Override
  public void periodic() {
    double voltage = calculateVoltage();

    if (targetPosition == constElevator.MIN_HEIGHT && getPositionInMeters() < 0.15) {
      voltage = 0.0;
    }

    rightMotorLeader.setVoltage(voltage);
    leftMotorFollower.setVoltage(voltage);

    // Check if the top top limit switch is hit then set that to the new height
    if (topLimitSwitch.get()) {
      encoderOffset = -rightMotorLeader.getPosition().getValueAsDouble()
          + ((constElevator.MAX_HEIGHT - constElevator.MIN_HEIGHT) / constElevator.SPOOL_CIRCUMFERENCE)
              * constElevator.GEAR_RATIO;
    }
    if (bottomLimitSwitch.get()) {
      encoderOffset = -rightMotorLeader.getPosition().getValueAsDouble();
    }
    // SmartDashboard.putBoolean("Elevator At Position", isAtPosition());
    SmartDashboard.putNumber("Elevator Position", getPositionInMeters());
  }
}