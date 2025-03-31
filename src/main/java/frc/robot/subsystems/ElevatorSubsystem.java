package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.constElevator;
import frc.robot.Ports.Elevator;
import frc.robot.Robot;
import static edu.wpi.first.units.Units.Volts;

public class ElevatorSubsystem extends SubsystemBase {
  public double targetPosition = constElevator.MIN_HEIGHT;
  private TalonFX leftMotorFollower;
  private TalonFX rightMotorLeader;
  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;
  private double encoderOffset;
  public double totalVoltage;

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
          null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
          (state) -> SignalLogger.writeString("state", state.toString())),
      new SysIdRoutine.Mechanism(
          (volts) -> rightMotorLeader.set(totalVoltage),
          null,
          this));

  public ElevatorSubsystem() {
    leftMotorFollower = new TalonFX(Elevator.ELEVATOR_FOLLOWER);
    rightMotorLeader = new TalonFX(Elevator.ELEVATOR_MASTER);
    topLimitSwitch = new DigitalInput(Elevator.LIMIT_SWITCH_TOP);
    bottomLimitSwitch = new DigitalInput(Elevator.LIMIT_SWITCH_BOTTOM);
    rightMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    leftMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);

    encoderOffset = -rightMotorLeader.getPosition().getValueAsDouble();

    stopControllers();
    resetSensors();
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

    totalVoltage = DoubleUtils.clamp(pidOutput + feedforwardVoltage, -12.0, 12.0);

    SmartDashboard.putNumber("Setpoint Position",
        constElevator.PID_CONTROLLER.getSetpoint().position);

    return totalVoltage;
  }

  public boolean isAtPosition() {
    if (Robot.isSimulation()) {
      return Simulation.elevatorAtPositionSimulation;
    }
    return Math.abs(getPositionInMeters() - targetPosition) < constElevator.TOLERANCE;
  }

  public boolean isAtPosition2(String type) {
    double test = 0.0;
    switch (type) {
      case "MIN":
        test = constElevator.MIN_HEIGHT;
        break;
      case "L1":
        test = constElevator.L1;
        break;
      case "L2":
        test = constElevator.L2;
        break;
      case "L3":
        test = constElevator.L3;
        break;
      case "A1":
        test = constElevator.ALGAE1;
        break;
      case "A2":
        test = constElevator.ALGAE2;
        break;
      case "A1P":
        test = constElevator.ALGAE1_PREMOVE;
        break;
      case "A2P":
        test = constElevator.ALGAE2_PREMOVE;
        break;
    }
    return Math.abs(getPositionInMeters() - test) < constElevator.TOLERANCE;
  }

  public boolean isAtPosition3(double posotion) {
    return Math.abs(getPositionInMeters() - posotion) < constElevator.TOLERANCE;
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
    if (Robot.isSimulation()) {
      return Simulation.elevatorHeightSimulation;
    }
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

  public void resetSensors() {
    stopControllers();
    encoderOffset = -rightMotorLeader.getPosition().getValueAsDouble();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
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
    // SmartDashboard.putNumber("Elevator Position", getPositionInMeters());
    // SmartDashboard.putNumber("Elevator Target Position", targetPosition);

    Logger.recordOutput("Subsystems/Elevator/Target Position", targetPosition);
    Logger.recordOutput("Subsystems/Elevator/Current Position", getPositionInMeters());
    Logger.recordOutput("Subsystems/Elevator/Is At Position", isAtPosition());
    Logger.recordOutput("Subsystems/Elevator/Elevator Voltage", voltage);

    Logger.recordOutput("Subsystems/Elevator/LoggedS1", new Pose3d(
        new Translation3d(0.0, 0.1, getPositionInMeters() / 2 + Units.inchesToMeters(1)),
        new Rotation3d()));
    Logger.recordOutput("Subsystems/Elevator/LoggedS2", new Pose3d(
        new Translation3d(0.0, 0.1, getPositionInMeters() - Units.inchesToMeters(1)),
        new Rotation3d()));
  }
}