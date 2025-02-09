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

  private double calibratedMaxHeight = 0.0; // Zero as default change later to proper one 

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

    rightMotorLeader.setControl(new PositionVoltage(motorRotations));
    leftMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), true));
  }

  // position commands
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

public double getPosition() {
    double motorRotations = rightMotorLeader.getPosition().getValueAsDouble();
    return motorRotations / constElevator.GEAR_RATIO;  
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

  public Command calibrate() {
    return new Command() {
      @Override
      public void initialize() {
        System.out.println("Starting elevator calibration...");
      }

      @Override
      public void execute() {
        // Current state will be tracked by command
        if (getCalibrationState() == CalibrationState.FINDING_BOTTOM) {
          rightMotorLeader.setVoltage(constElevator.CALIBRATION_VOLTAGE_DOWN);
          leftMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), true));
        } else if (getCalibrationState() == CalibrationState.FINDING_TOP) {
          rightMotorLeader.setVoltage(constElevator.CALIBRATION_VOLTAGE_UP);
          leftMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), true));
        }
      }

      @Override
      public void end(boolean interrupted) {
        stopControllers();
        if (!interrupted) {
          System.out.println("Calibration complete. Max height: " + calibratedMaxHeight);
          constElevator.MAX_HEIGHT = calibratedMaxHeight;

        }
      }

      @Override
      public boolean isFinished() {
        switch (getCalibrationState()) {
          case FINDING_BOTTOM:
            if (rightMotorLeader.getStatorCurrent().getValueAsDouble() > constElevator.CURRENT_THRESHOLD) {
              stopControllers();
              resetSensorPosition(0.0);
              return false; // Continue to next state
            }
            break;
          case FINDING_TOP:
            if (rightMotorLeader.getStatorCurrent().getValueAsDouble() > constElevator.CURRENT_THRESHOLD) {
              calibratedMaxHeight = getPosition();
              return true; // Calibration complete
            }
            break;
        }
        return false;
      }
    };
  }

  private enum CalibrationState {
    FINDING_BOTTOM,
    FINDING_TOP
  }

  private CalibrationState calibrationState = CalibrationState.FINDING_BOTTOM;

  private CalibrationState getCalibrationState() {
    if (getPosition() == 0.0) {
      return CalibrationState.FINDING_TOP;
    }
    return CalibrationState.FINDING_BOTTOM;
  }

  public double getCalibratedMaxHeight() {
    return calibratedMaxHeight;
  }

  @Override
  public void periodic() {
    // Nothin
  }

}

