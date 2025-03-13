package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import ca.team4308.absolutelib.math.DoubleUtils;
import frc.robot.Constants.constSlapdown;
import frc.robot.Ports;

public class SlapdownSubsystem extends SubsystemBase {
    private TalonSRX pivotMotor;
    private TalonFX intakeMotor;
    public double currentTarget = 90;   // straight up (degrees)

    public SlapdownSubsystem() {
        this.pivotMotor = new TalonSRX(Ports.AlgaeSlapdown.ALGAE_PIVOT);
        this.intakeMotor = new TalonFX(Ports.AlgaeSlapdown.ALGAE_ROLLER);
    }

    public double calculatePower() {
        double curPoint = getPosition();

        double pidOut = constSlapdown.PID_CONTROLLER.calculate(curPoint, currentTarget);

        double feedforwardOutput = constSlapdown.FEEDFORWARD.calculate(Math.toRadians(curPoint), 
                                                                    constSlapdown.PID_CONTROLLER.getSetpoint().velocity);

        double output = DoubleUtils.mapRange(DoubleUtils.clamp(pidOut + feedforwardOutput, -12, 12), -12, 12, -1, 1);

        return output;
    }

    public double getPosition() {
        double value = this.pivotMotor.getSelectedSensorPosition() / constSlapdown.ENCODER_TO_DEGREE_RATIO;    // returns degrees
        value = value / constSlapdown.GEAR_RATIO;   // gear ratio
        return value;
    }

    public Command setPosition(double angle) {
        return runOnce(() -> {
            currentTarget = DoubleUtils.clamp(angle, constSlapdown.PIVOT_BOTTOM_ANGLE, constSlapdown.PIVOT_TOP_ANGLE);
        });
    }

    public Command setIntake(double speed) {
        return run(() -> {
            intakeMotor.set(speed);
        });
    }

    // Starts the intake wheels. While it won't be very effective,
    // this will still work even if the arm is up. Try not to do that.
    public void StartIntake() {
        intakeMotor.set(constSlapdown.INTAKE_SPEED);
    }

    // Stops the intake wheels.
    public void StopIntake() {
        intakeMotor.set(0.0);
    }

    @Override
    public void periodic() {
        double power = calculatePower();
        pivotMotor.set(TalonSRXControlMode.PercentOutput, power);
    }
}
