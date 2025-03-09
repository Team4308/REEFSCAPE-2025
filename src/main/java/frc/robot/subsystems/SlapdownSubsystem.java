package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import ca.team4308.absolutelib.math.DoubleUtils;
import frc.robot.Constants.Slapdown;
import frc.robot.Ports;

public class SlapdownSubsystem extends SubsystemBase {
    private TalonSRX pivotMotor;
    private TalonFX intakeMotor;
    private ArmFeedforward slapdownArmFeedforward = new ArmFeedforward(0, 0, 0);
    private PIDController pidController = new PIDController(0, 0, 0);
    public double currentTarget = 90;// straight up

    public SlapdownSubsystem() {
        this.pivotMotor = new TalonSRX(Ports.AlgaeSlapdown.ALGAE_PIVOT);
        this.intakeMotor = new TalonFX(Ports.AlgaeSlapdown.ALGAE_ROLLER);
    }

    public void SetArmPosition(double degrees) {
        currentTarget = DoubleUtils.clamp(degrees, 0, 90);// 90 is straight down, probably too far
    }

    public double calculatePower() {
        double setPoint = currentTarget;
        double curPoint = getPosition();

        double pidOut = pidController.calculate(curPoint, setPoint);

        double feedforwardOutput = slapdownArmFeedforward.calculate(Math.toRadians(curPoint),
                69.0);  // change
        feedforwardOutput = DoubleUtils.mapRange(feedforwardOutput, -12, 12, -1, 1);

        double output = pidOut + feedforwardOutput;
        output = DoubleUtils.clamp(feedforwardOutput, -1, 1);
        return output;
    }

    public double getPosition() {
        double value = this.pivotMotor.getSelectedSensorPosition() / 10;// returns degrees
        value = value / 2;// gear ratio
        return value;
    }

    public Command setPosition(double angle) {
        return runOnce(() -> {
            currentTarget = angle;
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
        intakeMotor.set(Slapdown.INTAKE_SPEED);
    }

    // Stops the intake wheels.
    public void StopIntake() {
        intakeMotor.set(0.0);
    }

    @Override
    public void periodic() {
        double power = calculatePower();
        // pivotMotor.set(TalonSRXControlMode.PercentOutput, power);
    }
}
