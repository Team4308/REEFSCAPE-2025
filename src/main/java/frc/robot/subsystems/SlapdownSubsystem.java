package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;

public class SlapdownSubsystem extends SubsystemBase {

    private final double PIVOT_TOP = 0.0; // The angle (in rotations) that the pivot motor should be at when the arm is at the top.
    private final double PIVOT_BOTTOM = 0.2; // The angle (in rotations) that the pivot motor should be at when the arm is on the ground.
    private final double INTAKE_SPEED = 0.8; // The speed (in rotations/sec) that the intake motor spins at. Higher values spin faster.
    
    /*
     * Subsystem roughly works like this (not to scale):
     *        _
     *       | O
     *      [2]
     *     / |_O
     *    /
     *  [1]
     * 
     * Motor 1 is the pivot motor; it causes the whole mechanism to move up and down. When down, it can grab
     * pipes off of the ground.
     * 
     * Motor 2 is the intake motor; it does the actual intaking, pulling pipes in. Its only purpose is to
     * spin the wheels, and they are physically linked together.
     */

    private TalonFX pivotMotor;
    private TalonFX intakeMotor;

    public SlapdownSubsystem() {
        this.pivotMotor = new TalonFX(1); // To do: Get the right device IDs.
        this.intakeMotor = new TalonFX(2);
    }

    public void SetArmPosition(double position) {
        // 0.0 is all the way up, 1.0 is all the way down.
        pivotMotor.setControl(new PositionVoltage(PIVOT_TOP + position * (PIVOT_TOP - PIVOT_BOTTOM)));
    }

    // Starts the intake wheels. While it won't be very effective,
    // this will still work even if the arm is up. Try not to do that.
    public void StartIntake() {
        intakeMotor.set(INTAKE_SPEED);
    }

    // Stops the intake wheels.
    public void StopIntake() {
        intakeMotor.set(0.0);
    }
}
