package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ports;
import frc.robot.Robot;

public class CoralRollerSubsystem extends LogSubsystem {

    private TalonFX rollerMotor = new TalonFX(Ports.EndEffector.CORAL_MOTOR);

    private VelocityVoltage rollerVelocity = new VelocityVoltage(0.0);

    public DigitalInput beamBreak = new DigitalInput(Ports.EndEffector.INDEX_BEAM_BREAK);

    public double targetVelocity = 0;

    public CoralRollerSubsystem() {
        var config = new Slot0Configs();
        config.kS = 0.1; // Add 0.1 V output to overcome static friction
        config.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        config.kP = 0.11; // An error of 1 rps results in 0.11 V output
        config.kI = 0; // no output for integrated error
        config.kD = 0;
        rollerMotor.getConfigurator().apply(config);
        rollerVelocity.Slot = 0;

        rollerMotor.setNeutralMode(NeutralModeValue.Brake);

        stopControllers();
    }

    public void setRollerOutput(double velocity) {
        targetVelocity = velocity;
        rollerMotor.setControl(rollerVelocity.withVelocity(velocity));
    }

    public void stopControllers() {
        rollerMotor.set(0.0);
    }

    public Sendable log() {
        return this;
    }

    public boolean getBeamBreak() {
        if (Robot.isSimulation()) {
            return new XboxController(0).getAButton();
        }
        return !beamBreak.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intaken?", !beamBreak.get());
    }

}