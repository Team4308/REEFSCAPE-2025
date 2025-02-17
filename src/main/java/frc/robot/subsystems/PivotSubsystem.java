package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.wrapper.LogSubsystem;
import ca.team4308.absolutelib.wrapper.LoggedTunableNumber;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
public class PivotSubsystem extends LogSubsystem {

    public final TalonFX motor;
    public final PIDController pidController;
    private final CANcoder canCoder; 
    
    private final PIDController pid = new PIDController(Constants.ArmConstants.armPID[0],
        Constants.ArmConstants.armPID[1],
        Constants.ArmConstants.armPID[2]);
    private ArmFeedforward ffModel = new ArmFeedforward(
        Constants.ArmConstants.armSGV[0],
        Constants.ArmConstants.armSGV[1],
        Constants.ArmConstants.armSGV[2]);

        public Rotation2d encoderPosition = new Rotation2d();
    

    private LoggedTunableNumber armP = new LoggedTunableNumber("armP", Constants.ArmConstants.armPID[0]);
    private LoggedTunableNumber armI = new LoggedTunableNumber("armI", Constants.ArmConstants.armPID[1]);
    private LoggedTunableNumber armD = new LoggedTunableNumber("armD", Constants.ArmConstants.armPID[2]);
    private LoggedTunableNumber armS = new LoggedTunableNumber("armS", Constants.ArmConstants.armSGV[0]);
    private LoggedTunableNumber armG = new LoggedTunableNumber("armG", Constants.ArmConstants.armSGV[1]);
    private LoggedTunableNumber armV = new LoggedTunableNumber("armV", Constants.ArmConstants.armSGV[2]);

    private Rotation2d setpoint = new Rotation2d();
    private Rotation2d velocity = new Rotation2d();
    private Rotation2d goal = new Rotation2d();

    public PivotSubsystem() {
        motor = new TalonFX(Constants.Mapping.Shooter.motor);
        
        canCoder = new CANcoder(Constants.Mapping.Shooter.encoder);

        motor.setNeutralMode(NeutralModeValue.Brake);

        pidController = new PIDController(Constants.Shooter.AngleControl.kP, Constants.Shooter.AngleControl.kI, Constants.Shooter.AngleControl.kD);//pid not tuned

        stopControllers();
    }

    public void setMotorOutput(double percent){
        motor.set(percent);
    }

    
    public Rotation2d getEncoderPosition() {
        return encoderPosition;
    }


    public void setMotorPosition(double degree) { 
        SmartDashboard.putNumber("encoderDegree", canCoder.getPosition().getValueAsDouble());
        
        double wantedDegree = DoubleUtils.clamp(degree, Constants.Shooter.shooterStartDegree, Constants.Shooter.shooterEndDegree);

        double shooterDegree = DoubleUtils.mapRangeNew(canCoder.getPosition().getValueAsDouble(), Constants.Shooter.encoderStartRevolutions, Constants.Shooter.encoderEndRevolutions, Constants.Shooter.shooterStartDegree, Constants.Shooter.shooterEndDegree);

        double motorOutput = -DoubleUtils.clamp(pidController.calculate(shooterDegree, wantedDegree), -1.0, 1.0);

        SmartDashboard.putNumber("shooterDegree", shooterDegree);
        SmartDashboard.putNumber("wantedDegree", wantedDegree);
        SmartDashboard.putNumber("motorOutput", motorOutput);

        setMotorOutput(motorOutput);
    }

      public void checkTunableValues() {
        if (!Constants.enableTunableValues)
        return;
        // IDK about the ID's, if you know better, change them
    if (armP.hasChanged(0) || armI.hasChanged(0) || armD.hasChanged(0)) {
      pid.setPID(armP.get(), armI.get(), armD.get());
    }
    if (armS.hasChanged(0) || armG.hasChanged(0) || armV.hasChanged(0)) {
      ffModel = new ArmFeedforward(armS.get(), armG.get(), armV.get());
    }
  }

        public void stopControllers() {
        motor.set(0.0);
    }

    public Sendable log() {
        return this;
    }

    @Override
    public void periodic() {
        checkTunableValues();
  
        var ffOutput = ffModel.calculate(setpoint.getRadians(), velocity.getRadians());
        var pidOutput = pid.calculate(getEncoderPosition().getRadians(), setpoint.getRadians());
  
        SmartDashboard.putNumber("ffoutput arm", ffOutput);
    }
}

