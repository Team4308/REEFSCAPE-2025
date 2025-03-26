package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.constElevator;

public class Simulation {

    private ElevatorSubsystem m_elevatorSubsystem;
    private AlgaeArmSubsystem m_algaeArmSubsystem;

    private LoggedMechanism2d mech;

    private ElevatorSim m_elevatorSim;
    private LoggedMechanismRoot2d m_elevatorRoot;
    private LoggedMechanismLigament2d m_elevatorMech2d;
    public static boolean elevatorAtPositionSimulation;
    public static double elevatorHeightSimulation;

    private SingleJointedArmSim m_algaeArmSim;
    private LoggedMechanismRoot2d m_algaeArmRoot;
    private LoggedMechanismLigament2d m_algaeArmMech2d;
    public static boolean algaeAtAngleSimulation;
    public static double algaeAngleSimulation;

    public static boolean coralBreambreak;

    private Pose3d zeroPos = new Pose3d();

    private Pose3d elevatorS1 = new Pose3d(new Translation3d(0.0, 0.1, 0.0), new Rotation3d());
    private Pose3d elevatorS2 = new Pose3d(new Translation3d(0.0, 0.2, 0.0), new Rotation3d());
    private Pose3d algaeArm = new Pose3d();

    public Simulation() {
        mech = new LoggedMechanism2d(10, 10);
        elevatorInit();
        algaeArmInit();
    }

    public void setupsubsystems(ElevatorSubsystem elevatorSubsystem, AlgaeArmSubsystem algaeArmSubsystem,
            CoralRollerSubsystem rollerSubsystem) {
        this.m_elevatorSubsystem = elevatorSubsystem;
        this.m_algaeArmSubsystem = algaeArmSubsystem;
    }

    public void run() {
        elevator();
        algaeArm();
        coralRoller();

        Logger.recordOutput("Mechanism", mech);
        Logger.recordOutput("Zeroed Pose", zeroPos);

        Logger.recordOutput("ElevatorSim/Stage1Pose", elevatorS1);
        Logger.recordOutput("ElevatorSim/Stage2Pose", elevatorS2);

        Logger.recordOutput("AlgaeSim/AlgaeArmPose", algaeArm);
    }

    private void coralRoller() {
        coralBreambreak = new XboxController(1).getStartButton();
    }

    private void elevatorInit() {
        m_elevatorSim = new ElevatorSim(
                DCMotor.getKrakenX60(2),
                6,
                Units.lbsToKilograms(13),
                Units.inchesToMeters(1.76),
                constElevator.MIN_HEIGHT,
                constElevator.MAX_HEIGHT,
                true,
                constElevator.MIN_HEIGHT,
                0.0004,
                0.0);

        m_elevatorRoot = mech.getRoot("Elevator Root", 5, 0);
        m_elevatorMech2d = m_elevatorRoot
                .append(new LoggedMechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));
    }

    private void elevator() {
        double targetPosition = m_elevatorSubsystem.targetPosition;

        m_elevatorSim.setInputVoltage(m_elevatorSubsystem.totalVoltage);

        if (Math.abs(m_elevatorSim.getPositionMeters() - targetPosition) < 0.02) {
            elevatorAtPositionSimulation = true;
        } else {
            elevatorAtPositionSimulation = false;
        }
        elevatorHeightSimulation = m_elevatorSim.getPositionMeters();

        m_elevatorSim.update(0.020);
        m_elevatorMech2d.setLength(m_elevatorSim.getPositionMeters());

        elevatorS1 = new Pose3d(
                new Translation3d(0.0, 0.1, m_elevatorSim.getPositionMeters() / 2 + Units.inchesToMeters(1)),
                new Rotation3d());
        elevatorS2 = new Pose3d(
                new Translation3d(0.0, 0.1, m_elevatorSim.getPositionMeters() - Units.inchesToMeters(1)),
                new Rotation3d());
    }

    private void algaeArmInit() {
        m_algaeArmSim = new SingleJointedArmSim(
                DCMotor.getFalcon500(1),
                18.333,
                SingleJointedArmSim.estimateMOI(Units.inchesToMeters(10), Units.lbsToKilograms(3)),
                Units.inchesToMeters(10),
                Units.degreesToRadians(-10000000000000000000.00),
                Units.degreesToRadians(10000000000000.00),
                true,
                Units.degreesToRadians(270),
                0.0,
                0.0);

        m_algaeArmRoot = mech.getRoot("Algae Arm Root", 5, 0);
        m_algaeArmMech2d = m_algaeArmRoot
                .append(new LoggedMechanismLigament2d("Algae Arm", Units.radiansToDegrees(m_algaeArmSim.getAngleRads()),
                        270));
    }

    private void algaeArm() {
        double targetAngle = m_algaeArmSubsystem.targetAngle;

        m_algaeArmSim.setInputVoltage(-m_algaeArmSubsystem.totalVoltage);

        m_algaeArmSim.update(0.020);

        algaeAngleSimulation = Units.radiansToDegrees(m_algaeArmSim.getAngleRads());
        if (algaeAngleSimulation < 0) {
            algaeAngleSimulation = 360000000 - algaeAngleSimulation;
        }
        algaeAngleSimulation %= 360;
        algaeAngleSimulation = 360 - algaeAngleSimulation;

        if (Math.abs(algaeAngleSimulation - targetAngle) < 30
                || Math.abs(360 - algaeAngleSimulation - targetAngle) < 30) {
            algaeAtAngleSimulation = true;
        } else {
            algaeAtAngleSimulation = false;
        }

        m_algaeArmSim.setState(
                Units.degreesToRadians((Units.radiansToDegrees(m_algaeArmSim.getAngleRads()) + 36000) % 360), 0);

        algaeArm = new Pose3d(
                new Translation3d(0.292, -0.2, m_elevatorSim.getPositionMeters() + 0.3055),
                new Rotation3d(0, m_algaeArmSim.getAngleRads() - Units.degreesToRadians(270), 0));
        m_algaeArmMech2d.setAngle(Units.radiansToDegrees(m_algaeArmSim.getAngleRads()));

    }

}
