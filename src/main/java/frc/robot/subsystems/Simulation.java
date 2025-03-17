package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.constElevator;

public class Simulation {
    private ElevatorSubsystem m_elevatorSubsystem;
    private AlgaeArmSubsystem m_algaeArmSubsystem;
    private CoralRollerSubsystem m_coralRollerSubsystem;

    private Mechanism2d mech;

    private ElevatorSim m_elevatorSim;
    private MechanismRoot2d m_elevatorRoot;
    private MechanismLigament2d m_elevatorMech2d;

    private SingleJointedArmSim m_algaeSlapdownSim;
    private MechanismRoot2d m_algaeSlapdownRoot;
    private MechanismLigament2d m_algaeSlapdown2d;

    private StructPublisher<Pose3d> elevatorZero;
    private StructPublisher<Pose3d> slapdownPose;

    public Simulation() {
        mech = new Mechanism2d(10, 10);
        elevatorInit();
        algaeSlapdownInit();
    }

    public void setupsubsystems(ElevatorSubsystem a, AlgaeArmSubsystem b,
            CoralRollerSubsystem c) {
        m_elevatorSubsystem = a;
        m_algaeArmSubsystem = b;
        m_coralRollerSubsystem = c;
        c = m_coralRollerSubsystem;
    }

    public void run() {
        elevator();
        algaeSlapdown();

        // SmartDashboard.putData("MainMech", mech);
    }

    private void algaeSlapdownInit() {
        m_algaeSlapdownSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), 10, 0.0268228586, 0.2794, -1.5708, 1.5708,
                true, 1.5708, 0.01, 0.0);
        m_algaeSlapdownRoot = mech.getRoot("Slapdown Root", 3, 0);
        m_algaeSlapdown2d = m_algaeSlapdownRoot
                .append(new MechanismLigament2d("algaeSlapdown", 0.5, 90, 6, new Color8Bit(Color.kPurple)));

        slapdownPose = NetworkTableInstance.getDefault().getStructTopic("/Simulation/slapdownZero", Pose3d.struct)
                .publish();
        slapdownPose.set(new Pose3d());

    }

    private void algaeSlapdown() {
        double targetAngle = m_algaeArmSubsystem.targetAngle;

        m_algaeSlapdownSim.setState(Units.degreesToRadians(targetAngle), 0);
        m_algaeSlapdownSim.update(targetAngle);
        m_algaeSlapdown2d.setAngle(targetAngle);

        slapdownPose.set(new Pose3d(0, 0, 0, new Rotation3d(Units.degreesToRadians(-targetAngle), 0, 0)));
    }

    private void elevatorInit() {
        m_elevatorSim = new ElevatorSim(
                DCMotor.getKrakenX60(2),
                constElevator.GEAR_RATIO,
                Units.lbsToKilograms(20),
                constElevator.SPOOL_CIRCUMFERENCE / 2 / 3.14 * 2,
                constElevator.MIN_HEIGHT,
                constElevator.MAX_HEIGHT,
                true,
                constElevator.MIN_HEIGHT,
                0.001,
                0.0);
        m_elevatorRoot = mech.getRoot("Elevator Root", 5, 0);
        m_elevatorMech2d = m_elevatorRoot
                .append(new MechanismLigament2d("elevator", m_elevatorSim.getPositionMeters(), 90));

        elevatorZero = NetworkTableInstance.getDefault().getStructTopic("/Simulation/elevatorZero", Pose3d.struct)
                .publish();
        elevatorZero.set(new Pose3d());
    }

    private void elevator() {
        double targetPosition = m_elevatorSubsystem.targetPosition;

        m_elevatorSim.setState(targetPosition, 0);

        m_elevatorSim.update(0.020);
        m_elevatorMech2d.setLength(targetPosition);
    }
}
