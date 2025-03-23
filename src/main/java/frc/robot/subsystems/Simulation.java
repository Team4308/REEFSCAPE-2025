package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.constElevator;

public class Simulation {
    private ElevatorSubsystem m_elevatorSubsystem;
    private AlgaeArmSubsystem m_algaeArmSubsystem;
    private CoralRollerSubsystem m_coralRollerSubsystem;

    private LoggedMechanism2d mech;

    private ElevatorSim m_elevatorSim;
    private LoggedMechanismRoot2d m_elevatorRoot;
    private LoggedMechanismLigament2d m_elevatorMech2d;
    public static boolean elevatorAtPositionSimulation;
    public static double elevatorHeightSimulation;

    private Pose3d zeroPos = new Pose3d();

    public Simulation() {
        mech = new LoggedMechanism2d(10, 10);
        elevatorInit();
    }

    public void setupsubsystems(ElevatorSubsystem elevatorSubsystem, AlgaeArmSubsystem algaeArmSubsystem,
            CoralRollerSubsystem rollerSubsystem) {
        this.m_elevatorSubsystem = elevatorSubsystem;
        this.m_algaeArmSubsystem = algaeArmSubsystem;
        this.m_coralRollerSubsystem = rollerSubsystem;
    }

    public void run() {
        elevator();

        Logger.recordOutput("Mechanism", mech);
        Logger.recordOutput("Zeroed Pose", zeroPos);
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
                .append(new LoggedMechanismLigament2d("elevator", m_elevatorSim.getPositionMeters(), 90));
    }

    private void elevator() {
        double targetPosition = m_elevatorSubsystem.targetPosition;

        // m_elevatorSim.setState(targetPosition, 0);
        m_elevatorSim.setInputVoltage(m_elevatorSubsystem.totalVoltage);

        if (Math.abs(m_elevatorSim.getPositionMeters() - targetPosition) < constElevator.TOLERANCE) {
            elevatorAtPositionSimulation = true;
        } else {
            elevatorAtPositionSimulation = false;
        }
        elevatorHeightSimulation = m_elevatorSim.getPositionMeters();

        m_elevatorSim.update(0.020);
        m_elevatorMech2d.setLength(m_elevatorSim.getPositionMeters());
    }
}
