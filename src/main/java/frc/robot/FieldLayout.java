package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class FieldLayout {

    /**
     * Origin is the bottom left corner of the field image (Close right corner from
     * blue driver station POV)
     */

    //Everything in Meters
    public static final double kFieldLength = 17.5482504;
    public static final double kFieldWidth = 8.0519016;
    public static final double kTapeWidth = Units.inchesToMeters(2.0);

    // AprilTag locations
    public static final Map<Integer, Pose3d> aprilTags = Map.ofEntries(
        Map.entry(1, // Put location here
            new Pose3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(4),
                    Units.inchesToMeters(0),
                    new Rotation3d(0.0, 0.0, 0.0))),
        Map.entry(2, // Put location here
            new Pose3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(4),
                    Units.inchesToMeters(0),
                    new Rotation3d(0.0, 0.0, 0.0))),
        Map.entry(3, // Put location here
            new Pose3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(4),
                    Units.inchesToMeters(0),
                    new Rotation3d(0.0, 0.0, 0.0))),
        Map.entry(4, // Put location here
            new Pose3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(4),
                    Units.inchesToMeters(0),
                    new Rotation3d(0.0, 0.0, 0.0))),
        Map.entry(5, // Put location here
            new Pose3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(4),
                    Units.inchesToMeters(0),
                    new Rotation3d(0.0, 0.0, 0.0))),
        Map.entry(6, // Put location here
            new Pose3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(4),
                    Units.inchesToMeters(0),
                    new Rotation3d(0.0, 0.0, 0.0))),
        Map.entry(7, // Put location here
            new Pose3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(4),
                    Units.inchesToMeters(0),
                    new Rotation3d(0.0, 0.0, 0.0))),
        Map.entry(8, // Put location here
            new Pose3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(4),
                    Units.inchesToMeters(0),
                    new Rotation3d(0.0, 0.0, 0.0))),
        Map.entry(9, // Put location here
            new Pose3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(4),
                    Units.inchesToMeters(0),
                    new Rotation3d(0.0, 0.0, 0.0))),
        Map.entry(10, // Put location here
            new Pose3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(4),
                    Units.inchesToMeters(0),
                    new Rotation3d(0.0, 0.0, 0.0))),
        Map.entry(11, // Put location here
            new Pose3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(4),
                    Units.inchesToMeters(0),
                    new Rotation3d(0.0, 0.0, 0.0))),
        Map.entry(12, // Put location here
            new Pose3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(4),
                    Units.inchesToMeters(0),
                    new Rotation3d(0.0, 0.0, 0.0))),
        Map.entry(13, // Put location here
            new Pose3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(4),
                    Units.inchesToMeters(0),
                    new Rotation3d(0.0, 0.0, 0.0))),
        Map.entry(14, // Put location here
            new Pose3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(4),
                    Units.inchesToMeters(0),
                    new Rotation3d(0.0, 0.0, 0.0))),
        Map.entry(15, // Put location here
            new Pose3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(4),
                    Units.inchesToMeters(0),
                    new Rotation3d(0.0, 0.0, 0.0))),
        Map.entry(16, // Put location here
            new Pose3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(4),
                    Units.inchesToMeters(0),
                    new Rotation3d(0.0, 0.0, 0.0))),
        Map.entry(17, // Put location here
            new Pose3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(4),
                    Units.inchesToMeters(0),
                    new Rotation3d(0.0, 0.0, 0.0))),
        Map.entry(18, // Put location here
            new Pose3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(4),
                    Units.inchesToMeters(0),
                    new Rotation3d(0.0, 0.0, 0.0))),
        Map.entry(19, // Put location here
            new Pose3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(4),
                    Units.inchesToMeters(0),
                    new Rotation3d(0.0, 0.0, 0.0))),
        Map.entry(20, // Put location here
            new Pose3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(4),
                    Units.inchesToMeters(0),
                    new Rotation3d(0.0, 0.0, 0.0))),
        Map.entry(21, // Put location here
            new Pose3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(4),
                    Units.inchesToMeters(0),
                    new Rotation3d(0.0, 0.0, 0.0))),
        Map.entry(22, // Put location here
            new Pose3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(4),
                    Units.inchesToMeters(0),
                    new Rotation3d(0.0, 0.0, 0.0)))
            );
                
}