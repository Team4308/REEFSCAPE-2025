package frc.robot;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class FieldLayout {
    /**
     * Origin is the bottom left corner of the field image (Close right corner from
     * blue driver station POV)
     */

    // Everything in Meters
    public static final double kFieldLength = Units.inchesToMeters(690.875);
    public static final double kFieldWidth = Units.inchesToMeters(317.0);
    public static final double kTapeWidth = Units.inchesToMeters(2.0);

    public static class Reef {
        // REEF SCORING POSES
        public static final Pose2d A = new Pose2d(Units.inchesToMeters(126.900515), Units.inchesToMeters(169.573852),
                Rotation2d.fromDegrees(0));
        public static final Pose2d B = new Pose2d(Units.inchesToMeters(126.741216), Units.inchesToMeters(157.074867),
                Rotation2d.fromDegrees(0));
        public static final Pose2d C = new Pose2d(Units.inchesToMeters(142.199234), Units.inchesToMeters(121.144440),
                Rotation2d.fromDegrees(60));
        public static final Pose2d D = new Pose2d(Units.inchesToMeters(152.944023), Units.inchesToMeters(114.756990),
                Rotation2d.fromDegrees(60));
        public static final Pose2d E = new Pose2d(Units.inchesToMeters(191.601978), Units.inchesToMeters(109.630609),
                Rotation2d.fromDegrees(120));
        public static final Pose2d F = new Pose2d(Units.inchesToMeters(202.942230), Units.inchesToMeters(115.98605),
                Rotation2d.fromDegrees(120));
        public static final Pose2d G = new Pose2d(Units.inchesToMeters(226.790443), Units.inchesToMeters(147.381161),
                Rotation2d.fromDegrees(180));
        public static final Pose2d H = new Pose2d(Units.inchesToMeters(226.733095), Units.inchesToMeters(160.380105),
                Rotation2d.fromDegrees(180));
        public static final Pose2d I = new Pose2d(Units.inchesToMeters(211.477828), Units.inchesToMeters(196.336729),
                Rotation2d.fromDegrees(-120));
        public static final Pose2d J = new Pose2d(Units.inchesToMeters(200.303248), Units.inchesToMeters(202.979677),
                Rotation2d.fromDegrees(-120));
        public static final Pose2d K = new Pose2d(Units.inchesToMeters(161.688832), Units.inchesToMeters(207.179848),
                Rotation2d.fromDegrees(-60));
        public static final Pose2d L = new Pose2d(Units.inchesToMeters(150.348580), Units.inchesToMeters(201.073852),
                Rotation2d.fromDegrees(-60));

        // public static final List<Pose2d> BLUE_REEF_POSES = List.of(A, B, C, D, E, F, G, H, I, J, K, L);
        // public static final List<Pose2d> RED_REEF_POSES = getRedReefPoses();
        public static final List<Pose2d> BLUE_LEFT_REEF_POSES = List.of(A, C, E, G, I, K);
        public static final List<Pose2d> BLUE_RIGHT_REEF_POSES = List.of(B, D, F, H, J, L);

        public static final List<Pose2d> RED_LEFT_REEF_POSES = getRedLeftReefPoses();
        public static final List<Pose2d> RED_RIGHT_REEF_POSES = getRedRightReefPoses();
        

        // CORAL STATION POSES
        public static final Pose2d LEFT_CORAL_STATION_FAR = new Pose2d(Units.inchesToMeters(28.7), Units.inchesToMeters(59.037), Rotation2d.fromDegrees(-54.5)); // or 36??
                                                                                                             // idek
        public static final Pose2d LEFT_CORAL_STATION_NEAR = new Pose2d(Units.inchesToMeters(52.059), Units.inchesToMeters(26.566), Rotation2d.fromDegrees(-54.5));
        public static final Pose2d RIGHT_CORAL_STATION_FAR = new Pose2d(0, 0, Rotation2d.fromDegrees(55));
        public static final Pose2d RIGHT_CORAL_STATION_NEAR = new Pose2d(0, 0, Rotation2d.fromDegrees(55));

        public static final List<Pose2d> BLUE_CORAL_STATION_POSES = List.of(LEFT_CORAL_STATION_FAR,
                LEFT_CORAL_STATION_NEAR, RIGHT_CORAL_STATION_FAR, RIGHT_CORAL_STATION_NEAR);
        public static final List<Pose2d> RED_CORAL_STATION_POSES = getRedCoralStationPoses();
    }

    public static Pose2d getRedAlliancePose(Pose2d bluePose) {
        return new Pose2d(kFieldLength - bluePose.getX(),
                kFieldWidth - bluePose.getY(),
                bluePose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    public static List<Pose2d> getRedLeftReefPoses() {
        Pose2d[] RED_LEFT_REEF_POSES = new Pose2d[Reef.BLUE_LEFT_REEF_POSES.size()];
        for (int i = 0; i < Reef.BLUE_LEFT_REEF_POSES.size(); i++) {
            RED_LEFT_REEF_POSES[i] = getRedAlliancePose(Reef.BLUE_LEFT_REEF_POSES.get(i));
        }
        return List.of(RED_LEFT_REEF_POSES);
    }

    public static List<Pose2d> getRedRightReefPoses() {
        Pose2d[] RED_RIGHT_REEF_POSES = new Pose2d[Reef.BLUE_RIGHT_REEF_POSES.size()];
        for (int i = 0; i < Reef.BLUE_RIGHT_REEF_POSES.size(); i++) {
            RED_RIGHT_REEF_POSES[i] = getRedAlliancePose(Reef.BLUE_RIGHT_REEF_POSES.get(i));
        }
        return List.of(RED_RIGHT_REEF_POSES);
    }

    public static List<Pose2d> getRedCoralStationPoses() {
        Pose2d[] RED_CORAL_STATION_POSES = new Pose2d[Reef.BLUE_CORAL_STATION_POSES.size()];
        for (int i = 0; i < Reef.BLUE_CORAL_STATION_POSES.size(); i++) {
            RED_CORAL_STATION_POSES[i] = getRedAlliancePose(Reef.BLUE_CORAL_STATION_POSES.get(i));
        }
        return List.of(RED_CORAL_STATION_POSES);
    }

//     public static List<Pose2d> getRedReefPoses() {
//         Pose2d[] RED_REEF_POSES = new Pose2d[Reef.BLUE_REEF_POSES.size()];
//         for (int i = 0; i < Reef.BLUE_REEF_POSES.size(); i++) {
//             RED_REEF_POSES[i] = getRedAlliancePose(Reef.BLUE_REEF_POSES.get(i));
//         }
//         return List.of(RED_REEF_POSES);
//     }

    // AprilTag locations
    /*
     * public static final Map<Integer, Pose3d> aprilTags = Map.ofEntries(
     * Map.entry(1, // Put location here
     * new Pose3d(
     * Units.inchesToMeters(0),
     * Units.inchesToMeters(4),
     * Units.inchesToMeters(0),
     * new Rotation3d(0.0, 0.0, 0.0))),
     * Map.entry(2, // Put location here
     * new Pose3d(
     * Units.inchesToMeters(0),
     * Units.inchesToMeters(4),
     * Units.inchesToMeters(0),
     * new Rotation3d(0.0, 0.0, 0.0))),
     * Map.entry(3, // Put location here
     * new Pose3d(
     * Units.inchesToMeters(0),
     * Units.inchesToMeters(4),
     * Units.inchesToMeters(0),
     * new Rotation3d(0.0, 0.0, 0.0))),
     * Map.entry(4, // Put location here
     * new Pose3d(
     * Units.inchesToMeters(0),
     * Units.inchesToMeters(4),
     * Units.inchesToMeters(0),
     * new Rotation3d(0.0, 0.0, 0.0))),
     * Map.entry(5, // Put location here
     * new Pose3d(
     * Units.inchesToMeters(0),
     * Units.inchesToMeters(4),
     * Units.inchesToMeters(0),
     * new Rotation3d(0.0, 0.0, 0.0))),
     * Map.entry(6, // Put location here
     * new Pose3d(
     * Units.inchesToMeters(0),
     * Units.inchesToMeters(4),
     * Units.inchesToMeters(0),
     * new Rotation3d(0.0, 0.0, 0.0))),
     * Map.entry(7, // Put location here
     * new Pose3d(
     * Units.inchesToMeters(0),
     * Units.inchesToMeters(4),
     * Units.inchesToMeters(0),
     * new Rotation3d(0.0, 0.0, 0.0))),
     * Map.entry(8, // Put location here
     * new Pose3d(
     * Units.inchesToMeters(0),
     * Units.inchesToMeters(4),
     * Units.inchesToMeters(0),
     * new Rotation3d(0.0, 0.0, 0.0))),
     * Map.entry(9, // Put location here
     * new Pose3d(
     * Units.inchesToMeters(0),
     * Units.inchesToMeters(4),
     * Units.inchesToMeters(0),
     * new Rotation3d(0.0, 0.0, 0.0))),
     * Map.entry(10, // Put location here
     * new Pose3d(
     * Units.inchesToMeters(0),
     * Units.inchesToMeters(4),
     * Units.inchesToMeters(0),
     * new Rotation3d(0.0, 0.0, 0.0))),
     * Map.entry(11, // Put location here
     * new Pose3d(
     * Units.inchesToMeters(0),
     * Units.inchesToMeters(4),
     * Units.inchesToMeters(0),
     * new Rotation3d(0.0, 0.0, 0.0))),
     * Map.entry(12, // Put location here
     * new Pose3d(
     * Units.inchesToMeters(0),
     * Units.inchesToMeters(4),
     * Units.inchesToMeters(0),
     * new Rotation3d(0.0, 0.0, 0.0))),
     * Map.entry(13, // Put location here
     * new Pose3d(
     * Units.inchesToMeters(0),
     * Units.inchesToMeters(4),
     * Units.inchesToMeters(0),
     * new Rotation3d(0.0, 0.0, 0.0))),
     * Map.entry(14, // Put location here
     * new Pose3d(
     * Units.inchesToMeters(0),
     * Units.inchesToMeters(4),
     * Units.inchesToMeters(0),
     * new Rotation3d(0.0, 0.0, 0.0))),
     * Map.entry(15, // Put location here
     * new Pose3d(
     * Units.inchesToMeters(0),
     * Units.inchesToMeters(4),
     * Units.inchesToMeters(0),
     * new Rotation3d(0.0, 0.0, 0.0))),
     * Map.entry(16, // Put location here
     * new Pose3d(
     * Units.inchesToMeters(0),
     * Units.inchesToMeters(4),
     * Units.inchesToMeters(0),
     * new Rotation3d(0.0, 0.0, 0.0))),
     * Map.entry(17, // Put location here
     * new Pose3d(
     * Units.inchesToMeters(0),
     * Units.inchesToMeters(4),
     * Units.inchesToMeters(0),
     * new Rotation3d(0.0, 0.0, 0.0))),
     * Map.entry(18, // Put location here
     * new Pose3d(
     * Units.inchesToMeters(0),
     * Units.inchesToMeters(4),
     * Units.inchesToMeters(0),
     * new Rotation3d(0.0, 0.0, 0.0))),
     * Map.entry(19, // Put location here
     * new Pose3d(
     * Units.inchesToMeters(0),
     * Units.inchesToMeters(4),
     * Units.inchesToMeters(0),
     * new Rotation3d(0.0, 0.0, 0.0))),
     * Map.entry(20, // Put location here
     * new Pose3d(
     * Units.inchesToMeters(0),
     * Units.inchesToMeters(4),
     * Units.inchesToMeters(0),
     * new Rotation3d(0.0, 0.0, 0.0))),
     * Map.entry(21, // Put location here
     * new Pose3d(
     * Units.inchesToMeters(0),
     * Units.inchesToMeters(4),
     * Units.inchesToMeters(0),
     * new Rotation3d(0.0, 0.0, 0.0))),
     * Map.entry(22, // Put location here
     * new Pose3d(
     * Units.inchesToMeters(0),
     * Units.inchesToMeters(4),
     * Units.inchesToMeters(0),
     * new Rotation3d(0.0, 0.0, 0.0)))
     * );
     */

}