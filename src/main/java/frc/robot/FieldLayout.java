package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class FieldLayout {
    /**
     * Origin is the bottom left corner of the field image (Close right corner from blue driver station POV)
     */

    // Everything in Meters
    public static final double kFieldLength = Units.inchesToMeters(690.875);
    public static final double kFieldWidth = Units.inchesToMeters(317.0);
    public static final double kTapeWidth = Units.inchesToMeters(2.0);

    public static class REEF {
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

        public static final List<Pose2d> BLUE_LEFT_REEF_POSES = List.of(A, C, E, G, I, K);
        public static final List<Pose2d> BLUE_RIGHT_REEF_POSES = List.of(B, D, F, H, J, L);

        public static final List<Pose2d> RED_LEFT_REEF_POSES = new ArrayList<>(getRedLeftReefPoses());
        public static final List<Pose2d> RED_RIGHT_REEF_POSES = new ArrayList<>(getRedRightReefPoses());
        
    }

    public static class ALGAE {
        public static final Pose2d AB = new Pose2d(Units.inchesToMeters(123.935946), Units.inchesToMeters(163.037678), Rotation2d.fromDegrees(0));
        public static final Pose2d CD = new Pose2d(Units.inchesToMeters(146.409216), Units.inchesToMeters(114.944904), Rotation2d.fromDegrees(60));
        public static final Pose2d EF = new Pose2d(Units.inchesToMeters(199.237790), Units.inchesToMeters(110.492543), Rotation2d.fromDegrees(120));
        public static final Pose2d GH = new Pose2d(Units.inchesToMeters(229.453741), Units.inchesToMeters(153.839701), Rotation2d.fromDegrees(180));
        public static final Pose2d IJ = new Pose2d(Units.inchesToMeters(207.172845), Units.inchesToMeters(201.812875), Rotation2d.fromDegrees(-120));
        public static final Pose2d KL = new Pose2d(Units.inchesToMeters(154.377247), Units.inchesToMeters(206.570858), Rotation2d.fromDegrees(-60));

        public static final List<Pose2d> BLUE_ALGAE_POSES = List.of(AB, CD, EF, GH, IJ, KL);
        public static final List<Pose2d> RED_ALGAE_POSES = new ArrayList<>(getRedAlgaePoses());
    }

    public static class CORAL_STATION {
        // CORAL STATION INTAKING POSES
        public static final Pose2d RIGHT_FAR = new Pose2d(Units.inchesToMeters(59.406379), Units.inchesToMeters(28.178468), Rotation2d.fromDegrees(54));                                                                                                              // idek
        public static final Pose2d RIGHT_NEAR = new Pose2d(Units.inchesToMeters(26.571192), Units.inchesToMeters(51.887127), Rotation2d.fromDegrees(54));
        public static final Pose2d LEFT_FAR = new Pose2d(Units.inchesToMeters(60.171594), Units.inchesToMeters(289.529997), Rotation2d.fromDegrees(-54));
        public static final Pose2d LEFT_NEAR = new Pose2d(Units.inchesToMeters(27.880330), Units.inchesToMeters(265.923340), Rotation2d.fromDegrees(-54));

        public static final List<Pose2d> BLUE_CORAL_STATION_POSES = List.of(LEFT_FAR, LEFT_NEAR, RIGHT_FAR, RIGHT_NEAR);
        public static final List<Pose2d> RED_CORAL_STATION_POSES = new ArrayList<>(getRedCoralStationPoses());
    }

    public static Pose2d getRedAlliancePose(Pose2d bluePose) {
        return new Pose2d(kFieldLength - bluePose.getX(),
                kFieldWidth - bluePose.getY(),
                bluePose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    public static List<Pose2d> getRedLeftReefPoses() {
        Pose2d[] RED_LEFT_REEF_POSES = new Pose2d[REEF.BLUE_LEFT_REEF_POSES.size()];
        for (int i = 0; i < REEF.BLUE_LEFT_REEF_POSES.size(); i++) {
            RED_LEFT_REEF_POSES[i] = getRedAlliancePose(REEF.BLUE_LEFT_REEF_POSES.get(i));
        };
        return new ArrayList<>(Arrays.asList(RED_LEFT_REEF_POSES));
    }

    public static List<Pose2d> getRedRightReefPoses() {
        Pose2d[] RED_RIGHT_REEF_POSES = new Pose2d[REEF.BLUE_RIGHT_REEF_POSES.size()];
        for (int i = 0; i < REEF.BLUE_RIGHT_REEF_POSES.size(); i++) {
            RED_RIGHT_REEF_POSES[i] = getRedAlliancePose(REEF.BLUE_RIGHT_REEF_POSES.get(i));
        }
        return new ArrayList<>(Arrays.asList(RED_RIGHT_REEF_POSES));
    }

    public static List<Pose2d> getRedAlgaePoses(){
        Pose2d[] RED_ALGAE_POSES = new Pose2d[ALGAE.BLUE_ALGAE_POSES.size()];
        for (int i = 0; i < ALGAE.BLUE_ALGAE_POSES.size(); i++) {
                RED_ALGAE_POSES[i] = getRedAlliancePose(ALGAE.BLUE_ALGAE_POSES.get(i));
        }
        return new ArrayList<>(Arrays.asList(RED_ALGAE_POSES));
    }

    public static List<Pose2d> getRedCoralStationPoses() {
        Pose2d[] RED_CORAL_STATION_POSES = new Pose2d[CORAL_STATION.BLUE_CORAL_STATION_POSES.size()];
        for (int i = 0; i < CORAL_STATION.BLUE_CORAL_STATION_POSES.size(); i++) {
            RED_CORAL_STATION_POSES[i] = getRedAlliancePose(CORAL_STATION.BLUE_CORAL_STATION_POSES.get(i));
        }
        return new ArrayList<>(Arrays.asList(RED_CORAL_STATION_POSES));
    }
}