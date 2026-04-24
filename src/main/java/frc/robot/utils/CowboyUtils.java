package frc.robot.utils;

import java.util.Optional;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotState;
import frc.robot.RobotConstants.ScoringConstants;
import frc.robot.RobotConstants.ScoringConstants.BumpLabels;
import frc.robot.RobotConstants.ScoringConstants.FieldZones;
import edu.wpi.first.wpilibj.RobotBase;

public class CowboyUtils {

    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltWelded);

    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent() ? (DriverStation.getAlliance().get() == Alliance.Red) : (false);
    }

    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().isPresent() ? (DriverStation.getAlliance().get() == Alliance.Blue) : (false);
    }

    public static boolean isSim() {
        return RobotBase.isSimulation();
    }

    public static Pose2d getAllianceHubPose() {
        return isBlueAlliance() ? ScoringConstants.BLUE_ALLIANCE_HUB
                : FlippingUtil.flipFieldPose(ScoringConstants.BLUE_ALLIANCE_HUB);
    }

    public static Pose2d getAppropriateFeedingPose() {
        if (isBlueAlliance()) {
            if (RobotState.robotPose.getY() > 4) {
                System.out.println("RIGHT");
                return ScoringConstants.BLUE_ALLIANCE_LEFT_FEEDING_TARGET;
            } else {
                System.out.println("LEFT");
                return ScoringConstants.BLUE_ALLIANCE_RIGHT_FEEDING_TARGET;
            }
        }
        else {
            if (RobotState.robotPose.getY() < 4) {
                return FlippingUtil.flipFieldPose(ScoringConstants.BLUE_ALLIANCE_RIGHT_FEEDING_TARGET);
            } else {
                return FlippingUtil.flipFieldPose(ScoringConstants.BLUE_ALLIANCE_LEFT_FEEDING_TARGET);
            }
        }
    }

    public static BumpLabels getClosestBump(Pose2d robotPose) {
        BumpLabels closestLabel = null;
        double minDistance = Double.MAX_VALUE;

        for (BumpLabels label : BumpLabels.values()) {
            Pose2d bumpPose = getBumpPosition(label);

            double distance = robotPose.getTranslation().getDistance(bumpPose.getTranslation());

            if (distance < minDistance) {
                minDistance = distance;
                closestLabel = label;
            }
        }

        return closestLabel;
    }

    public static Pose2d getBumpPosition(BumpLabels label) {
        switch (label) {
            case BLUE_LEFT:
                return ScoringConstants.BUMP_POSITION_POSES[0][0];
            case BLUE_RIGHT:
                return ScoringConstants.BUMP_POSITION_POSES[0][1];
            case RED_LEFT:
                return ScoringConstants.BUMP_POSITION_POSES[1][0];
            case RED_RIGHT:
                return ScoringConstants.BUMP_POSITION_POSES[1][1];
            default:
                return null; // Never will happen
        }
    }

    public static FieldZones getFieldZoneFromPose(Pose2d pose) {
        double x = pose.getX();
        double blueThreshold = ScoringConstants.BLUE_ALLIANCE_HUB.getX();
        double redThreshold = FlippingUtil.flipFieldPose(ScoringConstants.BLUE_ALLIANCE_HUB).getX();

        if (x < blueThreshold) {
            return FieldZones.BLUE_ZONE;
        } else if (x > blueThreshold && x < redThreshold) {
            return FieldZones.NEUTRAL_ZONE;
        } else if (x > redThreshold) {
            return FieldZones.RED_ZONE;
        }
        return FieldZones.NEUTRAL_ZONE;

    }

    /**
     * @see https://en.wikipedia.org/wiki/Vector_projection#Scalar_projection
     */
    public static double getParallelError(Pose2d origin, Pose2d target) {
        Translation2d originToTarget = origin.minus(target).getTranslation();
        Rotation2d angleBetween = originToTarget.getAngle();
        double parallelError = originToTarget.getNorm() * angleBetween.getSin();

        return parallelError;

        // return origin.minus(target).getY();
    }

    /**
     * @see https://en.wikipedia.org/wiki/Vector_projection#Scalar_projection
     */
    public static double getPerpendicularError(Pose2d origin, Pose2d target) {
        Translation2d originToTarget = origin.minus(target).getTranslation();
        Rotation2d angleBetween = originToTarget.getAngle();
        double perpendicularError = originToTarget.getNorm() * angleBetween.getCos();

        return perpendicularError;

        // return -origin.minus(target).getX();
    }

    private static Rotation2d getAngleToPose(Pose2d pose1, Pose2d pose2) {
        double dx = pose1.getX() - pose2.getX();
        double dy = pose1.getY() - pose2.getY();

        return new Rotation2d(Units.degreesToRadians(Math.atan2(dy, dx)));

    }

    public static final class RobotModes {
        public static Mode simMode = Mode.SIM;
        public static Mode replayMode = Mode.REPLAY;
        public static Mode realMode = Mode.REAL;
        public static Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

        public static enum Mode {
            /** Running on a real robot. */
            REAL,

            /** Running a physics simulator. */
            SIM,

            /** Replaying from a log file. */
            REPLAY
        }

    }
}