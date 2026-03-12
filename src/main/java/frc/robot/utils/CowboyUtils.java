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

    public static Pose2d getAllianceFeedingPosition() {
        return isBlueAlliance() ? ScoringConstants.BLUE_ALLIANCE_FEEDING_TARGET
                : FlippingUtil.flipFieldPose(ScoringConstants.BLUE_ALLIANCE_FEEDING_TARGET);
    }


    public static boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its
        // likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            return true;
        } else if (matchTime > 105) {
            return shift1Active;
        } else if (matchTime > 80) {
            return !shift1Active;
        } else if (matchTime > 55) {
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
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
        double blueThreshold = getAllianceHubPose().getX();
        double redThreshold = getAllianceHubPose().getX();

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