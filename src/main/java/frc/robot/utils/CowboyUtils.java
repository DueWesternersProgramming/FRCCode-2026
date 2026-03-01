package frc.robot.utils;

import java.io.IOException;

import org.opencv.core.Mat;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotConstants.ScoringConstants;
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