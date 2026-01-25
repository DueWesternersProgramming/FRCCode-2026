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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotConstants.ScoringConstants;
import edu.wpi.first.wpilibj.RobotBase;

public class CowboyUtils {

    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.kDefaultField);

    public static Pose2d testPose = new Pose2d(1.4, 5.55, new Rotation2d(Math.toRadians(0)));

    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent() ? (DriverStation.getAlliance().get() == Alliance.Red) : (false);
    }

    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().isPresent() ? (DriverStation.getAlliance().get() == Alliance.Blue) : (false);
    }

    public static boolean isSim() {
        return RobotBase.isSimulation();
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
        double xOffset = pose2.getX() - pose1.getX();

        double yOffset = pose2.getY() - pose1.getY();

        return new Rotation2d(Math.atan(yOffset / xOffset));

    }

    // public static Pose2d getPoseAlongArcFromY(Pose2d robotPose, Pose2d rotationCenterPose, double radius) {

    //     double currentOffX = robotPose.getX() - rotationCenterPose.getX();
    //     doubluble currentOffY = robotPose.getY() - rotationCenterPose.getY();

    //     double hubSideTriangleAngle = Math.atan2(currentOffY, currentOffX);

    //     double newOffX = Math.cos(hubSideTriangleAngle) * radius;
    //     double newOffY = Math.sin(hubSideTriangleAngle) * radius;

    //     double newX = rotationCenterPose.getX() + newOffX;
    //     double newY = rotationCenterPose.getY() + newOffY;

    //     return new Pose2d(newX, newY, getAngleToPose(robotPose, rotationCenterPose));
    // }

    public static Pose2d getAllianceHubPose() {
        return isBlueAlliance() ? ScoringConstants.BLUE_ALLIANCE_HUB
                : FlippingUtil.flipFieldPose(ScoringConstants.BLUE_ALLIANCE_HUB);
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