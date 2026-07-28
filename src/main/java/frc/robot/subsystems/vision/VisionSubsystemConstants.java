package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.CowboyUtils.RobotModes;

public class VisionSubsystemConstants {

                public static final record AprilTagCameraConfig(VisionSource source, int servoPort,
                                SimCameraConfig simConfig) {
                }

                public record VisionSource(String name, Transform3d robotToCamera) {
                }

                public static final Optional<VisionSystemSim> aprilTagSim = RobotModes.currentMode == RobotModes.simMode
                                ? Optional.of(new VisionSystemSim("AprilTagSim"))
                                : Optional.empty();

                // configure our camera objects here
                public static final List<AprilTagCameraConfig> CAMERA_CONFIGS = List.of(
                                // Front Left
                                new AprilTagCameraConfig(
                                                new VisionSource(
                                                                "frontLeftCamera",
                                                                new Transform3d(
                                                                                new Translation3d(
                                                                                                Units.inchesToMeters(
                                                                                                                0.018), // forward+
                                                                                                Units.inchesToMeters(
                                                                                                                7.129), // left+
                                                                                                Units.inchesToMeters(
                                                                                                                25.318)), // up+
                                                                                new Rotation3d(
                                                                                                // Counter clockwise
                                                                                                // positive
                                                                                                Units.degreesToRadians(
                                                                                                                0),
                                                                                                Units.degreesToRadians(
                                                                                                                -20),
                                                                                                Units.degreesToRadians(
                                                                                                                -15)))),
                                                -1,
                                                SimCameraConfig.ARDUCAM_OV9281_45),
                                // Front Right
                                new AprilTagCameraConfig(
                                                new VisionSource(
                                                                "frontRightCamera",
                                                                new Transform3d(
                                                                                new Translation3d(
                                                                                                Units.inchesToMeters(
                                                                                                                0.018), // forward+
                                                                                                Units.inchesToMeters(
                                                                                                                -7.129), // left+
                                                                                                Units.inchesToMeters(
                                                                                                                25.318)), // up+
                                                                                new Rotation3d(
                                                                                                Units.degreesToRadians(
                                                                                                                0),
                                                                                                Units.degreesToRadians(
                                                                                                                -20),
                                                                                                Units.degreesToRadians(
                                                                                                                15)))),
                                                -1,
                                                SimCameraConfig.ARDUCAM_OV9281_45));
}
