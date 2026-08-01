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
    public static final record AprilTagCameraConfig(VisionSource source, int servoPort,
            SimCameraConfig simConfig) {
    }

    public record VisionSource(String name, Transform3d robotToCamera) {
    }
    public record VisionSource(String name, Transform3d robotToCamera) {
    }

    public static final Optional<VisionSystemSim> aprilTagSim = RobotModes.currentMode == RobotModes.simMode
            ? Optional.of(new VisionSystemSim("AprilTagSim"))
            : Optional.empty();
    public static final Optional<VisionSystemSim> aprilTagSim = RobotModes.currentMode == RobotModes.simMode
            ? Optional.of(new VisionSystemSim("AprilTagSim"))
            : Optional.empty();

    // configure our camera objects here
    public static final List<AprilTagCameraConfig> CAMERA_CONFIGS = List.of(
            // Front
            new AprilTagCameraConfig(
                    new VisionSource(
                            "front_camera",
                            new Transform3d(
                                    new Translation3d(
                                            Units.inchesToMeters(
                                                    12.375), // forward+
                                            Units.inchesToMeters(
                                                    0), // left+
                                            Units.inchesToMeters(
                                                    22.5 + 2.75)), // up+
                                    new Rotation3d(
                                            // Counter clockwise
                                            // positive
                                            Units.degreesToRadians(
                                                    0),
                                            Units.degreesToRadians(
                                                    -15),
                                            Units.degreesToRadians(
                                                    0)))),
                    -1,
                    SimCameraConfig.ARDUCAM_OV9281_45),
            // Front Right
            new AprilTagCameraConfig(
                    new VisionSource(
                            "right_camera",
                            new Transform3d(
                                    new Translation3d(
                                            Units.inchesToMeters(
                                                    3.5), // forward+
                                            Units.inchesToMeters(
                                                    -11.5), // left+
                                            Units.inchesToMeters(
                                                    4 + 2.75)), // up+
                                    new Rotation3d(
                                            Units.degreesToRadians(
                                                    0),
                                            Units.degreesToRadians(
                                                    -20),
                                            Units.degreesToRadians(
                                                    -90)))),
                    -1,
                    SimCameraConfig.ARDUCAM_OV9281_45),
            new AprilTagCameraConfig(
                    new VisionSource(
                            "left_camera",
                            new Transform3d(
                                    new Translation3d(
                                            Units.inchesToMeters(
                                                    3.5), // forward+
                                            Units.inchesToMeters(
                                                    11.5), // left+
                                            Units.inchesToMeters(
                                                    4 + 2.75)), // up+
                                    new Rotation3d(
                                            Units.degreesToRadians(
                                                    0),
                                            Units.degreesToRadians(
                                                    -20),
                                            Units.degreesToRadians(
                                                    90)))),
                    -1,
                    SimCameraConfig.ARDUCAM_OV9281_45));
}
