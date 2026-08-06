package frc.robot.autonomous;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.automation.misc.DriveToPoseCommand;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.CowboyUtils;

public class BLine {
    public static Command BLineTrajectory(DriveSubsystem driveSubsystem, String name, boolean mirrorVertically) {
        Path path = new Path(name);

        if (mirrorVertically) {
            path.mirror();
        }

        Pose2d startPose = CowboyUtils.isRedAlliance()
                ? FlippingUtil.flipFieldPose(path.getStartPose())
                : path.getStartPose();

        return Commands.either(
                driveSubsystem.getBLineBuilder().build(path),
                Commands.sequence(
                        new DriveToPoseCommand(driveSubsystem, () -> startPose),
                        driveSubsystem.getBLineBuilder().build(path)),
                () -> driveSubsystem.getPose()
                        .getTranslation()
                        .getDistance(startPose.getTranslation()) < 0.10
                        && Math.abs(
                                driveSubsystem.getPose()
                                        .getRotation()
                                        .minus(startPose.getRotation())
                                        .getDegrees()) < 5.0)
                .beforeStarting(Commands.print(name + ": start"))
                .andThen(Commands.print(name + ": end"));
    }

    public static Command BLineTrajectory(DriveSubsystem driveSubsystem, Path path, boolean mirrorVertically) {
        if (mirrorVertically) {
            path.mirror();
        }

        Pose2d startPose = CowboyUtils.isRedAlliance()
                ? FlippingUtil.flipFieldPose(path.getStartPose())
                : path.getStartPose();
        
        System.out.println("Start Pose: " + startPose);

        return Commands.either(
                driveSubsystem.getBLineBuilder().build(path),
                Commands.sequence(
                        new DriveToPoseCommand(driveSubsystem, () -> startPose),
                        driveSubsystem.getBLineBuilder().build(path)),
                () -> driveSubsystem.getPose()
                        .getTranslation()
                        .getDistance(startPose.getTranslation()) < 0.10
                        && Math.abs(
                                driveSubsystem.getPose()
                                        .getRotation()
                                        .minus(startPose.getRotation())
                                        .getDegrees()) < 5.0);
    }

    public static Path getPath(String name, boolean mirrorVertically) {
        Path path = new Path(name);

        if (mirrorVertically) {
            path.mirror();
        }

        return path;
    }

    public static String getPathJson(String name) {
        try {
            java.nio.file.Path file = java.nio.file.Paths.get(
                    Filesystem.getDeployDirectory().getAbsolutePath(),
                    "autos",
                    "paths",
                    name + ".json");

            return java.nio.file.Files.readString(file);

        } catch (IOException e) {
            throw new RuntimeException(
                    "Failed to load B-Line path JSON: " + name,
                    e);
        }
    }
}
