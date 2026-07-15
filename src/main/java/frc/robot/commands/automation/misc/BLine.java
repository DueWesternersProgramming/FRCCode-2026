package frc.robot.commands.automation.misc;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.drive.DriveSubsystem;

public class BLine {
    public static Command BLineTrajectory(DriveSubsystem driveSubsystem, String name, boolean mirrorVertically) {
        Path path = new Path(name);

        if (mirrorVertically) {
            path.mirror();
        }

        return Commands.sequence(driveSubsystem.getBLineBuilder().build(path))
                .beforeStarting(Commands.print(name + ": start"))
                .andThen(Commands.print(name + ": end"));
    }

    public static Command BLineTrajectory(DriveSubsystem driveSubsystem, Path path, boolean mirrorVertically) {
        if (mirrorVertically) {
            path.mirror();
        }

        return Commands.sequence(driveSubsystem.getBLineBuilder().build(path));
    }

    public static Path getPathFromFile(String name) {
        return new Path(name);
    }

}
