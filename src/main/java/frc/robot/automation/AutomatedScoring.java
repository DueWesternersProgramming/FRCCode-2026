package frc.robot.automation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.CowboyUtils;
import frc.robot.RobotState;
import frc.robot.RobotConstants.ScoringConstants;
import frc.robot.RobotConstants.ScoringConstants.Setpoints;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;

public class AutomatedScoring {

    public static Command exampleCommandDynamicAuton(Integer exampleParam) {
        return Commands.print("EA Sports: It's in the game! Example Param: " + exampleParam);
    }

    public static Command PPmoveToPose(Pose2d pose) {
        PathConstraints constraints = new PathConstraints(
                3.0, 3.0,
                Units.degreesToRadians(360), Units.degreesToRadians(540));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                pose,
                constraints // Rotation delay distance in meters. This is how far the robot should travel
                            // before attempting to rotate.
        );
        return Commands.deferredProxy(() -> pathfindingCommand);

    }
}