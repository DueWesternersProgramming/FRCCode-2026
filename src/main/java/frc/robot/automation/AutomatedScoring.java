package frc.robot.automation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;


public class AutomatedScoring {

    public static Command exampleCommandDynamicAuton(Integer exampleParam) {
        return Commands.print("EA Sports: It's in the game! Example Param: " + exampleParam);
    }

    public static Command shootFromHopperContinousCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem){
        return (Commands.sequence(
            shooterSubsystem.setPercentSpeedCommand(-1),
            new WaitCommand(.5),
            Commands.parallel(
                intakeSubsystem.runIntakeAgitationContinousCommand(),
                indexerSubsystem.runIndexerAgitationContinousCommand(),
                feederSubsystem.startFeedingBallsCommand()
            )
            ));
    }

    public static Command stopAllSuperStructure(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem){
        return (Commands.sequence(
            Commands.parallel(
                intakeSubsystem.stopIntakingCommand(),
                indexerSubsystem.stopIndexing(),
                feederSubsystem.stopFeedingBallsCommand(),
                shooterSubsystem.setPercentSpeedCommand(0)
            )
            ));
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