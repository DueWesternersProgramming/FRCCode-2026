package frc.robot.automation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotConstants.ScoringConstants.FieldZones;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.CowboyUtils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

public class AutomatedScoring {

        public static Command exampleCommandDynamicAuton(Integer exampleParam) {
                return Commands.print("EA Sports: It's in the game! Example Param: " + exampleParam);
        }

        /**
         * Automaticly agitates and outtakes balls through the shooter while active.
         * @param speed Percent for the shooter wheel to spin at.
         */
        public static Command shootFromHopperContinousCommand(IntakeSubsystem intakeSubsystem,
                        IndexerSubsystem indexerSubsystem, FeederSubsystem feederSubsystem,
                        ShooterSubsystem shooterSubsystem,
                        double speed) {
                return (Commands.sequence(
                                shooterSubsystem.setPercentSpeedCommand(-speed),
                                new WaitCommand(.5),
                                Commands.parallel(
                                                intakeSubsystem.runIntakeAgitationContinousCommand(),
                                                indexerSubsystem.runIndexerAgitationContinousCommand(),
                                                feederSubsystem.startFeedingBallsCommand())));
        }

        /**
         * Takes over robot rotation and automaticly agitates/shoots balls. Allows for translation control for shoot on the move.
         * Uses interpolation for distance, and calculates the angle of the robot needed based on chassis speeds.
         */
        public static Command generalContinuousShootOnMoveAutomationCommand(DriveSubsystem driveSubsystem,
                        Joystick driveJoystick,
                        IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem,
                        FeederSubsystem feederSubsystem,
                        ShooterSubsystem shooterSubsystem) {

                if (CowboyUtils.getFieldZoneFromPose(driveSubsystem.getPose()) == FieldZones.NEUTRAL_ZONE) { // In the
                                                                                                             // center
                                                                                                             // feeding
                                                                                                             // to a
                                                                                                             // spot on
                                                                                                             // your
                                                                                                             // zone
                        return (Commands.parallel(
                                        new InterpolateShootCommand(driveSubsystem, shooterSubsystem, driveJoystick, CowboyUtils.getAllianceFeedingPosition()),
                                        Commands.sequence(
                                                        new WaitCommand(.5),
                                                        Commands.parallel(
                                                                        intakeSubsystem.runIntakeAgitationContinousCommand(),
                                                                        indexerSubsystem.runIndexerAgitationContinousCommand(),
                                                                        feederSubsystem.startFeedingBallsCommand()))));
                } else { // In an alliance zone for scoring in the hub
                        return (Commands.parallel(
                                        new InterpolateShootCommand(driveSubsystem, shooterSubsystem, driveJoystick, CowboyUtils.getAllianceHubPose()),
                                        Commands.sequence(
                                                        new WaitCommand(.5),
                                                        Commands.parallel(
                                                                        intakeSubsystem.runIntakeAgitationContinousCommand(),
                                                                        indexerSubsystem.runIndexerAgitationContinousCommand(),
                                                                        feederSubsystem.startFeedingBallsCommand()))));
                }
        }

        /**
         * Stops all super structure and ball control systems.
         */
        public static Command stopAllSuperStructure(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem,
                        FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem) {
                return (Commands.sequence(
                                Commands.parallel(
                                                intakeSubsystem.stopIntakingCommand(),
                                                indexerSubsystem.stopIndexing(),
                                                feederSubsystem.stopFeedingBallsCommand(),
                                                shooterSubsystem.setPercentSpeedCommand(0))));
        }

        public static Command PPmoveToPose(Pose2d pose) {
                PathConstraints constraints = new PathConstraints(
                                3.0, 3.0,
                                Units.degreesToRadians(360), Units.degreesToRadians(540));

                // Since AutoBuilder is configured, we can use it to build pathfinding commands
                Command pathfindingCommand = AutoBuilder.pathfindToPose(
                                pose,
                                constraints // Rotation delay distance in meters. This is how far the robot should
                                            // travel
                                            // before attempting to rotate.
                );
                return Commands.deferredProxy(() -> pathfindingCommand);

        }
}