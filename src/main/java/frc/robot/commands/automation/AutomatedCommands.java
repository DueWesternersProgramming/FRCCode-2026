package frc.robot.commands.automation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotConstants.LEDConstants.LEDModes;
import frc.robot.RobotConstants.ScoringConstants.FieldZones;
import frc.robot.commands.automation.interpolation.shootOnMoveInterpolationCommand;
import frc.robot.commands.automation.interpolation.shootSimpleInterpolationCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.CowboyUtils;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

/**
 * This class is used to automate the 'superstructure' of our robot - this is
 * where any automation
 * should go where commands are built up and abstracted away into a single
 * command.
 */
public class AutomatedCommands {

        public static Command exampleCommandDynamicAuton(Integer exampleParam) {
                return Commands.print("EA Sports: It's in the game! Example Param: " + exampleParam);
        }

        public static Command intakeCommand(IntakeSubsystem intakeSubsystem, LEDSubsystem ledSubsystem) {
                return Commands.parallel(
                                new SequentialCommandGroup(
                                                intakeSubsystem.setIntakeSpeedCommand(-.3), new WaitCommand(.1),
                                                intakeSubsystem.setIntakeSpeedCommand(1)),
                                ledSubsystem.setLEDModeCommand(LEDModes.INTAKING));
        }

        public static Command reverseSuperstructure(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem,
                        FeederSubsystem feederSubsystem, LEDSubsystem ledSubsystem) {
                return Commands.parallel(indexerSubsystem.setIndexerSpeedCommand(-.3, -1),
                                intakeSubsystem.setIntakeSpeedCommand(-.6), feederSubsystem.setFeederSpeed(-.5),
                                ledSubsystem.setLEDModeCommand(LEDModes.REVERSING));
        }

        /**
         * Agitates and outtakes balls through the shooter while active.
         * 
         * @param target Placement of the balls, using interpolation for distance. No
         *               rotation control here.
         */
        public static Command shootFromHopperContinousCommand(
                        IntakeSubsystem intakeSubsystem,
                        IndexerSubsystem indexerSubsystem,
                        FeederSubsystem feederSubsystem,
                        ShooterSubsystem shooterSubsystem,
                        Supplier<Pose2d> targetSupplier) {

                return Commands.parallel(
                                new WaitCommand(.5),
                                new shootSimpleInterpolationCommand(shooterSubsystem, targetSupplier),
                                Commands.sequence(
                                                new WaitCommand(.5),
                                                Commands.parallel(
                                                                intakeSubsystem.runIntakeAgitationContinousCommand(),
                                                                indexerSubsystem.runIndexerAgitationContinousCommand(),
                                                                feederSubsystem.startFeedingBallsCommand())));
        }

        /**
         * Automaticly agitates and outtakes balls through the shooter while active.
         * 
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
         * Takes over robot rotation and automaticly agitates/shoots balls. Allows for
         * translation control for shooting on the move.
         * Uses interpolation for distance, and calculates the angle of the robot needed
         * based on chassis speeds.
         */
        public static Command teleopShootOnMoveAutomationCommand(DriveSubsystem driveSubsystem,
                        Joystick driveJoystick,
                        IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem,
                        FeederSubsystem feederSubsystem,
                        ShooterSubsystem shooterSubsystem, LEDSubsystem ledSubsystem) {

                return Commands.defer(() -> {
                        if (CowboyUtils.getFieldZoneFromPose(
                                        driveSubsystem.getPose()) != (CowboyUtils.isRedAlliance() ? FieldZones.RED_ZONE
                                                        : FieldZones.BLUE_ZONE)) { // Passing shots
                                return Commands.parallel(
                                                new shootOnMoveInterpolationCommand(driveSubsystem, shooterSubsystem,
                                                                driveJoystick,
                                                                () -> CowboyUtils.getAllianceFeedingPosition()),
                                                Commands.sequence(
                                                                new WaitCommand(.5),
                                                                Commands.parallel(
                                                                                intakeSubsystem.runIntakeAgitationContinousCommand(),
                                                                                indexerSubsystem.runIndexerAgitationContinousCommand(),
                                                                                feederSubsystem.startFeedingBallsCommand())),
                                                ledSubsystem.setLEDModeCommand(LEDModes.SHOOTING));
                        } else { // Shooting spots
                                return Commands.parallel(
                                                //new FunctionalCommand(()->{}, null, null, null, null);
                                                new shootOnMoveInterpolationCommand(driveSubsystem, shooterSubsystem,
                                                                driveJoystick, () -> CowboyUtils.getAllianceHubPose()),
                                                Commands.sequence(
                                                                new WaitCommand(.5),
                                                                Commands.parallel(
                                                                                intakeSubsystem.runIntakeAgitationContinousCommand(),
                                                                                indexerSubsystem.runIndexerAgitationContinousCommand(),
                                                                                feederSubsystem.startFeedingBallsCommand())),
                                                ledSubsystem.setLEDModeCommand(LEDModes.SHOOTING));
                        }
                }, RobotContainer.allSubsystemsSet);
        }

        /**
         * Stops all super structure and ball control systems.
         */
        public static Command stopAllSuperStructure(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem,
                        FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem, LEDSubsystem ledSubsystem) {
                return (Commands.sequence(
                                Commands.parallel(
                                                intakeSubsystem.stopIntakingCommand(),
                                                indexerSubsystem.stopIndexing(),
                                                feederSubsystem.stopFeedingBallsCommand(),
                                                shooterSubsystem.setPercentSpeedCommand(0),
                                                ledSubsystem.setLEDModeCommand(LEDModes.IDLE))));

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