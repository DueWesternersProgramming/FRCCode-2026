package frc.robot.configurableAutos;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.RobotConstants.ScoringConstants;
import frc.robot.RobotConstants.ScoringConstants.DynamicAutoScoringPositions;
import frc.robot.commands.automation.AutomatedCommands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.CowboyUtils;

public class DynamicAutoCommands {
    
    public static Command exampleCommandDynamicAuton(Integer exampleParam) {
                return Commands.print("EA Sports: It's in the game! Example Param: " + exampleParam);
        }

    /**
     * A command built to be executed ONLY by the dynamic auto system.
     * @param position Integer value 0 -> 3 inclusive, for left/center/right respectively.
     * @param timeSeconds Integer value to repersent how long to shoot from the hopper for, since we don't have sensors in the hopper yet.
     */
    public static Command DynamicAutoScorePosition(Integer position, Integer timeSeconds, DriveSubsystem driveSubsystem,
            IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, FeederSubsystem feederSubsystem,
            ShooterSubsystem shooterSubsystem) {
        Pose2d goalPose;
        
        goalPose = ScoringConstants.BLUE_ALLIANCE_DYNAMIC_AUTO_SCORING_POSES[position];

        Command cmd = new SequentialCommandGroup(AutomatedCommands.PPmoveToPose(goalPose),AutomatedCommands.shootFromHopperContinousCommand(intakeSubsystem,indexerSubsystem,feederSubsystem,shooterSubsystem,()->CowboyUtils.getAllianceHubPose()));
        
        
        return Commands.defer(() -> cmd, RobotContainer.allSubsystemsSet);
    }
}
