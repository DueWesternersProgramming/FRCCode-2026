package frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autonomous.AutonomousQuestionaire.Option;
import frc.robot.commands.automation.AutomatedCommands;
import frc.robot.commands.automation.interpolation.shootSimpleInterpolationCommand;
import frc.robot.commands.automation.misc.BLine;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.CowboyUtils;

public class AutomomousManager {
    public static enum AutoMode {
        PREDEFINED_AUTO,
        DYNAMIC_AUTO
    }

    public enum BumpPositions {
        LEFT_BUMP,
        RIGHT_BUMP
    }

    public enum NeutralZoneIntakePositions {
        LEFT_SWEEP,
        CENTER_SWEEP,
        RIGHT_SWEEP
    }

    private final AutonomousQuestionaire<Command> predefinedAutoChooser;
    private final AutonomousQuestionaire<AutoMode> autoMode;

    DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    FeederSubsystem feederSubsystem;
    ShooterSubsystem shooterSubsystem;
    LEDSubsystem ledSubsystem;

    public AutomomousManager(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem,
            FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem, LEDSubsystem ledSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.ledSubsystem = ledSubsystem;
        
        this.autoMode = new AutonomousQuestionaire<>(
                "Autonomous/AutoMode",
                new Option<>("Dynamic Auto", AutoMode.DYNAMIC_AUTO),
                List.of(new Option<>("Predefined Auto", AutoMode.PREDEFINED_AUTO)));

        this.predefinedAutoChooser = new AutonomousQuestionaire<>(
                "Autonomous/PredefinedAutoChooser",
                new Option<>("DEFAULT", Commands.print("Default Do Nothing Auto")),
                List.of());

        registerTriggerCommands();
        addPredefinedAutoOptions();
    }

    public Command getAutonomousCommand() {
        AutoMode selectedAutoMode = autoMode.get();
        if (selectedAutoMode == AutoMode.PREDEFINED_AUTO) {
            return predefinedAutoChooser.get();
        } else {
            return Commands.print("Dynamic Auto Not Implemented");
        }
    }

    private void registerTriggerCommands(){
        FollowPath.registerEventTrigger("SimpleShoot", new shootSimpleInterpolationCommand(shooterSubsystem, () -> CowboyUtils.getAllianceHubPose()));
        FollowPath.registerEventTrigger("RunIntake", AutomatedCommands.intakeCommand(intakeSubsystem, ledSubsystem));
    }

    private void addPredefinedAutoOptions() {
        this.predefinedAutoChooser.addOption(new Option<>("LeftSideOneSweep", BLine.BLineTrajectory(driveSubsystem, "LeftSideOneSweep", false)));
        this.predefinedAutoChooser.addOption(new Option<>("RightSideOneSweep", BLine.BLineTrajectory(driveSubsystem, "LeftSideOneSweep", true)));
    }

    public Command exitViaBumpCommand(BumpPositions bumpPosition) {
        return BLine.BLineTrajectory(driveSubsystem, "ExitLeftBump", (bumpPosition == BumpPositions.RIGHT_BUMP));
    }

    public Command neutralZoneIntakeCommand(NeutralZoneIntakePositions intakePosition) {
        switch (intakePosition) {
            case LEFT_SWEEP:
                return BLine.BLineTrajectory(driveSubsystem, "NeutralZoneIntakeLeft", false);
            case CENTER_SWEEP:
                return BLine.BLineTrajectory(driveSubsystem, "NeutralZoneIntakeCenter", false);
            case RIGHT_SWEEP:
                return BLine.BLineTrajectory(driveSubsystem, "NeutralZoneIntakeLeft", true);
            default:
                return BLine.BLineTrajectory(driveSubsystem, "NeutralZoneIntakeCenter", false);
        }
    }

}
