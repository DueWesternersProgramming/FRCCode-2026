package frc.robot.autonomous;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

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
        DEEP_LEFT_SWEEP,
        DEEP_RIGHT_SWEEP,
        DEEP_CENTER_SWEEP,
        SHALLOW_LEFT_SWEEP,
        SHALLOW_RIGHT_SWEEP,
        SHALLOW_CENTER_SWEEP
    }

    private final AutonomousQuestionaire<Supplier<Command>> predefinedAutoChooser;
    private final AutonomousQuestionaire<AutoMode> autoMode;

    private List<AutonomousQuestionaire<Supplier<Command>>> dynamicAutoChoosers;

    DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    FeederSubsystem feederSubsystem;
    ShooterSubsystem shooterSubsystem;
    LEDSubsystem ledSubsystem;

    public AutomomousManager(
            DriveSubsystem driveSubsystem,
            IntakeSubsystem intakeSubsystem,
            FeederSubsystem feederSubsystem,
            ShooterSubsystem shooterSubsystem,
            LEDSubsystem ledSubsystem) {

        this.driveSubsystem = driveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.ledSubsystem = ledSubsystem;

        this.autoMode = new AutonomousQuestionaire<>(
                "Autonomous/Autoselector/AutoMode",
                new Option<>("Dynamic Auto", AutoMode.DYNAMIC_AUTO),
                List.of(
                        new Option<>("Predefined Auto", AutoMode.PREDEFINED_AUTO)));

        this.predefinedAutoChooser = new AutonomousQuestionaire<>(
                "Autonomous/Autoselector/PredefinedAutoChooser",
                new Option<>("DEFAULT", () -> Commands.print("Default Do Nothing Auto")),
                List.of());

        registerTriggerCommands();
        addPredefinedAutoOptions();
        publishDynamicAutoOptions();
    }

    public Command getAutonomousCommand() {
        AutoMode selectedAutoMode = autoMode.get();

        if (selectedAutoMode == AutoMode.PREDEFINED_AUTO) {
            return predefinedAutoChooser.get().get();
        } else {
            return getDynamicAutoCommand();
        }
    }

    private void publishDynamicAutoOptions() {

        dynamicAutoChoosers = new ArrayList<>();

        dynamicAutoChoosers.add(
                new AutonomousQuestionaire<>(
                        "Autonomous/DynamicAuto/Selector1",
                        new Option<>("Default",
                                () -> Commands.print("Default Do Nothing")),
                        List.of(
                                new Option<>("Exit Left Bump",
                                        () -> exitViaBumpCommand(BumpPositions.LEFT_BUMP)),
                                new Option<>("Exit Right Bump",
                                        () -> exitViaBumpCommand(BumpPositions.RIGHT_BUMP)))));

        dynamicAutoChoosers.add(
                new AutonomousQuestionaire<>(
                        "Autonomous/DynamicAuto/Selector2",
                        new Option<>("Default",
                                () -> Commands.print("Default Do Nothing")),
                        List.of(
                                new Option<>("Sweep Neutral Zone (Deep Left)",
                                        () -> neutralZoneIntakeCommand(
                                                NeutralZoneIntakePositions.DEEP_LEFT_SWEEP)),
                                new Option<>("Sweep Neutral Zone (Deep Right)",
                                        () -> neutralZoneIntakeCommand(
                                                NeutralZoneIntakePositions.DEEP_RIGHT_SWEEP)))));
    }

    private Command getDynamicAutoCommand() {
        return Commands.sequence(
                dynamicAutoChoosers.stream()
                        .map(AutonomousQuestionaire::get)
                        .map(Supplier::get)
                        .toArray(Command[]::new));
    }

    private void registerTriggerCommands() {
        FollowPath.registerEventTrigger("SimpleShoot",
                new shootSimpleInterpolationCommand(
                        shooterSubsystem,
                        () -> CowboyUtils.getAllianceHubPose()));

        FollowPath.registerEventTrigger("RunIntake",
                AutomatedCommands.intakeCommand(
                        intakeSubsystem,
                        ledSubsystem));
    }

    private void addPredefinedAutoOptions() {
        this.predefinedAutoChooser.addOption(
                new Option<>("LeftSideOneSweep",
                        () -> BLine.BLineTrajectory(
                                driveSubsystem,
                                "LeftSideOneSweep",
                                false)));

        this.predefinedAutoChooser.addOption(
                new Option<>("RightSideOneSweep",
                        () -> BLine.BLineTrajectory(
                                driveSubsystem,
                                "LeftSideOneSweep",
                                true)));
    }

    public Command exitViaBumpCommand(BumpPositions bumpPosition) {
        return BLine.BLineTrajectory(
                driveSubsystem,
                "ExitLeftBump",
                (bumpPosition == BumpPositions.RIGHT_BUMP));
    }

    public Command neutralZoneIntakeCommand(
            NeutralZoneIntakePositions intakePosition) {

        switch (intakePosition) {
            case DEEP_LEFT_SWEEP:
                return BLine.BLineTrajectory(
                        driveSubsystem,
                        "NeutralZoneDeepLeftSweep",
                        false);

            case DEEP_CENTER_SWEEP:
                return BLine.BLineTrajectory(
                        driveSubsystem,
                        "",
                        false);

            case DEEP_RIGHT_SWEEP:
                return BLine.BLineTrajectory(
                        driveSubsystem,
                        "NeutralZoneDeepLeftSweep",
                        true);

            case SHALLOW_LEFT_SWEEP:
                return BLine.BLineTrajectory(
                        driveSubsystem,
                        "NeutralZoneShallowLeftSweep",
                        false);

            case SHALLOW_CENTER_SWEEP:
                return BLine.BLineTrajectory(
                        driveSubsystem,
                        "",
                        false);

            case SHALLOW_RIGHT_SWEEP:
                return BLine.BLineTrajectory(
                        driveSubsystem,
                        "NeutralZoneShallowLeftSweep",
                        true);

            default:
                return BLine.BLineTrajectory(
                        driveSubsystem,
                        "NeutralZoneIntakeCenter",
                        false);
        }
    }
}