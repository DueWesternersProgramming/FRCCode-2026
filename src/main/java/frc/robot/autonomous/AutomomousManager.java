package frc.robot.autonomous;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autonomous.AutonomousQuestionaire.Option;
import frc.robot.commands.automation.AutomatedCommands;
import frc.robot.commands.automation.interpolation.shootSimpleInterpolationCommand;
import frc.robot.commands.automation.misc.BLine;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.JsonUtils;
import frc.robot.lib.BLine.Path;
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

        private List<AutonomousQuestionaire<Supplier<BLinePathSource>>> dynamicAutoChoosers;

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
                                                                () -> new BLinePathSource("NothingPath", false)),
                                                List.of(
                                                                new Option<>("Exit Left Bump",
                                                                                () -> new BLinePathSource(
                                                                                                "ExitLeftBump", false)),
                                                                new Option<>("Exit Right Bump",
                                                                                () -> new BLinePathSource(
                                                                                                "ExitRightBump",
                                                                                                false)))));

                dynamicAutoChoosers.add(
                                new AutonomousQuestionaire<>(
                                                "Autonomous/DynamicAuto/Selector2",
                                                new Option<>("Default",
                                                                () -> new BLinePathSource("NothingPath", false)),
                                                List.of(
                                                                new Option<>("Sweep Neutral Zone (Deep Left)",
                                                                                () -> new BLinePathSource(
                                                                                                "NeutralZoneDeepLeftSweep",
                                                                                                false)),
                                                                new Option<>("Sweep Neutral Zone (Deep Right)",
                                                                                () -> new BLinePathSource(
                                                                                                "NeutralZoneDeepLeftSweep",
                                                                                                true)))));
        }

        private Path combinePaths(List<BLinePathSource> paths) {

                JsonArray combinedElements = new JsonArray();
                JsonArray combinedVelocityConstraints = new JsonArray();

                int elementOffset = 0;

                for (BLinePathSource pathSource : paths) {

                        String jsonText = BLine.getPathJson(pathSource.name());

                        JsonObject json = JsonParser.parseString(jsonText)
                                        .getAsJsonObject();

                        JsonArray elements = json.getAsJsonArray("path_elements");

                        /*
                         * Add path elements
                         * Skip first element of additional paths to avoid duplicate connection point
                         */
                        for (int i = 0; i < elements.size(); i++) {

                                if (elementOffset > 0 && i == 0) {
                                        continue;
                                }

                                combinedElements.add(elements.get(i));
                        }

                        /*
                         * Add velocity constraints while shifting ordinals
                         */
                        if (json.has("constraints")) {

                                JsonObject constraints = json.getAsJsonObject("constraints");

                                if (constraints.has("max_velocity_meters_per_sec")) {

                                        JsonArray velocities = constraints
                                                        .getAsJsonArray("max_velocity_meters_per_sec");

                                        for (JsonElement velocityElement : velocities) {

                                                JsonObject velocity = velocityElement.getAsJsonObject()
                                                                .deepCopy();

                                                int start = velocity.get("start_ordinal").getAsInt();

                                                int end = velocity.get("end_ordinal").getAsInt();

                                                velocity.addProperty(
                                                                "start_ordinal",
                                                                start + elementOffset);

                                                velocity.addProperty(
                                                                "end_ordinal",
                                                                end + elementOffset);

                                                combinedVelocityConstraints.add(velocity);
                                        }
                                }
                        }

                        /*
                         * Increase offset by the number of elements added
                         * (minus one because we removed the duplicate first element)
                         */
                        elementOffset += elements.size();

                        if (elementOffset > 0) {
                                elementOffset--;
                        }
                }

                JsonObject combinedConstraints = new JsonObject();
                combinedConstraints.add(
                                "max_velocity_meters_per_sec",
                                combinedVelocityConstraints);

                JsonObject combined = new JsonObject();

                combined.add(
                                "path_elements",
                                combinedElements);

                combined.add(
                                "constraints",
                                combinedConstraints);

                return JsonUtils.loadPathFromJsonString(
                                combined.toString(),
                                null);
        }

        private Command getDynamicAutoCommand() {

                List<BLinePathSource> selectedPaths = dynamicAutoChoosers.stream()
                                .map(AutonomousQuestionaire::get)
                                .map(Supplier::get)
                                .toList();

                Path combinedPath = combinePaths(selectedPaths);

                return BLine.BLineTrajectory(
                                driveSubsystem,
                                combinedPath,
                                false);
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