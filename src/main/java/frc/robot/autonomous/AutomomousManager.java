package frc.robot.autonomous;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.HighLevelCommands;
import frc.robot.commands.automation.interpolation.shootSimpleInterpolationCommand;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.JsonUtils;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.LoggedChooser;
import frc.robot.utils.CowboyUtils;
import frc.robot.utils.LoggedChooser.Option;

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

        private final LoggedChooser<Supplier<Command>> predefinedAutoChooser;
        private final LoggedChooser<AutoMode> autoMode;

        private List<LoggedChooser<Supplier<Object>>> dynamicAutoChoosers;

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

                this.autoMode = new LoggedChooser<>(
                                "Autonomous/Autoselector/AutoMode",
                                new Option<>("Dynamic Auto", AutoMode.DYNAMIC_AUTO),
                                List.of(
                                                new Option<>("Predefined Auto", AutoMode.PREDEFINED_AUTO)));

                this.predefinedAutoChooser = new LoggedChooser<>(
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
                        try {
                                return predefinedAutoChooser.get().get();
                        } catch (Exception e) {
                                DriverStation.reportError("Predefined Auto Error: " + e, true);
                                return Commands.none();
                        }
                } else {
                        try {
                        } catch (Exception e) {
                                DriverStation.reportError("Dynamic Auto Error: " + e, true);
                                return Commands.none();
                        }
                        return getDynamicAutoCommand();
                }
        }

        private void publishDynamicAutoOptions() {

                dynamicAutoChoosers = new ArrayList<>();

                dynamicAutoChoosers.add(
                                new LoggedChooser<>(
                                                "Autonomous/DynamicAuto/ExitZoneAction",
                                                new Option<>("No Exit",
                                                                () -> new BLinePathSource("NothingPath", false)),
                                                List.of(
                                                                new Option<>("Exit Left Bump",
                                                                                () -> new BLinePathSource(
                                                                                                "ExitLeftBump", false)),
                                                                new Option<>("Exit Right Bump",
                                                                                () -> new BLinePathSource(
                                                                                                "ExitLeftBump",
                                                                                                true)))));

                dynamicAutoChoosers.add(
                                new LoggedChooser<>(
                                                "Autonomous/DynamicAuto/IntakeAction",
                                                new Option<>("No Intake",
                                                                () -> new BLinePathSource("NothingPath", false)),
                                                List.of(
                                                                new Option<>("Sweep Neutral Zone (Deep Left, 1/2)",
                                                                                () -> new BLinePathSource(
                                                                                                "NeutralZoneDeepLeftSweepHalf",
                                                                                                false)),
                                                                new Option<>("Sweep Neutral Zone (Deep Right, 1/2)",
                                                                                () -> new BLinePathSource(
                                                                                                "NeutralZoneDeepLeftSweepHalf",
                                                                                                true)),

                                                                new Option<>("Sweep Neutral Zone (Deep Left, 1/4)",
                                                                                () -> new BLinePathSource(
                                                                                                "NeutralZoneDeepLeftSweepQuarter",
                                                                                                false)),
                                                                new Option<>("Sweep Neutral Zone (Deep Right, 1/4)",
                                                                                () -> new BLinePathSource(
                                                                                                "NeutralZoneDeepLeftSweepQuarter",
                                                                                                true)),

                                                                new Option<>("Sweep Neutral Zone (Shallow Left)",
                                                                                () -> new BLinePathSource(
                                                                                                "NeutralZoneShallowLeftSweep",
                                                                                                false)),
                                                                new Option<>("Sweep Neutral Zone (Shallow Right)",
                                                                                () -> new BLinePathSource(
                                                                                                "NeutralZoneShallowLeftSweep",
                                                                                                true)),
                                                                new Option<>("Custom Ball Seeking Command",
                                                                                () -> Commands.print(
                                                                                                "I AM LOOKING FOR BALLS!!")))));
        }

        private Path combinePaths(List<BLinePathSource> paths) {

                JsonArray combinedElements = new JsonArray();
                JsonArray combinedVelocityConstraints = new JsonArray();

                int elementOffset = 0;

                for (BLinePathSource pathSource : paths) {

                        String jsonText = "";
                        try {
                                jsonText = BLine.getPathJson(pathSource.name());
                        } catch (IOException e) {
                                // TODO Auto-generated catch block
                                e.printStackTrace();
                        }

                        JsonObject json = JsonParser.parseString(jsonText)
                                        .getAsJsonObject();

                        if (pathSource.mirrorVertically()) {
                                mirrorJsonPath(json);
                        }

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

        private static void mirrorWaypoint(JsonObject waypoint) {
                JsonObject translationTarget = waypoint.getAsJsonObject("translation_target");

                Translation2d mirroredTranslation = FlippingUtil.mirrorFieldPosition(
                                new Translation2d(
                                                translationTarget.get("x_meters").getAsDouble(),
                                                translationTarget.get("y_meters").getAsDouble()));

                translationTarget.addProperty("x_meters", mirroredTranslation.getX());
                translationTarget.addProperty("y_meters", mirroredTranslation.getY());

                JsonObject rotationTarget = waypoint.getAsJsonObject("rotation_target");

                Rotation2d mirroredRotation = FlippingUtil.mirrorFieldRotation(
                                Rotation2d.fromRadians(
                                                rotationTarget.get("rotation_radians").getAsDouble()));

                rotationTarget.addProperty(
                                "rotation_radians",
                                mirroredRotation.getRadians());
        }

        private static void mirrorTranslation(JsonObject translation) {

                Translation2d mirrored = FlippingUtil.mirrorFieldPosition(
                                new Translation2d(
                                                translation.get("x_meters").getAsDouble(),
                                                translation.get("y_meters").getAsDouble()));

                translation.addProperty("x_meters", mirrored.getX());
                translation.addProperty("y_meters", mirrored.getY());
        }

        private static void mirrorRotation(JsonObject rotation) {

                Rotation2d mirrored = FlippingUtil.mirrorFieldRotation(
                                Rotation2d.fromRadians(
                                                rotation.get("rotation_radians").getAsDouble()));

                rotation.addProperty(
                                "rotation_radians",
                                mirrored.getRadians());
        }

        private static void mirrorJsonPath(JsonObject json) {

                JsonArray elements = json.getAsJsonArray("path_elements");

                for (JsonElement element : elements) {

                        JsonObject obj = element.getAsJsonObject();

                        switch (obj.get("type").getAsString()) {

                                case "waypoint":
                                        mirrorWaypoint(obj);
                                        break;

                                case "translation":
                                        mirrorTranslation(obj);
                                        break;

                                case "rotation":
                                        mirrorRotation(obj);
                                        break;

                                default:
                                        break;
                        }
                }
        }

        private Command getDynamicAutoCommand() {

                List<Command> finalCommands = new ArrayList<>();

                List<BLinePathSource> tempCombinedPaths = new ArrayList<>();

                List<Object> selections = dynamicAutoChoosers.stream()
                                .map(LoggedChooser::get)
                                .map(Supplier::get)
                                .toList();

                for (Object item : selections) {
                        if (item instanceof Command command) {
                                finalCommands.add(BLine.BLineTrajectory(
                                                driveSubsystem,
                                                combinePaths(tempCombinedPaths),
                                                false));

                                tempCombinedPaths.clear();

                                finalCommands.add(command);

                        } else if (item instanceof BLinePathSource path) {
                                tempCombinedPaths.add(path);
                        }
                }

                if (tempCombinedPaths.size() > 0) {
                        finalCommands.add(BLine.BLineTrajectory(
                                        driveSubsystem,
                                        combinePaths(tempCombinedPaths),
                                        false));
                }

                return Commands.sequence(finalCommands.stream().toArray(Command[]::new));

        }

        private void registerTriggerCommands() {
                FollowPath.registerEventTrigger("SimpleShoot",
                                HighLevelCommands.shootFromHopperContinousCommand(intakeSubsystem, feederSubsystem, shooterSubsystem, ()->CowboyUtils.getAllianceHubPose()));

                FollowPath.registerEventTrigger("RunIntake",
                                HighLevelCommands.intakeCommand(
                                                intakeSubsystem,
                                                feederSubsystem,
                                                ledSubsystem));
        }

        private void addPredefinedAutoOptions() {
                this.predefinedAutoChooser.addOption(
                                new Option<>("leftsideonesweep",
                                                () -> BLine.BLineTrajectory(
                                                                driveSubsystem,
                                                                "leftsideonesweep",
                                                                false)));

                this.predefinedAutoChooser.addOption(
                                new Option<>("rightsideonesweep",
                                                () -> BLine.BLineTrajectory(
                                                                driveSubsystem,
                                                                "leftsideonesweep",
                                                                true)));
                this.predefinedAutoChooser.addOption(
                                new Option<>("5ft Test",
                                                () -> BLine.BLineTrajectory(
                                                                driveSubsystem,
                                                                "5ft-test",
                                                                false)));
        }
}