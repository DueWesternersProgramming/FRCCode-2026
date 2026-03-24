// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RobotSystemsCheckCommand;
import frc.robot.commands.automation.AimAlongArcRadiusCommand;
import frc.robot.commands.automation.AutomatedScoring;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.configurableAutos.AutoCommandDef;
import frc.robot.configurableAutos.AutoParamDef;
import frc.robot.configurableAutos.DynamicAutoCommands;
import frc.robot.configurableAutos.DynamicAutoRegistry;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIONAVX;
import frc.robot.subsystems.drive.gyro.GyroIOSim;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystemIO;
import frc.robot.subsystems.feeder.FeederSubsystemIOSim;
import frc.robot.subsystems.feeder.FeederSubsystemIOSparkMax;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystemIO;
import frc.robot.subsystems.indexer.IndexerSubsystemIOSim;
import frc.robot.subsystems.indexer.IndexerSubsystemIOSparkMax;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystemIO;
import frc.robot.subsystems.intake.IntakeSubsystemIOSim;
import frc.robot.subsystems.intake.IntakeSubsystemIOSparkMax;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.led.LEDSubsystemIO;
import frc.robot.subsystems.led.LEDSubsystemIOCandle;
import frc.robot.subsystems.led.LEDSubsystemIOSim;
import frc.robot.subsystems.questnav.QuestNavIO;
import frc.robot.subsystems.questnav.QuestNavIOReal;
import frc.robot.subsystems.questnav.QuestNavSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystemIO;
import frc.robot.subsystems.shooter.ShooterSubsystemIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystemIOSparkMax;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.RobotConstants.PortConstants;
import frc.robot.utils.CowboyUtils;
import frc.robot.utils.QuestCalibration;
import frc.robot.utils.CowboyUtils.RobotModes;
import frc.robot.utils.FuelSim;
import frc.robot.RobotConstants.PortConstants.CAN;
import frc.robot.RobotState.AutoMode;

//@Logged(name = "RobotContainer")
public class RobotContainer {
        public final VisionSubsystem visionSubsystem = new VisionSubsystem();
        public final QuestNavSubsystem questNavSubsystem;
        public final DriveSubsystem driveSubsystem;
        public final IntakeSubsystem intakeSubsystem;
        public final IndexerSubsystem indexerSubsystem;
        public final FeederSubsystem feederSubsystem;
        public final ShooterSubsystem shooterSubsystem;
        public final LEDSubsystem ledSubsystem;

        public static Set<Subsystem> allSubsystemsSet = new HashSet<>();
        public static Set<Subsystem> superStructureSet = new HashSet<>();

        private final Joystick driveJoystick = new Joystick(RobotConstants.PortConstants.Controller.DRIVE_JOYSTICK);
        private final Joystick operatorJoystick = new Joystick(
                        RobotConstants.PortConstants.Controller.OPERATOR_JOYSTICK);

        ModuleIO[] moduleIOs;

        SendableChooser<Command> autoPPChooser = new SendableChooser<>();
        SendableChooser<AutoMode> autoMode = new SendableChooser<>();

        DynamicAutoRegistry dynamicAutoRegistry;

        PowerDistribution pdp;

        private final Field2d field = new Field2d();

        public RobotContainer() {
                System.out.println("Robot Mode: " + CowboyUtils.RobotModes.currentMode);

                switch (RobotModes.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations

                                moduleIOs = new ModuleIO[] {
                                                new ModuleIOSpark(RobotConstants.PortConstants.CAN.FRONT_LEFT_DRIVING,
                                                                RobotConstants.PortConstants.CAN.FRONT_LEFT_TURNING,
                                                                RobotConstants.PortConstants.CAN.FRONT_LEFT_CANCODER,
                                                                false),
                                                new ModuleIOSpark(RobotConstants.PortConstants.CAN.FRONT_RIGHT_DRIVING,
                                                                RobotConstants.PortConstants.CAN.FRONT_RIGHT_TURNING,
                                                                RobotConstants.PortConstants.CAN.FRONT_RIGHT_CANCODER,
                                                                false),
                                                new ModuleIOSpark(RobotConstants.PortConstants.CAN.REAR_LEFT_DRIVING,
                                                                RobotConstants.PortConstants.CAN.REAR_LEFT_TURNING,
                                                                RobotConstants.PortConstants.CAN.REAR_LEFT_CANCODER,
                                                                true),
                                                new ModuleIOSpark(RobotConstants.PortConstants.CAN.REAR_RIGHT_DRIVING,
                                                                RobotConstants.PortConstants.CAN.REAR_RIGHT_TURNING,
                                                                RobotConstants.PortConstants.CAN.REAR_RIGHT_CANCODER,
                                                                false),
                                };
                                driveSubsystem = new DriveSubsystem(moduleIOs, new GyroIONAVX());

                                questNavSubsystem = new QuestNavSubsystem(new QuestNavIOReal());

                                intakeSubsystem = new IntakeSubsystem(new IntakeSubsystemIOSparkMax());

                                indexerSubsystem = new IndexerSubsystem(new IndexerSubsystemIOSparkMax());

                                feederSubsystem = new FeederSubsystem(new FeederSubsystemIOSparkMax());

                                shooterSubsystem = new ShooterSubsystem(new ShooterSubsystemIOSparkMax());

                                ledSubsystem = new LEDSubsystem(new LEDSubsystemIOCandle());

                                break;

                        case SIM:
                                moduleIOs = new ModuleIO[] {
                                                new ModuleIOSim(),
                                                new ModuleIOSim(),
                                                new ModuleIOSim(),
                                                new ModuleIOSim(),
                                };

                                driveSubsystem = new DriveSubsystem(moduleIOs, new GyroIOSim());

                                questNavSubsystem = new QuestNavSubsystem(new QuestNavIOReal());

                                intakeSubsystem = new IntakeSubsystem(new IntakeSubsystemIOSim());

                                indexerSubsystem = new IndexerSubsystem(new IndexerSubsystemIOSim());

                                feederSubsystem = new FeederSubsystem(new FeederSubsystemIOSim());

                                shooterSubsystem = new ShooterSubsystem(new ShooterSubsystemIOSim());

                                ledSubsystem = new LEDSubsystem(new LEDSubsystemIOSim());

                                break;

                        default:

                                moduleIOs = new ModuleIO[] {
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                };
                                driveSubsystem = new DriveSubsystem(moduleIOs, new GyroIO() {
                                });

                                questNavSubsystem = new QuestNavSubsystem(new QuestNavIO() {
                                });

                                intakeSubsystem = new IntakeSubsystem(new IntakeSubsystemIO() {

                                });

                                indexerSubsystem = new IndexerSubsystem(new IndexerSubsystemIO() {
                                });

                                feederSubsystem = new FeederSubsystem(new FeederSubsystemIO() {
                                });

                                shooterSubsystem = new ShooterSubsystem(new ShooterSubsystemIO() {

                                });

                                ledSubsystem = new LEDSubsystem(new LEDSubsystemIO() {

                                });

                                break;
                }
                allSubsystemsSet.add(driveSubsystem);
                allSubsystemsSet.add(intakeSubsystem);
                allSubsystemsSet.add(indexerSubsystem);
                allSubsystemsSet.add(feederSubsystem);
                allSubsystemsSet.add(shooterSubsystem);

                superStructureSet.add(intakeSubsystem);
                superStructureSet.add(indexerSubsystem);
                superStructureSet.add(feederSubsystem);
                superStructureSet.add(shooterSubsystem);

                createNamedCommands();

                configureButtonBindings();

                try {
                        pdp = new PowerDistribution(CAN.PDH, ModuleType.kRev);

                        autoPPChooser = AutoBuilder.buildAutoChooser("Test Auto");

                        Shuffleboard.getTab("Autonomous Selection").add(autoPPChooser);

                        autoMode.addOption("Pathplanner", AutoMode.PP_AUTO);
                        autoMode.setDefaultOption("Dynamic", AutoMode.DYNAMIC_AUTO);

                        Shuffleboard.getTab("Autonomous Selection").add("PathPlannerAutoSelector", autoPPChooser);
                        Shuffleboard.getTab("Autonomous Selection").add("AutoModeSelector", autoMode);

                        Shuffleboard.getTab("Power").add(pdp);

                        configureFuelSim();

                        DriverStation.silenceJoystickConnectionWarning(true);
                } catch (

                Exception e) {
                        e.printStackTrace();
                }
        }

        private void createNamedCommands() {
                // Add commands here to be able to execute in auto

                NamedCommands.registerCommand("Start Intake", intakeSubsystem.setIntakeSpeedCommand(.8));

                // These two commands never end, so we have to use a time based race condition.
                NamedCommands.registerCommand("Interpolate Score",
                                Commands.defer(()->AutomatedScoring.shootFromHopperContinousCommand(intakeSubsystem, indexerSubsystem,
                                                feederSubsystem, shooterSubsystem, ()->CowboyUtils.getAllianceHubPose()), allSubsystemsSet));
                NamedCommands.registerCommand("Interpolate Pass",
                                Commands.defer(()->AutomatedScoring.shootFromHopperContinousCommand(intakeSubsystem, indexerSubsystem,
                                                feederSubsystem, shooterSubsystem, ()->CowboyUtils.getAllianceFeedingPosition()), allSubsystemsSet));

                NamedCommands.registerCommand("Stop All Superstructure", AutomatedScoring.stopAllSuperStructure(
                                intakeSubsystem, indexerSubsystem, feederSubsystem, shooterSubsystem));

                NamedCommands.registerCommand("Example", new RunCommand(() -> {
                        System.out.println("Running...");
                }));


                
                dynamicAutoRegistry = new DynamicAutoRegistry();

                dynamicAutoRegistry.registerCommand(new AutoCommandDef("Example Command",
                                List.of(new AutoParamDef("Example Param", 0)), params -> Commands.deferredProxy(
                                                // this is the command factory
                                                () -> DynamicAutoCommands.exampleCommandDynamicAuton(
                                                                params.get("Example Param")))));

                dynamicAutoRegistry.registerCommand(new AutoCommandDef("Score From Position",
                                List.of(new AutoParamDef("Position", 1),new AutoParamDef("Time", 5)), params -> Commands.deferredProxy(
                                                // this is the command factory
                                                () -> DynamicAutoCommands.DynamicAutoScorePosition(
                                                                params.get("Position"),
                                                                params.get("Time"),
                                                                driveSubsystem,
                                                                intakeSubsystem,
                                                                indexerSubsystem,
                                                                feederSubsystem,
                                                                shooterSubsystem))));

                dynamicAutoRegistry.publishCommands();
        }

        private void configureButtonBindings() {

                driveSubsystem.setDefaultCommand(new TeleopDriveCommand(driveSubsystem, driveJoystick));

                // QuestNav Offset Calibration:

                // new JoystickButton(driveJoystick, 6).onTrue(QuestCalibration
                // .CollectCalibrationDataCommand(
                // driveSubsystem::runChassisSpeeds,
                // driveSubsystem::resetOdometry,
                // questNavSubsystem::getUncorrectedPose,
                // driveSubsystem,
                // questNavSubsystem));

                if (!CowboyUtils.isSim()) { // Real robot

                        // Right operator trigger, enables shoot on the move and turrets the robot.
                        new Trigger(() -> operatorJoystick.getRawAxis(3) > .4)
                                        .whileTrue(AutomatedScoring.teleopShootOnMoveAutomationCommand(
                                                        driveSubsystem, driveJoystick, intakeSubsystem,
                                                        indexerSubsystem, feederSubsystem, shooterSubsystem))
                                        .onFalse(AutomatedScoring.stopAllSuperStructure(intakeSubsystem,
                                                        indexerSubsystem, feederSubsystem, shooterSubsystem));

                        // Left operator trigger, runs intake while held.
                        new Trigger(() -> operatorJoystick.getRawAxis(2) > .4)
                                        .whileTrue(new SequentialCommandGroup(
                                                        intakeSubsystem.setIntakeSpeedCommand(-.3), new WaitCommand(.1),
                                                        intakeSubsystem.setIntakeSpeedCommand(1)))
                                        .onFalse(intakeSubsystem.stopIntakingCommand());

                        // Operator X button, reverses indexer if needed to clear jams
                        new JoystickButton(operatorJoystick, 3)
                                        .whileTrue(indexerSubsystem.setIndexerSpeedCommand(-.3, -1)).whileTrue(intakeSubsystem.setIntakeSpeedCommand(-.6))
                                        .onFalse(indexerSubsystem.stopIndexing()).onFalse(intakeSubsystem.stopIntakingCommand()).whileTrue(feederSubsystem.setFeederSpeed(-.5)).onFalse(feederSubsystem.setFeederSpeed(0));

                        // new JoystickButton(driveJoystick, 11)
                        // .whileTrue(new ParallelCommandGroup(new
                        // AimAlongArcRadiusCommand(driveSubsystem, 3.5, driveJoystick),
                        // AutomatedScoring.shootFromHopperContinousCommand(intakeSubsystem,indexerSubsystem,feederSubsystem,
                        // shooterSubsystem,.8))).onFalse(AutomatedScoring.stopAllSuperStructure(intakeSubsystem,indexerSubsystem,feederSubsystem,shooterSubsystem));

                        new JoystickButton(driveJoystick, 1).onTrue(RobotState.setCanRotate(true))
                                        .onFalse(RobotState.setCanRotate(false));

                        new JoystickButton(driveJoystick, 6)
                                        .whileTrue(new SequentialCommandGroup(
                                                        Commands.deferredProxy(
                                                                        () -> questNavSubsystem.resetPoseYaw(
                                                                                        new Rotation2d())),
                                                        driveSubsystem.gyroReset()));

                        new JoystickButton(driveJoystick, 7).onTrue(driveSubsystem.resetEncodersCommand());
                }

                else { // Sim, just the one Logitech F310 controller for testing

                        new Trigger(() -> driveJoystick.getRawAxis(3) > .4)
                                        .whileTrue(AutomatedScoring.teleopShootOnMoveAutomationCommand(
                                                        driveSubsystem, driveJoystick, intakeSubsystem,
                                                        indexerSubsystem, feederSubsystem, shooterSubsystem))
                                        .onFalse(AutomatedScoring.stopAllSuperStructure(intakeSubsystem,
                                                        indexerSubsystem, feederSubsystem, shooterSubsystem));

                        new Trigger(() -> driveJoystick.getRawAxis(2) > .4)
                                        .whileTrue(new SequentialCommandGroup(
                                                        intakeSubsystem.setIntakeSpeedCommand(-.3), new WaitCommand(.1),
                                                        intakeSubsystem.setIntakeSpeedCommand(.8)))
                                        .onFalse(intakeSubsystem.stopIntakingCommand());

                        new JoystickButton(operatorJoystick, 6)
                                        .whileTrue(indexerSubsystem.setIndexerSpeedCommand(-.3, -1))
                                        .onFalse(indexerSubsystem.stopIndexing());

                        new JoystickButton(driveJoystick, 8)
                                        .whileTrue(new SequentialCommandGroup(
                                                        Commands.deferredProxy(
                                                                        () -> questNavSubsystem.resetPoseYaw(
                                                                                        new Rotation2d())),
                                                        driveSubsystem.gyroReset()));
                }
        }

        public Command getPPAutonomousCommand() {
                if (autoPPChooser.getSelected() != null) {
                        return autoPPChooser.getSelected();
                } else {
                        return driveSubsystem.gyroReset();
                }
        }

        public AutoMode getSelectedAutoMode() {
                AutoMode selectedAutoMode = autoMode.getSelected();
                Logger.recordOutput("RobotState/Selected Auto Mode", selectedAutoMode);

                return selectedAutoMode;
        }

        public Command getTestingCommand() {
                return new RobotSystemsCheckCommand(driveSubsystem);
        }

        public Field2d getField() {
                return field;
        }

        private void configureFuelSim() {
                FuelSim instance = FuelSim.getInstance();
                instance.spawnStartingFuel();

                instance.registerRobot(
                                Units.inchesToMeters(33),
                                Units.inchesToMeters(33),
                                Units.inchesToMeters(6),
                                driveSubsystem::getPose,
                                driveSubsystem::getChassisSpeeds);
                // instance.registerIntake(
                // -Dimensions.FULL_LENGTH.div(2).in(Meters),
                // Dimensions.FULL_LENGTH.div(2).in(Meters),
                // -Dimensions.FULL_WIDTH.div(2).plus(Inches.of(7)).in(Meters),
                // -Dimensions.FULL_WIDTH.div(2).in(Meters),
                // () -> intake.isRightDeployed() && turret.simAbleToIntake(),
                // turret::simIntake);

                instance.start();
                SmartDashboard.putData(Commands.runOnce(() -> {
                        FuelSim.getInstance().clearFuel();
                        FuelSim.getInstance().spawnStartingFuel();
                })
                                .withName("Reset Fuel")
                                .ignoringDisable(true));
        }

}
