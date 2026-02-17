// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RobotSystemsCheckCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.configurableAutos.AutoCommandDef;
import frc.robot.configurableAutos.AutoParamDef;
import frc.robot.configurableAutos.DynamicAutoRegistry;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIONAVX;
import frc.robot.subsystems.drive.gyro.GyroIOSim;
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
import frc.robot.automation.AutomationTabletInput;
import frc.robot.RobotConstants.PortConstants;
import frc.robot.utils.CowboyUtils;
import frc.robot.utils.QuestCalibration;
import frc.robot.utils.CowboyUtils.RobotModes;
import frc.robot.utils.FuelSim;
import frc.robot.RobotConstants.PortConstants.CAN;
import frc.robot.RobotState.AutoMode;
import frc.robot.automation.AimAlongArcRadiusCommand;
import frc.robot.automation.AutomatedScoring;
import frc.robot.commands.intake.SetIntakePositionCommand;
import frc.robot.commands.shooter.SetShooterSpeedCommand;
import frc.robot.RobotState.IntakePositions;

//@Logged(name = "RobotContainer")
public class RobotContainer {
        public final VisionSubsystem visionSubsystem = new VisionSubsystem();
        public final QuestNavSubsystem questNavSubsystem;
        public final DriveSubsystem driveSubsystem;
        public final IntakeSubsystem intakeSubsystem;
        public final ShooterSubsystem shooterSubsystem;
        public final LEDSubsystem ledSubsystem;

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
                                                                true),
                                                new ModuleIOSpark(RobotConstants.PortConstants.CAN.REAR_LEFT_DRIVING,
                                                                RobotConstants.PortConstants.CAN.REAR_LEFT_TURNING,
                                                                RobotConstants.PortConstants.CAN.REAR_LEFT_CANCODER,
                                                                false),
                                                new ModuleIOSpark(RobotConstants.PortConstants.CAN.REAR_RIGHT_DRIVING,
                                                                RobotConstants.PortConstants.CAN.REAR_RIGHT_TURNING,
                                                                RobotConstants.PortConstants.CAN.REAR_RIGHT_CANCODER,
                                                                true),
                                };
                                driveSubsystem = new DriveSubsystem(moduleIOs, new GyroIONAVX());

                                questNavSubsystem = new QuestNavSubsystem(new QuestNavIOReal());

                                intakeSubsystem = new IntakeSubsystem(new IntakeSubsystemIOSparkMax());

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

                                shooterSubsystem = new ShooterSubsystem(new ShooterSubsystemIO() {

                                });

                                ledSubsystem = new LEDSubsystem(new LEDSubsystemIO() {

                                });

                                break;
                }

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
                } catch (

                Exception e) {
                        e.printStackTrace();
                }
        }

        private void createNamedCommands() {
                // Add commands here to be able to execute in auto

                NamedCommands.registerCommand("Example", new RunCommand(() -> {
                        System.out.println("Running...");
                }));

                dynamicAutoRegistry = new DynamicAutoRegistry();

                dynamicAutoRegistry.registerCommand(new AutoCommandDef("Example Command",
                                List.of(new AutoParamDef("Example Param", 0)), params -> Commands.deferredProxy(
                                                // this is the command factory
                                                () -> AutomatedScoring.exampleCommandDynamicAuton(
                                                                params.get("Example Param")))));

                dynamicAutoRegistry.publishCommands();

        }

        private void configureButtonBindings() {

                driveSubsystem.setDefaultCommand(new TeleopDriveCommand(driveSubsystem, driveJoystick));

                // new JoystickButton(driveJoystick, 6).onTrue(QuestCalibration
                // .CollectCalibrationDataCommand(
                // driveSubsystem::runChassisSpeeds,
                // driveSubsystem::resetOdometry,
                // questNavSubsystem::getUncorrectedPose,
                // driveSubsystem,
                // questNavSubsystem));

                new JoystickButton(driveJoystick, 11).whileTrue(new SetShooterSpeedCommand(1, shooterSubsystem));

                new JoystickButton(driveJoystick, 6)
                                .whileTrue(new AimAlongArcRadiusCommand(driveSubsystem, 2.25, driveJoystick));

                new JoystickButton(driveJoystick, 1).onTrue(RobotState.setCanRotate(true))
                                .onFalse(RobotState.setCanRotate(false));

                new JoystickButton(driveJoystick, 3).onChange(driveSubsystem.xCommand());

                new JoystickButton(driveJoystick, 4)
                                .whileTrue(new SequentialCommandGroup(
                                                Commands.deferredProxy(
                                                                () -> questNavSubsystem.resetPoseYaw(new Rotation2d())),
                                                driveSubsystem.gyroReset()));


                new POVButton(driveJoystick, 180).whileTrue(new SetIntakePositionCommand(intakeSubsystem, IntakePositions.DEPLOYED));
                new POVButton(driveJoystick, 0).whileTrue(new SetIntakePositionCommand(intakeSubsystem, IntakePositions.RETRACTED));

                                                
                // Above = DriveJoystick, Below = OperatorJoystick

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
