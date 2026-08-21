// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashSet;
import java.util.Set;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.HighLevelCommands;
import frc.robot.commands.RobotSystemsCheckCommand;
import frc.robot.commands.drive.TeleopDriveCommand;

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
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystemIO;
import frc.robot.subsystems.intake.IntakeSubsystemIOSim;
import frc.robot.subsystems.intake.IntakeSubsystemIOSparkMax;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.led.LEDSubsystemIO;
import frc.robot.subsystems.led.LEDSubsystemIOCandle;
import frc.robot.subsystems.led.LEDSubsystemIOSim;
import frc.robot.subsystems.questnav.QuestNavSubsystemIO;
import frc.robot.subsystems.questnav.QuestNavSubsystemIOReal;
import frc.robot.subsystems.questnav.QuestNavSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystemIO;
import frc.robot.subsystems.shooter.ShooterSubsystemIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystemIOSparkMax;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.CowboyUtils;
import frc.robot.utils.CowboyUtils.RobotModes;
import frc.robot.utils.FuelSim;
import frc.robot.RobotConstants.PortConstants.CAN;
import frc.robot.autonomous.AutomomousManager;

public class RobotContainer {
        public final VisionSubsystem visionSubsystem = new VisionSubsystem();
        public final QuestNavSubsystem questNavSubsystem;
        public final DriveSubsystem driveSubsystem;
        public final IntakeSubsystem intakeSubsystem;
        public final FeederSubsystem feederSubsystem;
        public final ShooterSubsystem shooterSubsystem;
        public final LEDSubsystem ledSubsystem;

        public static Set<Subsystem> allSubsystemsSet = new HashSet<>();
        public static Set<Subsystem> superStructureSet = new HashSet<>();

        private final Joystick driveController = new Joystick(RobotConstants.PortConstants.Controller.DRIVE_CONTROLLER);

        private final Joystick operatorController = new Joystick(
                        RobotConstants.PortConstants.Controller.OPERATOR_CONTROLLER);

        ModuleIO[] moduleIOs;

        AutomomousManager automomousManager;

        PowerDistribution pdp;

        private final Field2d field = new Field2d();

        public RobotContainer() {
                System.out.println("Robot Mode: " + CowboyUtils.RobotModes.currentMode);

                switch (RobotModes.currentMode) {
                        case REAL:
                                new Tuning();
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

                                questNavSubsystem = new QuestNavSubsystem(new QuestNavSubsystemIOReal());

                                intakeSubsystem = new IntakeSubsystem(new IntakeSubsystemIOSparkMax());

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

                                questNavSubsystem = new QuestNavSubsystem(new QuestNavSubsystemIOReal());

                                intakeSubsystem = new IntakeSubsystem(new IntakeSubsystemIOSim());

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

                                questNavSubsystem = new QuestNavSubsystem(new QuestNavSubsystemIO() {
                                });

                                intakeSubsystem = new IntakeSubsystem(new IntakeSubsystemIO() {

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
                allSubsystemsSet.add(feederSubsystem);
                allSubsystemsSet.add(shooterSubsystem);

                superStructureSet.add(intakeSubsystem);
                superStructureSet.add(feederSubsystem);
                superStructureSet.add(shooterSubsystem);

                automomousManager = new AutomomousManager(driveSubsystem, intakeSubsystem, feederSubsystem,
                                shooterSubsystem, ledSubsystem);

                configureButtonBindings();

                try {
                        pdp = new PowerDistribution(CAN.PDH, ModuleType.kRev);

                        DriverStation.silenceJoystickConnectionWarning(true);
                } catch (

                Exception e) {
                        e.printStackTrace();
                }
        }

        private void configureButtonBindings() {

                driveSubsystem.setDefaultCommand(new TeleopDriveCommand(driveSubsystem, driveController)); // Same for
                                                                                                           // both sim
                                                                                                           // and
                                                                                                           // real. The
                                                                                                           // joystick
                                                                                                           // axis
                                                                                                           // constants
                                                                                                           // change
                                                                                                           // depending
                                                                                                           // on
                                                                                                           // mode.
                //feederSubsystem.setDefaultCommand(feederSubsystem.setFeederSpeedCommand(-.2, -.2));

                if (!CowboyUtils.isSim()) { // Real robot

                        // Manual scoring button, used ONLY if vision goes down mid-match.
                        new POVButton(operatorController, 0)
                                        .whileTrue(HighLevelCommands.shootFromHopperContinousCommand(
                                                        intakeSubsystem, feederSubsystem, shooterSubsystem, 6000))
                                        .onFalse(HighLevelCommands.stopAllSuperStructure(intakeSubsystem,
                                                        feederSubsystem, shooterSubsystem, ledSubsystem));

                        // Right operator trigger, enables SOTM and turrets the robot. Used for both
                        // automated feeding and scoring.

                        Trigger spinup = new Trigger(()->operatorController.getRawButton(6)).whileTrue(shooterSubsystem.setRPMCommand(6000));
                        
                        // new Trigger(() -> operatorController.getRawAxis(3) > .3)
                        //                 .whileTrue(HighLevelCommands.teleopShootOnMoveAutomationCommand(
                        //                                 driveSubsystem, driveController, intakeSubsystem,
                        //                                 feederSubsystem, shooterSubsystem,
                        //                                 ledSubsystem))
                        //                 .onFalse(HighLevelCommands.stopAllSuperStructure(intakeSubsystem,
                        //                                 feederSubsystem, shooterSubsystem,
                        //                                 ledSubsystem));

                        // Left operator trigger, runs intake and performs the 'ball wave' agitation
                        // while held.
                        new Trigger(() -> operatorController.getRawAxis(2) > .3)
                                        .whileTrue(HighLevelCommands.intakeCommand(intakeSubsystem, feederSubsystem,
                                                        ledSubsystem))
                                        .onFalse(HighLevelCommands.stopAllSuperStructure(intakeSubsystem,
                                                        feederSubsystem, shooterSubsystem,
                                                        ledSubsystem));

                        
                        new JoystickButton(operatorController, 2).onTrue(feederSubsystem.pullBallsBackCommand()).onFalse(feederSubsystem.setFeederSpeedCommand(0, 0));

                        // Operator X button, reverses indexer if needed to clear jams
                        new JoystickButton(operatorController, 3)
                                        .whileTrue(HighLevelCommands.reverseSuperstructure(intakeSubsystem,
                                                        feederSubsystem, shooterSubsystem, ledSubsystem))
                                        .onFalse(HighLevelCommands.stopAllSuperStructure(intakeSubsystem,
                                                        feederSubsystem, shooterSubsystem,
                                                        ledSubsystem));

                        new JoystickButton(driveController, 7)
                                        .whileTrue(new SequentialCommandGroup(
                                                        Commands.deferredProxy(
                                                                        () -> questNavSubsystem.resetPoseYaw(
                                                                                        new Rotation2d())),
                                                        driveSubsystem.gyroReset()));

                        // Manually re-seed the encoders in the Neo motors. Used if there is any mid
                        // match misalignment, ONLY used if needed. Reset on robot boot is ALWAYS ran.
                        new JoystickButton(driveController, 7).onTrue(driveSubsystem.resetEncodersCommand());
                }

                else { // Sim, just the one Logitech F310 controller for testing

                        new Trigger(() -> driveController.getRawAxis(3) > .4)
                                        .whileTrue(HighLevelCommands.teleopShootOnMoveAutomationCommand(
                                                        driveSubsystem, driveController, intakeSubsystem,
                                                        feederSubsystem, shooterSubsystem,
                                                        ledSubsystem))
                                        .onFalse(HighLevelCommands.stopAllSuperStructure(intakeSubsystem,
                                                        feederSubsystem, shooterSubsystem,
                                                        ledSubsystem));

                        new Trigger(() -> driveController.getRawAxis(2) > .4)
                                        .whileTrue(HighLevelCommands.intakeCommand(intakeSubsystem, feederSubsystem,
                                                        ledSubsystem))
                                        .onFalse(HighLevelCommands.stopAllSuperStructure(intakeSubsystem,
                                                        feederSubsystem, shooterSubsystem,
                                                        ledSubsystem));

                        new JoystickButton(driveController, 8)
                                        .whileTrue(new SequentialCommandGroup(
                                                        Commands.deferredProxy(
                                                                        () -> questNavSubsystem.resetPoseYaw(
                                                                                        new Rotation2d())),
                                                        driveSubsystem.gyroReset()));
                }
        }

        public Command getTestingCommand() {
                return new RobotSystemsCheckCommand(driveSubsystem);
        }

        public Field2d getField() {
                return field;
        }

}
