// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RobotSystemsCheckCommand;
import frc.robot.commands.automation.AutomatedCommands;
import frc.robot.commands.automation.interpolation.shootOnMoveInterpolationCommand;
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
import frc.robot.subsystems.questnav.QuestNavIO;
import frc.robot.subsystems.questnav.QuestNavIOReal;
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
import frc.robot.autonomous.AutomomousManager.AutoMode;

//@Logged(name = "RobotContainer")
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

        private final Joystick driveJoystick = new Joystick(RobotConstants.PortConstants.Controller.DRIVE_JOYSTICK);
        private final Joystick operatorJoystick = new Joystick(
                        RobotConstants.PortConstants.Controller.OPERATOR_JOYSTICK);

        ModuleIO[] moduleIOs;

        AutomomousManager automomousManager;

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

                automomousManager = new AutomomousManager(driveSubsystem, intakeSubsystem, feederSubsystem, shooterSubsystem,ledSubsystem);

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

                driveSubsystem.setDefaultCommand(new TeleopDriveCommand(driveSubsystem, driveJoystick)); // Same for
                                                                                                         // both sim and
                                                                                                         // real. The
                                                                                                         // joystick
                                                                                                         // axis
                                                                                                         // constants
                                                                                                         // change
                                                                                                         // depending on
                                                                                                         // mode.
                feederSubsystem.setDefaultCommand(feederSubsystem.setFeederSpeedCommand(-.2, -.2));

                if (!CowboyUtils.isSim()) { // Real robot

                        // new JoystickButton(operatorJoystick, 0).onTrue(shooterSubsystem.increaseRPMModificationSpeed());
                        // new JoystickButton(driveJoystick, 0).onTrue(shooterSubsystem.decreaseRPMModificationSpeed());

                        // Manual feeding button
                        new POVButton(operatorJoystick, 0).whileTrue(AutomatedCommands.shootFromHopperContinousCommand(
                                        intakeSubsystem, feederSubsystem, shooterSubsystem, 6000)).onFalse(AutomatedCommands.stopAllSuperStructure(intakeSubsystem, feederSubsystem, shooterSubsystem, ledSubsystem));

                        //Manual scoring button, used ONLY if vision goes down mid-match.
                        new POVButton(operatorJoystick, 180)
                                        .whileTrue(AutomatedCommands.shootFromHopperContinousCommand(
                                                        intakeSubsystem, feederSubsystem,
                                                        shooterSubsystem, 4000)).onFalse(AutomatedCommands.stopAllSuperStructure(intakeSubsystem, feederSubsystem, shooterSubsystem, ledSubsystem));

                        // Right operator trigger, enables SOTM and turrets the robot. Used for both
                        // automated feeding and scoring.
                        new Trigger(() -> operatorJoystick.getRawAxis(3) > .3)
                                        .whileTrue(AutomatedCommands.teleopShootOnMoveAutomationCommand(
                                                        driveSubsystem, driveJoystick, intakeSubsystem,
                                                        feederSubsystem, shooterSubsystem,
                                                        ledSubsystem))
                                        .onFalse(AutomatedCommands.stopAllSuperStructure(intakeSubsystem,
                                                        feederSubsystem, shooterSubsystem,
                                                        ledSubsystem));

                        // Left operator trigger, runs intake while held.
                        new Trigger(() -> operatorJoystick.getRawAxis(2) > .3)
                                        .whileTrue(AutomatedCommands.intakeCommand(intakeSubsystem, feederSubsystem,ledSubsystem))
                                        .onFalse(AutomatedCommands.stopAllSuperStructure(intakeSubsystem,
                                                        feederSubsystem, shooterSubsystem,
                                                        ledSubsystem));

                        // Operator X button, reverses indexer if needed to clear jams
                        new JoystickButton(operatorJoystick, 3)
                                        .whileTrue(AutomatedCommands.reverseSuperstructure(intakeSubsystem,
                                                        feederSubsystem, shooterSubsystem, ledSubsystem))
                                        .onFalse(AutomatedCommands.stopAllSuperStructure(intakeSubsystem,
                                                        feederSubsystem, shooterSubsystem,
                                                        ledSubsystem));

                        new JoystickButton(driveJoystick, 1).onTrue(RobotState.setCanRotate(true))
                                        .onFalse(RobotState.setCanRotate(false));

                        new JoystickButton(driveJoystick, 6)
                                        .whileTrue(new SequentialCommandGroup(
                                                        Commands.deferredProxy(
                                                                        () -> questNavSubsystem.resetPoseYaw(
                                                                                        new Rotation2d())),
                                                        driveSubsystem.gyroReset()));

                        //Manually re-seed the encoders in the Neo motors. Used if there is any mid match misalignment, ONLY used if needed. Reset on robot boot is ALWAYS ran.
                        new JoystickButton(driveJoystick, 7).onTrue(driveSubsystem.resetEncodersCommand());
                }

                else { // Sim, just the one Logitech F310 controller for testing

                        new Trigger(() -> driveJoystick.getRawAxis(3) > .4)
                                        .whileTrue(AutomatedCommands.teleopShootOnMoveAutomationCommand(
                                                        driveSubsystem, driveJoystick, intakeSubsystem,
                                                        feederSubsystem, shooterSubsystem,
                                                        ledSubsystem))
                                        .onFalse(AutomatedCommands.stopAllSuperStructure(intakeSubsystem,
                                                        feederSubsystem, shooterSubsystem,
                                                        ledSubsystem));

                        new Trigger(() -> driveJoystick.getRawAxis(2) > .4)
                                        .whileTrue(AutomatedCommands.intakeCommand(intakeSubsystem, feederSubsystem, ledSubsystem))
                                        .onFalse(AutomatedCommands.stopAllSuperStructure(intakeSubsystem,
                                                        feederSubsystem, shooterSubsystem,
                                                        ledSubsystem));

                        new JoystickButton(driveJoystick, 8)
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
