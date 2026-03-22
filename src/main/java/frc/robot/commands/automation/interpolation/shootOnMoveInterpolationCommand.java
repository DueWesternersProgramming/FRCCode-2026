package frc.robot.commands.automation.interpolation;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.RobotState;
import frc.robot.RobotConstants.PortConstants.Controller;
import frc.robot.RobotConstants.TeleopConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.CowboyUtils;

public class shootOnMoveInterpolationCommand extends Command {

        // Angular controller (RADIANS)
        private final ProfiledPIDController angleController = new ProfiledPIDController(
                        1.25, 0.0, 0.0,
                        new TrapezoidProfile.Constraints(
                                        Units.degreesToRadians(500), // max angular velocity
                                        Units.degreesToRadians(700) // max angular acceleration
                        ));

        DriveSubsystem driveSubsystem;
        ShooterSubsystem shooterSubsystem;
        Joystick joystick;
        Supplier<Pose2d> targetSupplier;

        public shootOnMoveInterpolationCommand(
                        DriveSubsystem driveSubsystem,
                        ShooterSubsystem shooterSubsystem,
                        Joystick joystick, Supplier<Pose2d> targetSupplier) {

                this.driveSubsystem = driveSubsystem;
                this.shooterSubsystem = shooterSubsystem;
                this.joystick = joystick;
                this.targetSupplier = targetSupplier;

                addRequirements(driveSubsystem);

                angleController.enableContinuousInput(-Math.PI, Math.PI);
        }

        @Override
        public void execute() {

                Pose2d target = targetSupplier.get();

                double xRaw = -(joystick.getRawAxis(Controller.DRIVE_COMMAND_X_AXIS));
                double yRaw = -(joystick.getRawAxis(Controller.DRIVE_COMMAND_Y_AXIS));
                double rotRaw = -(joystick.getRawAxis(Controller.DRIVE_COMMAND_ROT_AXIS));

                double xConstrained = MathUtil.applyDeadband(
                                MathUtil.clamp(xRaw, -TeleopConstants.MAX_SPEED_PERCENT,
                                                TeleopConstants.MAX_SPEED_PERCENT),
                                RobotConstants.PortConstants.Controller.JOYSTICK_AXIS_THRESHOLD);
                double yConstrained = MathUtil.applyDeadband(
                                MathUtil.clamp(yRaw, -TeleopConstants.MAX_SPEED_PERCENT,
                                                TeleopConstants.MAX_SPEED_PERCENT),
                                RobotConstants.PortConstants.Controller.JOYSTICK_AXIS_THRESHOLD);

                double rotConstrained = MathUtil.applyDeadband(
                                MathUtil.clamp(rotRaw, -TeleopConstants.MAX_SPEED_PERCENT,
                                                TeleopConstants.MAX_SPEED_PERCENT),
                                RobotConstants.PortConstants.Controller.JOYSTICK_AXIS_THRESHOLD);

                double xSquared = Math.copySign(xConstrained * xConstrained, xConstrained);
                double ySquared = Math.copySign(yConstrained * yConstrained, yConstrained);
                double rotSquared = Math.copySign(rotConstrained * rotConstrained, rotConstrained);

                Pose2d currentRobotPose = driveSubsystem.getPose();

                double currentDistanceToHub = currentRobotPose.getTranslation().getDistance(target.getTranslation());

                double tof = shooterSubsystem.getTimeOfFlightFromDistance(currentDistanceToHub);

                ChassisSpeeds currentChassisSpeeds = driveSubsystem.getChassisSpeeds();

                Pose2d predictedRobotPose = new Pose2d(
                                (currentRobotPose.getX() + (currentChassisSpeeds.vxMetersPerSecond * tof)),
                                (currentRobotPose.getY() + (currentChassisSpeeds.vyMetersPerSecond * tof)),
                                new Rotation2d()); // rotation is not set since we don't really care about that here

                Logger.recordOutput("Interpolation/PredictedRobotPose", predictedRobotPose);
                Logger.recordOutput("Interpolation/target", target);

                Logger.recordOutput("Interpolation/VX", currentChassisSpeeds.vxMetersPerSecond);
                Logger.recordOutput("Interpolation/VY", currentChassisSpeeds.vyMetersPerSecond);

                double predictedDistanceToHub = predictedRobotPose.getTranslation()
                                .getDistance(target.getTranslation());

                double dx = target.getX() - predictedRobotPose.getX();
                double dy = target.getY() - predictedRobotPose.getY();

                double predictedAngleToRobot = Math.atan2(dy, dx);

                angleController.setGoal(predictedAngleToRobot);

                Logger.recordOutput("Interpolation/targetAngle", predictedAngleToRobot);
                Logger.recordOutput("Interpolation/currentAngle", currentRobotPose.getRotation().getRadians());

                double rotOutput = angleController.calculate(
                                currentRobotPose.getRotation().getRadians());

                // driveSubsystem.drive(
                //                 ySquared, xSquared,
                //                 rotOutput,
                //                 true,
                //                 true,
                //                 false);

                if (RobotState.canRotate) {
                        driveSubsystem.drive(ySquared, xSquared, rotSquared, true, true,
                                        RobotState.isAntiTippingEnabled);
                } else {
                        driveSubsystem.drive(ySquared, xSquared, 0, true, true, true);
                }

                double shooterSpeed = MathUtil.clamp(0, -1, rotOutput);

                shooterSubsystem.setPercentSpeed(shooterSubsystem.getPercentFromDistance(predictedDistanceToHub));
        };

        @Override
        public boolean isFinished() {
                return false;
        }

        @Override
        public void end(boolean interrupted) {
                angleController.reset(
                                driveSubsystem.getPose().getRotation().getRadians());
        }
}