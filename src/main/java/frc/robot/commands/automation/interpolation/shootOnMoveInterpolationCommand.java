package frc.robot.commands.automation.interpolation;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.Tuning;
import frc.robot.RobotConstants.PortConstants.Controller;
import frc.robot.RobotConstants.TeleopConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class shootOnMoveInterpolationCommand extends Command {

        private final PIDController angleController = new PIDController(
                        1.25, 0.0, 0.0);

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

                Logger.recordOutput("Interpolation/targetAngle", Units.radiansToDegrees(predictedAngleToRobot));
                Logger.recordOutput("Interpolation/currentAngle", currentRobotPose.getRotation().getRadians());
                
                double rotOutput = angleController.calculate(currentRobotPose.getRotation().getRadians(), predictedAngleToRobot);

                driveSubsystem.drive(
                                (ySquared*MathUtil.clamp(Math.abs(Math.sin(predictedAngleToRobot)),.3,1)), (xSquared*MathUtil.clamp(Math.abs(Math.sin(predictedAngleToRobot)),.3,1)),
                                Tuning.sotmEnabled.get() ? rotOutput : 0,
                                true,
                                true,
                                false);

                double shooterSpeed = shooterSubsystem.getRPMFromDistance(predictedDistanceToHub);

                if (Tuning.tuningEnabled.get()){
                        shooterSubsystem.setPercentSpeed(Tuning.tuningVoltage.get()); //directly taken as a percent, read out rpm on dashboard
                }
                else{
                        shooterSubsystem.setRPM(shooterSpeed);
                }
                
                Logger.recordOutput("Tuning/predictedDistanceToHub", predictedDistanceToHub);
        };

        @Override
        public boolean isFinished() {
                return false;
        }

        @Override
        public void end(boolean interrupted) {
        }
}