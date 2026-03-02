package frc.robot.automation;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.PortConstants.Controller;
import frc.robot.RobotConstants.TeleopConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.CowboyUtils;

public class AutomatedBumpCrossing extends Command {

        private final DriveSubsystem driveSubsystem;
        private final Joystick joystick;

        public AutomatedBumpCrossing(
                        DriveSubsystem driveSubsystem,
                        Joystick joystick, Pose2d target) {

                this.driveSubsystem = driveSubsystem;
                this.joystick = joystick;

                addRequirements(driveSubsystem);
        }

        @Override
        public void initialize(){
            Pose2d bumpPose = CowboyUtils.getBumpPosition(CowboyUtils.getClosestBump(driveSubsystem.getPose()));

            CommandScheduler.getInstance().schedule(AutomatedScoring.PPmoveToPose(bumpPose));
        }

        @Override
        public void execute() {
        };

        @Override
        public boolean isFinished() {
                return false;
        }

        @Override
        public void end(boolean interrupted) {
        }
}
