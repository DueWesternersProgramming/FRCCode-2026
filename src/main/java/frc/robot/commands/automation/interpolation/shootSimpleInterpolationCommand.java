package frc.robot.commands.automation.interpolation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class shootSimpleInterpolationCommand extends Command {

    ShooterSubsystem shooterSubsystem;
    Pose2d target;

    public shootSimpleInterpolationCommand(
            ShooterSubsystem shooterSubsystem,Pose2d target) {

        this.shooterSubsystem = shooterSubsystem;
        this.target = target;

        addRequirements(shooterSubsystem);

    }

    @Override
    public void execute() {

        Pose2d currentRobotPose = RobotState.robotPose;

        double distanceToHub = currentRobotPose.getTranslation().getDistance(target.getTranslation());

        shooterSubsystem.setPercentSpeed(shooterSubsystem.getPercentFromDistance(distanceToHub));
    };

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
