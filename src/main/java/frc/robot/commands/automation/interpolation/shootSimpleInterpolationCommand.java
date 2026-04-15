package frc.robot.commands.automation.interpolation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.Supplier;

public class shootSimpleInterpolationCommand extends Command {

    private final ShooterSubsystem shooterSubsystem;
    private final Supplier<Pose2d> targetSupplier;

    /**
     * @param shooterSubsystem The subsystem to control
     * @param targetSupplier A lambda or method reference that returns the current target Pose2d
     */
    public shootSimpleInterpolationCommand(
            ShooterSubsystem shooterSubsystem, 
            Supplier<Pose2d> targetSupplier) {

        this.shooterSubsystem = shooterSubsystem;
        this.targetSupplier = targetSupplier;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        Pose2d target = targetSupplier.get();
        
        Pose2d currentRobotPose = RobotState.robotPose;

        double distanceToHub = currentRobotPose.getTranslation().getDistance(target.getTranslation());

        shooterSubsystem.setRPM(shooterSubsystem.getRPMFromDistance(distanceToHub));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setRPM(0);
    }
}