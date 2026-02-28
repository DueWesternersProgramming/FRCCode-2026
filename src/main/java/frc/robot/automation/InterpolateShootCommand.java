package frc.robot.automation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.PortConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.CowboyUtils;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class InterpolateShootCommand extends Command {

    // Angular controller (RADIANS)
    private final ProfiledPIDController angleController = new ProfiledPIDController(
            1.25, 0.0, 0.0,
            new TrapezoidProfile.Constraints(
                    Units.degreesToRadians(360), // max angular velocity
                    Units.degreesToRadians(720) // max angular acceleration
            ));


    private final DriveSubsystem driveSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final Joystick joystick;

    public InterpolateShootCommand(
            DriveSubsystem driveSubsystem,
            ShooterSubsystem shooterSubsystem,
            Joystick joystick) {

        this.driveSubsystem = driveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.joystick = joystick;

        addRequirements(driveSubsystem);

        angleController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {

        Pose2d robotPose = driveSubsystem.getPose();
        Pose2d hubPose = CowboyUtils.getAllianceHubPose();

        double dx = robotPose.getX() - hubPose.getX();
        double dy = robotPose.getY() - hubPose.getY();

        double angleToRobot = Math.atan2(dy, dx);
    
        angleController.setGoal(angleToRobot + Math.PI);

        double rotOutput = angleController.calculate(
                robotPose.getRotation().getRadians());

        driveSubsystem.drive(
                0,0,
                rotOutput,
                true,
                true, 
                false);

        shooterSubsystem.setPercentSpeed(shooterSubsystem.getPercentFromDistance(robotPose.getTranslation().getDistance(hubPose.getTranslation())));
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
