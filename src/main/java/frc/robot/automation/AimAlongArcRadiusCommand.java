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
import frc.robot.utils.CowboyUtils;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class AimAlongArcRadiusCommand extends Command {

    // Angular controller (RADIANS)
    private final ProfiledPIDController angleController = new ProfiledPIDController(
            1.0, 0.0, 0.0,
            new TrapezoidProfile.Constraints(
                    Units.degreesToRadians(360), // max angular velocity
                    Units.degreesToRadians(720) // max angular acceleration
            ));

    // X controller (meters)
    private final PIDController xController = new PIDController(1.0, 0.0, 0.0);

    // Live tuning
    private final LoggedNetworkNumber angleP = new LoggedNetworkNumber("/Tuning/AngleP", 1.0);
    private final LoggedNetworkNumber angleI = new LoggedNetworkNumber("/Tuning/AngleI", 0.0);
    private final LoggedNetworkNumber angleD = new LoggedNetworkNumber("/Tuning/AngleD", 0.0);

    private final DriveSubsystem driveSubsystem;
    private final Joystick joystick;
    private final double radiusMeters;

    public AimAlongArcRadiusCommand(
            DriveSubsystem driveSubsystem,
            double radiusMeters,
            Joystick joystick) {

        this.driveSubsystem = driveSubsystem;
        this.joystick = joystick;
        this.radiusMeters = radiusMeters;

        addRequirements(driveSubsystem);

        angleController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        angleController.setPID(
                angleP.getAsDouble(),
                angleI.getAsDouble(),
                angleD.getAsDouble());

        Pose2d robotPose = driveSubsystem.getPose();
        Pose2d hubPose = CowboyUtils.getAllianceHubPose();

        //AI assisted:

        // Vector from hub -> robot
        double dx = robotPose.getX() - hubPose.getX();
        double dy = robotPose.getY() - hubPose.getY();

        double angleToRobot = Math.atan2(dy, dx);

        // Current distance from hub
        double currentRadius = Math.hypot(dx, dy);

        // Radial error (positive = too far out)
        double radialError = currentRadius - radiusMeters;

        // PID pushes robot back to desired radius
        double radialOutput = xController.calculate(radialError, 0.0);

        // Tangent direction (CCW around hub)
        double tangentX = -Math.sin(angleToRobot);
        double tangentY = Math.cos(angleToRobot);

        // Driver controls motion along arc
        double tangentialSpeed = joystick.getRawAxis(
                PortConstants.Controller.DRIVE_COMMAND_X_AXIS);

        // Combine radial + tangential
        double xOutput = radialOutput * Math.cos(angleToRobot)
                + tangentialSpeed * tangentX;

        double yOutput = radialOutput * Math.sin(angleToRobot)
                + tangentialSpeed * tangentY;

        // Face the hub
        angleController.setGoal(angleToRobot + Math.PI);

        double rotOutput = angleController.calculate(
                robotPose.getRotation().getRadians());

        driveSubsystem.drive(
                xOutput,
                yOutput,
                rotOutput,
                true,
                true, 
                false);

        Logger.recordOutput(
                "DriveSubsystem/HubArc/RadiusError",
                radialError);
    }

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
