package frc.robot.commands.automation.misc;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveToPoseCommand extends Command {

    private final DriveSubsystem drive;
    private final Supplier<Pose2d> targetSupplier;

    private final PIDController xController = new PIDController(4.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(4.0, 0.0, 0.0);
    private final PIDController thetaController = new PIDController(6.0, 0.0, 0.0);

    private static final double POSITION_TOLERANCE = 0.05; // meters
    private static final double ANGLE_TOLERANCE = 3.0; // degrees

    public DriveToPoseCommand(
            DriveSubsystem drive,
            Supplier<Pose2d> targetSupplier) {

        this.drive = drive;
        this.targetSupplier = targetSupplier;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    @Override
    public void initialize() {

        System.out.println("DriveToPoseCommand: Initializing");

        Pose2d current = drive.getPose();

        System.out.println("Initial Pose : " + current);
        System.out.println("Target Pose  : " + targetSupplier.get());

        xController.reset();
        yController.reset();
        thetaController.reset();

        xController.setTolerance(POSITION_TOLERANCE);
        yController.setTolerance(POSITION_TOLERANCE);
        thetaController.setTolerance(Math.toRadians(ANGLE_TOLERANCE));
    }

    @Override
    public void execute() {

        Pose2d current = drive.getPose();
        Pose2d target = targetSupplier.get();

        double vx = xController.calculate(
                current.getX(),
                target.getX());

        double vy = yController.calculate(
                current.getY(),
                target.getY());

        double omega = thetaController.calculate(
                current.getRotation().getRadians(),
                target.getRotation().getRadians());

        vx = MathUtil.clamp(vx, -4.0, 4.0);
        vy = MathUtil.clamp(vy, -4.0, 4.0);
        omega = MathUtil.clamp(omega, -2.0 * Math.PI, 2.0 * Math.PI);

        ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);

        System.out.println("Current Pose : " + current);
        System.out.println("Target Pose  : " + target);
        System.out.printf(
                "Errors: x=%.3f y=%.3f theta=%.1f°%n",
                target.getX() - current.getX(),
                target.getY() - current.getY(),
                target.getRotation()
                        .minus(current.getRotation())
                        .getDegrees());

        System.out.printf(
                "PID Outputs: vx=%.2f vy=%.2f omega=%.2f%n",
                vx,
                vy,
                omega);

        drive.runChassisSpeeds(speeds, true);
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint()
                && yController.atSetpoint()
                && thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drive.runChassisSpeeds(new ChassisSpeeds());

        xController.reset();
        yController.reset();
        thetaController.reset();
    }
}