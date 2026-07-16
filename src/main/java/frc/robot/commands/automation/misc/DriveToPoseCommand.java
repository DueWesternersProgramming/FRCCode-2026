package frc.robot.commands.automation.misc;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

import java.util.function.Supplier;

public class DriveToPoseCommand extends Command {

    private final DriveSubsystem drive;
    private final Supplier<Pose2d> targetSupplier;

    private final PIDController xController =
            new PIDController(4.0, 0.0, 0.0);

    private final PIDController yController =
            new PIDController(4.0, 0.0, 0.0);

    private final ProfiledPIDController thetaController =
            new ProfiledPIDController(
                    6.0,
                    0.0,
                    0.0,
                    new TrapezoidProfile.Constraints(
                            Math.PI * 4.0,
                            Math.PI * 8.0));

    private final HolonomicDriveController controller =
            new HolonomicDriveController(
                    xController,
                    yController,
                    thetaController);

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

        thetaController.reset(
                current.getRotation().getRadians());

        controller.setTolerance(
                new Pose2d(
                        POSITION_TOLERANCE,
                        POSITION_TOLERANCE,
                        Rotation2d.fromDegrees(ANGLE_TOLERANCE)));
    }

    @Override
    public void execute() {

        Pose2d current = drive.getPose();
        Pose2d target = targetSupplier.get();

        ChassisSpeeds speeds =
                controller.calculate(
                        current,
                        target,
                        0.0,                      // desired translation speed
                        target.getRotation());

        // Optional speed limiting
        speeds.vxMetersPerSecond =
                MathUtil.clamp(
                        speeds.vxMetersPerSecond,
                        -2.0,
                        2.0);

        speeds.vyMetersPerSecond =
                MathUtil.clamp(
                        speeds.vyMetersPerSecond,
                        -2.0,
                        2.0);

        speeds.omegaRadiansPerSecond =
                MathUtil.clamp(
                        speeds.omegaRadiansPerSecond,
                        -Math.PI * 2,
                        Math.PI * 2);

        drive.runChassisSpeeds(speeds); // <-- replace with your method
    }

    @Override
    public boolean isFinished() {
        return controller.atReference();
    }

    @Override
    public void end(boolean interrupted) {
        drive.runChassisSpeeds(new ChassisSpeeds()); // <-- replace if needed
    }
}