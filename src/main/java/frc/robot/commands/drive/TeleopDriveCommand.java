package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.TeleopConstants;
import frc.robot.RobotConstants.PortConstants.Controller;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystemConstants;

public class TeleopDriveCommand extends Command {
    private final DriveSubsystem drive;
    private final Joystick joystick;
    private double lastIntentionalRotation;
    private final PIDController rotationController = new PIDController(
            2,
            0,
            0);

    public TeleopDriveCommand(DriveSubsystem drive, Joystick joystick) {
        this.drive = drive;
        this.joystick = joystick;
        rotationController.enableContinuousInput(-180, 180);
        this.lastIntentionalRotation = drive.getHeading();
        addRequirements(drive);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public void execute() {
        boolean fieldRelative = DriveSubsystemConstants.FIELD_RELATIVE;

        double xRaw = -(joystick.getRawAxis(Controller.DRIVE_COMMAND_X_AXIS));
        double yRaw = -(joystick.getRawAxis(Controller.DRIVE_COMMAND_Y_AXIS));
        double rotRaw = -(joystick.getRawAxis(Controller.DRIVE_COMMAND_ROT_AXIS));

        double xConstrained = MathUtil.applyDeadband(
                MathUtil.clamp(xRaw, -TeleopConstants.MAX_SPEED_PERCENT, TeleopConstants.MAX_SPEED_PERCENT),
                RobotConstants.PortConstants.Controller.JOYSTICK_AXIS_THRESHOLD);
        double yConstrained = MathUtil.applyDeadband(
                MathUtil.clamp(yRaw, -TeleopConstants.MAX_SPEED_PERCENT, TeleopConstants.MAX_SPEED_PERCENT),
                RobotConstants.PortConstants.Controller.JOYSTICK_AXIS_THRESHOLD);
        double rotConstrained = MathUtil.applyDeadband(
                MathUtil.clamp(rotRaw, -TeleopConstants.MAX_SPEED_PERCENT, TeleopConstants.MAX_SPEED_PERCENT),
                RobotConstants.PortConstants.Controller.JOYSTICK_AXIS_THRESHOLD);

        double xSquared = Math.copySign(xConstrained * xConstrained, xConstrained);
        double ySquared = Math.copySign(yConstrained * yConstrained, yConstrained);
        double rotSquared = Math.copySign(rotConstrained * rotConstrained, rotConstrained);

        if (RobotState.xLocked) {
            drive.setX();
            return;
        }

        RobotState.canRotate = Math.abs(rotSquared) > .05;

        boolean translating = Math.abs(xSquared) > 0.02 ||
                Math.abs(ySquared) > 0.02;

        if (RobotState.canRotate) {
            drive.drive(ySquared, xSquared, rotSquared, fieldRelative, true, RobotState.isAntiTippingEnabled);
            
            lastIntentionalRotation = drive.getHeading();
            
        } else {
            double correction = translating ?MathUtil.clamp(
                    rotationController.calculate(
                            drive.getHeading(),
                            lastIntentionalRotation),
                    -1.0,
                    1.0) : 0;

            drive.drive(ySquared, xSquared, correction,
                    fieldRelative, true, RobotState.isAntiTippingEnabled);
        }

    }

    @Override
    public void initialize() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
