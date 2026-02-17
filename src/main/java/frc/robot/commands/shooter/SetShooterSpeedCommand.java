package frc.robot.commands.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.RobotState.IntakePositions;
import frc.robot.RobotState;
import frc.robot.RobotConstants.IntakeContants;
import frc.robot.utils.CowboyUtils;

public class SetShooterSpeedCommand extends Command {
    double percent;
    ShooterSubsystem shooterSubsystem;

    public SetShooterSpeedCommand(double percent, ShooterSubsystem shooterSubsystem) {
        
        this.shooterSubsystem = shooterSubsystem;
        this.percent = percent;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setPercentSpeed(-percent);
    }

    @Override
    public void execute() {

        
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setPercentSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}