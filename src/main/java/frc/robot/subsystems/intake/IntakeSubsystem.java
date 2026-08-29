package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeSubsystem extends SubsystemBase {
    public IntakeSubsystemIO io;
    IntakeSubsystemIOInputsAutoLogged inputs = new IntakeSubsystemIOInputsAutoLogged();
    

    public IntakeSubsystem(IntakeSubsystemIO io) {
        this.io = io;
    }

    public void setIntakeSpeed(double speed) {
        io.setPercentSpeed(speed);
    }

    public Command setIntakeSpeedCommand(double speed) {
        return new InstantCommand(() -> io.setPercentSpeed(speed), this);
    }

    public Command stopIntakingCommand() {
        return new InstantCommand(() -> io.setPercentSpeed(0), this);
    }

    public double getIntakeRPM() {
        return io.getIntakeRPM();
    }

    public double getIntakeCurrentDraw() {
        return io.getIntakeCurrent();
    }

    public boolean isIntakeStalled() {
        return io.isIntakeStalled();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IntakeSubsystem", inputs);
    }
}