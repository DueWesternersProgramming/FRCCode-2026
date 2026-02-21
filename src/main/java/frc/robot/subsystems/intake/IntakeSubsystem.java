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

    public void setDeploymentMotorSpeed(double percent) {
        io.setDeploymentMotorPercentSpeed(percent);
    }

    public double getDeploymentMotorEncoderRevs() {
        return io.getDeploymentMotorEncoderRevs();
    }

    public Command setIntakeSpeedCommand(double speed){
        return new InstantCommand(()->io.setPercentSpeed(speed), this);
    }

    public Command runIntakeAgitationContinousCommand() {
        return Commands.repeatingSequence(
                setIntakeSpeedCommand(-.7),
                new WaitCommand(0.075),
                setIntakeSpeedCommand(.75),
                new WaitCommand(2));
    }

    public Command runIntakeNormalCommand(){
        return new InstantCommand(()->setDeploymentMotorSpeed(-.85), this);
    }

    public Command stopIntakingCommand() {
        return new InstantCommand(() -> io.setPercentSpeed(0), this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IntakeSubsystem", inputs);
    }
}