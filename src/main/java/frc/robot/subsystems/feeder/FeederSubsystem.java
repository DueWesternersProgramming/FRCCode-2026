package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FeederSubsystem extends SubsystemBase {
    public FeederSubsystemIO io;
    FeederSubsystemIOInputsAutoLogged inputs = new FeederSubsystemIOInputsAutoLogged();

    public FeederSubsystem(FeederSubsystemIO io) {
        this.io = io;
    }

    public Command setFeederSpeed(double speed){
        return new InstantCommand(()->io.setFeederPercentSpeed(speed), this);
    }

    public Command startFeedingBallsCommand() {
        return new InstantCommand(() -> {
            io.setFeederPercentSpeed(1);
        }, this);
    }

    public Command stopFeedingBallsCommand() {
        return new InstantCommand(()->io.setFeederPercentSpeed(0), this);
    }

    public Command pullBallsBackCommand(){
        return Commands.sequence(
            setFeederSpeed(-.5),
            new WaitCommand(.25),
            setFeederSpeed(0)
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("FeederSubsystem", inputs);
    }
}