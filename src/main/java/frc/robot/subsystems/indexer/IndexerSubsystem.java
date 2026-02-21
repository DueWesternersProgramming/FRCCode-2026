package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IndexerSubsystem extends SubsystemBase {
    public IndexerSubsystemIO io;
    IndexerSubsystemIOInputsAutoLogged inputs = new IndexerSubsystemIOInputsAutoLogged();

    public IndexerSubsystem(IndexerSubsystemIO io) {
        this.io = io;
    }


    public Command setIndexerSpeedCommand(double conveyerSpeed, double rollerSpeed){
        return new InstantCommand(()->{
            io.setConveyorPercentSpeed(conveyerSpeed);
            io.setRollerPercentSpeed(rollerSpeed);
        }, this);
    }

    public Command runIndexerAgitationContinousCommand() {
        return Commands.repeatingSequence(
                setIndexerSpeedCommand(1,1),
                new WaitCommand(4),
                setIndexerSpeedCommand(-.8,-.6),
                new WaitCommand(.2));
    }

    public Command stopIndexing() {
        return new InstantCommand(() -> {
            io.setConveyorPercentSpeed(0);
            io.setRollerPercentSpeed(0);
        }, this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IndexerSubsystem", inputs);
    }
}