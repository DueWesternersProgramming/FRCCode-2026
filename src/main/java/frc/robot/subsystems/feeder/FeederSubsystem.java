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

    public Command setFeederSpeedCommand(double floorRollers, double cageRollers) {
        return new InstantCommand(() -> {
            io.setFloorRollersPercentSpeed(floorRollers);
            io.setVerticalRollersPercentSpeed(cageRollers);
        }, this);
    }

    public Command pullBallsBackCommand() {
        return Commands.sequence(
                setFeederSpeedCommand(-.7, -.7),
                new WaitCommand(.75),
                setFeederSpeedCommand(0,0));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("FeederSubsystem", inputs);
    }
}