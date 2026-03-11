package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterSubsystemIO.ShooterSubsystemIOInputs;

public class ShooterSubsystem extends SubsystemBase {
    public ShooterSubsystemIO io;
    ShooterSubsystemIOInputsAutoLogged inputs = new ShooterSubsystemIOInputsAutoLogged();

    public ShooterSubsystem(ShooterSubsystemIO io) {
        this.io = io;
    }

    public void setPercentSpeed(double percent){
        io.setPercentSpeed(percent);
    }

    public void setRPM(double rpm){
        io.setRMP(rpm);
    }

    public Command setPercentSpeedCommand(double percent){
        return new InstantCommand(()->setPercentSpeed(percent), this);
    }

    public Command setRPMCommand(double percent){
        return new InstantCommand(()->setRPM(percent), this);
    }
    
    public double getPercentFromDistance(double distanceMeters){
        return -((distanceMeters * 0.1585) + 0.21);
    }

    public double getTimeOfFlightFromDistance(double distanceMeters){
        return .1; //TODO: Make this equation
    }    

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ShooterSubsystem", inputs);
    }
}