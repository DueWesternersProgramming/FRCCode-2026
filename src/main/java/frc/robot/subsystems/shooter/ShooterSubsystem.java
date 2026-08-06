package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    public ShooterSubsystemIO io;
    ShooterSubsystemIOInputsAutoLogged inputs = new ShooterSubsystemIOInputsAutoLogged();

    private double rpmModification = 0;

    public ShooterSubsystem(ShooterSubsystemIO io) {
        this.io = io;
    }

    public Command increaseRPMModificationSpeed(){
        return Commands.runOnce(()->rpmModification+=25);
    }

    public Command decreaseRPMModificationSpeed(){
        return Commands.runOnce(()->rpmModification+=25);
    }

    public void setRPMModificationSpeed(double rpm){
        rpmModification = rpm;
    }

    public double getRPMModificationSpeed(){
        return rpmModification;
    }

    public void setPercentSpeed(double percent){
        io.setPercentSpeed(percent);
    }

    public void setVoltage(double volts){
        io.setVoltage(volts);
    }

    public void setRPM(double rpm){
        io.setRPM(rpm);
    }

    public Command setPercentSpeedCommand(double percent){
        return new InstantCommand(()->setPercentSpeed(percent), this);
    }

    public Command setRPMCommand(double percent){
        return new InstantCommand(()->setRPM(percent), this);
    }
    
    public double getRPMFromDistance(double distanceMeters){
        return 98* Math.pow(distanceMeters, 1.772) + 4708.98;
    }

    public double getTimeOfFlightFromDistance(double distanceMeters){
        return 1.4; //TODO: Make this equation
    }    

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ShooterSubsystem", inputs);
        Logger.recordOutput("ShooterSubsystem/RPMModificationValue", rpmModification);
    }
}