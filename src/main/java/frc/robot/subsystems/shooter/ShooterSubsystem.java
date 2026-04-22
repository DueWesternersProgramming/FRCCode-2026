package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterSubsystemIO.ShooterSubsystemIOInputs;

public class ShooterSubsystem extends SubsystemBase {
    public ShooterSubsystemIO io;
    ShooterSubsystemIOInputsAutoLogged inputs = new ShooterSubsystemIOInputsAutoLogged();

    InterpolatingDoubleTreeMap interpolationTable = new InterpolatingDoubleTreeMap();

    public ShooterSubsystem(ShooterSubsystemIO io) {
        this.io = io;
        configureInterpolationTable();
    }

    private void configureInterpolationTable(){
        interpolationTable.put(3.0, 440.0);
        interpolationTable.put(3.25, null);
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
        return (488.96555*distanceMeters) + 1821.2949;
    }

    public double getTimeOfFlightFromDistance(double distanceMeters){
        return 1.4; //TODO: Make this equation
    }    

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ShooterSubsystem", inputs);
    }
}