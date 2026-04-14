package frc.robot.subsystems.led;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotConstants.LEDConstants.AnimationTypes;
import frc.robot.RobotConstants.LEDConstants.LEDModes;

public class LEDSubsystem extends SubsystemBase {
    public LEDSubsystemIO io;
    LEDSubsystemIOInputsAutoLogged inputs = new LEDSubsystemIOInputsAutoLogged();

    public LEDSubsystem(LEDSubsystemIO io) {
        this.io = io;
    }

    public Command setLEDModeCommand(LEDModes mode){
        return new InstantCommand(()->setLEDMode(mode), this);
    }

    public void setLEDMode(LEDModes mode){
        switch (mode) {
            case IDLE:
                break;
            case INTAKING:
                break;
            case SHOOTING:
                break;
            case REVERSING:
                break;        
            default:
                break;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("LEDSubsystem", inputs);
    }
}