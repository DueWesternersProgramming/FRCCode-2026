package frc.robot.subsystems.led;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotConstants.LEDConstants.AnimationTypes;
import frc.robot.RobotConstants.LEDConstants.LEDStatus;

public class LEDSubsystem extends SubsystemBase {
    public LEDSubsystemIO io;
    LEDSubsystemIOInputsAutoLogged inputs = new LEDSubsystemIOInputsAutoLogged();

    public LEDSubsystem(LEDSubsystemIO io) {
        this.io = io;
    }

    public Command setLEDStatusCommand(LEDStatus mode){
        return new InstantCommand(()->setLEDStatus(mode), this);
    }


    private void setLEDStatus(LEDStatus status){

        switch (status) {
            case IDLE:
                io.setAnimation(AnimationTypes.SetAll, new RGBWColor(255,0,0));
                break;
            case INTAKING:
                io.setAnimation(AnimationTypes.SetAll, new RGBWColor(3,5,27));
                break;
            case SHOOTING:
                io.setAnimation(AnimationTypes.SetAll, new RGBWColor(97,23,79));
                break;
            case REVERSING:
                io.setAnimation(AnimationTypes.SetAll, new RGBWColor(56,93,45)); 
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