package frc.robot.subsystems.led;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotConstants.LEDConstants.AnimationTypes;

public class LEDSubsystem extends SubsystemBase {
    public LEDSubsystemIO io;
    LEDSubsystemIOInputsAutoLogged inputs = new LEDSubsystemIOInputsAutoLogged();

    public LEDSubsystem(LEDSubsystemIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("LEDSubsystem", inputs);
    }
}