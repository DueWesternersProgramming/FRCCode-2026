package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    public IndexerSubsystemIO io;
    IntakeSubsystemIOInputsAutoLogged inputs = new IntakeSubsystemIOInputsAutoLogged();

    public IntakeSubsystem(IntakeSubsystemIO io) {
        this.io = io;
    }

    public void turnIntakeOn(){
        io.setPercentSpeed(.5);
    }

    public void turnIntakeOff(){
        io.setPercentSpeed(0);
    }

    public void setIntakeSpeed(double percent){
        io.setPercentSpeed(percent);
    }

    public void setDeploymentMotorSpeed(double percent){
        io.setDeploymentMotorPercentSpeed(percent);
    }

    public double getDeploymentMotorEncoderRevs(){
        return io.getDeploymentMotorEncoderRevs();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IndexerSubsystem", inputs);
    }
}