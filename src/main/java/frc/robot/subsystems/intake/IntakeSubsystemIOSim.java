package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.RobotState;

public class IntakeSubsystemIOSim implements IntakeSubsystemIO {
    private double percent = 0.0;

    @Override
    public void setPercentSpeed(double percent){
        this.percent = percent;
    }

    @Override
    public double getIntakeRPM() {
        return percent * 5676;
    }

    @Override
    public double getIntakeCurrent() {
        return percent * 30;
    }

    @Override
    public boolean isIntakeStalled() {
        return false;
    }

    @Override
    public void updateInputs(IntakeSubsystemIOInputs inputs) {
        inputs.intakePercent = percent;
        inputs.intakeTempC = 0;
        inputs.intakeRPM = getIntakeRPM();
        inputs.intakeCurrent = getIntakeCurrent();
        inputs.intakeStalled = isIntakeStalled();
    }

    
}
