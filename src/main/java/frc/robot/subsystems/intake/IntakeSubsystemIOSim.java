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
    public void updateInputs(IntakeSubsystemIOInputs inputs) {
        inputs.intakePercent = percent;
        inputs.intakeTempC = 0;
        //inputs.deployMotorEncoderPosition = 0;
    }
}
