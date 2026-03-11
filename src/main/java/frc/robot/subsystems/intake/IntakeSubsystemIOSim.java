package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.RobotState;

public class IntakeSubsystemIOSim implements IntakeSubsystemIO {
    public IntakeSubsystemIOSim(){

    }

    @Override
    public void setPercentSpeed(double percent){
        
    }

    @Override
    public void setDeploymentMotorPercentSpeed(double percent){
        
    }

    @Override
    public double getDeploymentMotorEncoderRevs(){
        return 0.0;
    }

    @Override
    public void updateInputs(IntakeSubsystemIOInputs inputs) {
        inputs.motorRMP = 0;
        inputs.motorTempC = 0;
        inputs.deployMotorEncoderPosition = 0;
    }
}
