package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.RobotState;



public interface IntakeSubsystemIO {

    @AutoLog
    public static class IntakeSubsystemIOInputs {
        public double motorRMP = 0.0;
        public double motorTempC = 0.0;
        public double deployMotorRMP = 0.0;
        public double deployMotorTempC = 0.0;
        public double deployMotorEncoderPosition = 0.0;
    }

    default void updateInputs(IntakeSubsystemIOInputs inputs) {
    }

    default void setPercentSpeed(double percent){
    }

    default double getDeploymentMotorEncoderRevs(){return 0.0;}

    default void setDeploymentMotorPercentSpeed(double percent){}
}
