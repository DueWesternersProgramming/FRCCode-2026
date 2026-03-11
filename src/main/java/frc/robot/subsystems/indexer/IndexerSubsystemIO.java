package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;



public interface IndexerSubsystemIO {

    @AutoLog
    public static class IndexerSubsystemIOInputs {
        public double conveyorMotorPercent = 0.0;
        public double conveyorMotorRMP = 0.0;
        public double conveyorMotorTempC = 0.0;

        public double indexerMotorPercent = 0.0;
        public double indexerMotorRPM = 0.0;
        public double indexerMotorTempC = 0.0;
    }

    default void updateInputs(IndexerSubsystemIOInputs inputs) {
    }

    default void setConveyorPercentSpeed(double percent){
    }

    default void setRollerPercentSpeed(double percent){
    }
}
