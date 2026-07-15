package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederSubsystemIO {

    @AutoLog
    public static class FeederSubsystemIOInputs {
        public double floorRollersPercent = 0.0;
        public double verticalRollersPercent = 0.0;

        public double floorRollersMotorRPM = 0.0;
        public double verticalRollersMotorRPM = 0.0;

        public double floorRollersMotorTempC = 0.0;
        public double verticalRollersMotorTempC = 0.0;
    }

    default void updateInputs(FeederSubsystemIOInputs inputs) {
    }

    default void setFloorRollersPercentSpeed(double percent){
    }
    default void setVerticalRollersPercentSpeed(double percent){
    }
}
