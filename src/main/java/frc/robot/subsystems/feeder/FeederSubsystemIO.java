package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederSubsystemIO {

    @AutoLog
    public static class FeederSubsystemIOInputs {
        public double feederPercent = 0.0;
        public double feederMotorRPM = 0.0;
        public double feederMotorTempC = 0.0;
    }

    default void updateInputs(FeederSubsystemIOInputs inputs) {
    }

    default void setFeederPercentSpeed(double percent){
    }
}
