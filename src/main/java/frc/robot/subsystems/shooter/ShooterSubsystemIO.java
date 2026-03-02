package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterSubsystemIO {
//Sets the motor rotations per minute and temperature to 0
    @AutoLog
    public static class ShooterSubsystemIOInputs {
        public double motorRMP = 0.0;
        public double motorPercent = 0.0;
        public double motorTempC = 0.0;

    }

    default void updateInputs(ShooterSubsystemIOInputs inputs) {
    }

    default void setPercentSpeed(double percent){
    }

    default void setRMP(double rpm){

    }
}
