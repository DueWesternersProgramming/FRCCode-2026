package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterSubsystemIO {

    @AutoLog
    public static class ShooterSubsystemIOInputs {
        public double leftMotorRPM = 0.0;
        public double leftMotorTempC = 0.0;
        public double leftMotorCurrentDraw = 0.0;

        public double rightMotorRPM = 0.0;
        public double rightMotorTempC = 0.0;
        public double rightMotorCurrentDraw = 0.0;
    }

    default void updateInputs(ShooterSubsystemIOInputs inputs) {
    }

    default void setPercentSpeed(double percent){
    }

    default void setRPM(double rpm){

    }
}
