package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterSubsystemIO {

    @AutoLog
    public static class ShooterSubsystemIOInputs {
        public double leftMotor1RPM = 0.0;
        public double leftMotor1TempC = 0.0;
        public double leftMotor1CurrentDraw = 0.0;

        public double leftMotor2RPM = 0.0;
        public double leftMotor2TempC = 0.0;
        public double leftMotor2CurrentDraw = 0.0;

        public double rightMotor1RPM = 0.0;
        public double rightMotor1TempC = 0.0;
        public double rightMotor1CurrentDraw = 0.0;

        public double rightMotor2RPM = 0.0;
        public double rightMotor2TempC = 0.0;
        public double rightMotor2CurrentDraw = 0.0;
    }

    default void updateInputs(ShooterSubsystemIOInputs inputs) {
    }

    default void setPercentSpeed(double percent){
    }

    default void setRPM(double rpm){

    }

    default void setVoltage(double volts){
        
    }

    default boolean hasReachedTargetVelocity(){
        return false;
    }
}
