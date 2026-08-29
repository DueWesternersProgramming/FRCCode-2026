package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeSubsystemIO {

    @AutoLog
    public static class IntakeSubsystemIOInputs {
        public double intakePercent = 0.0;
        public double intakeTempC = 0.0;
        public double intakeRPM = 0.0;
        public double intakeCurrent = 0.0;
        public boolean intakeStalled = false;
    }

    default void updateInputs(IntakeSubsystemIOInputs inputs) {
    }

    default void setPercentSpeed(double percent){
    }

    default double getIntakeCurrent() {
        return 0;
    }

    default double getIntakeRPM() {
        return 0;
    }

    default boolean isIntakeStalled() {
        return false;
    }
}
