package frc.robot.subsystems.feeder;

public class FeederSubsystemIOSim implements FeederSubsystemIO {
    public FeederSubsystemIOSim() {

    }

    @Override
    public void setFloorRollersPercentSpeed(double percent) {
    }

    @Override
    public void setVerticalRollersPercentSpeed(double percent) {
    }

    @Override
    public void updateInputs(FeederSubsystemIOInputs inputs) {
        inputs.floorRollersPercent = 0.0;
        inputs.verticalRollersPercent = 0.0;

        inputs.floorRollersMotorRPM = 0.0;
        inputs.verticalRollersMotorRPM = 0.0;

        inputs.floorRollersMotorTempC = 0.0;
        inputs.verticalRollersMotorTempC = 0.0;
    }
}
