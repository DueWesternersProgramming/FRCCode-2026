package frc.robot.subsystems.indexer;

public class IndexerSubsystemIOSim implements IndexerSubsystemIO {
    private double conveyorPercent = 0.0;
    private double indexerPercent = 0.0;

    @Override
    public void setConveyorPercentSpeed(double percent) {
        conveyorPercent = percent;
    }

    @Override
    public void setRollerPercentSpeed(double percent) {
        indexerPercent = percent;
    }

    @Override
    public void updateInputs(IndexerSubsystemIOInputs inputs) {
        inputs.conveyorMotorPercent = conveyorPercent;
        inputs.conveyorMotorRMP = 0.0;
        inputs.conveyorMotorTempC = 0.0;

        inputs.indexerMotorPercent = indexerPercent;
        inputs.indexerMotorRPM = 0.0;
        inputs.indexerMotorTempC = 0.0;
    }
}
