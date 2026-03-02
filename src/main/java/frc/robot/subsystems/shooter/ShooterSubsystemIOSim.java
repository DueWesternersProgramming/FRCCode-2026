package frc.robot.subsystems.shooter;


public class ShooterSubsystemIOSim implements ShooterSubsystemIO {
    private double rpm;
    private double percent;

    public void setPercentSpeed(double percent){
        this.percent = percent;
    }

    public void setRMP(double rpm){
        this.rpm = rpm;
    }

    @Override
    public void updateInputs(ShooterSubsystemIOInputs inputs) {
        inputs.motorRMP = rpm;
        inputs.motorPercent = percent;
        inputs.motorTempC = 0;
    }
}
