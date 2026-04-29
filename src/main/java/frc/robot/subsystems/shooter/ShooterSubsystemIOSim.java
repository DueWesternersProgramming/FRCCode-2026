package frc.robot.subsystems.shooter;


public class ShooterSubsystemIOSim implements ShooterSubsystemIO {
    private double rpm;
    private double percent;

    public void setPercentSpeed(double percent){
        this.percent = percent;
    }

    public void setRPM(double rpm){
        this.rpm = rpm;
    }

    public boolean hasReachedTargetVelocity(){
        return true;
    }

    @Override
    public void updateInputs(ShooterSubsystemIOInputs inputs) {
    }
}
