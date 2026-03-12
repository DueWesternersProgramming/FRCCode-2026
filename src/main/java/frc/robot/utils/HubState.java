package frc.robot.utils;

public class HubState {
    public boolean active;
    public double timeUntilSwap;

    public HubState(boolean active, double timeUntilSwap){
        this.active = active;
        this.timeUntilSwap = timeUntilSwap;
    }
}
