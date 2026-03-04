package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is used ONLY for miscellaneous values that should be added to the Elastic driver dashboard.
 */
public class dashboardData {
    public static void controllerStatus() {
        String port0Name = DriverStation.getJoystickName(0);
        String port1Name = DriverStation.getJoystickName(1);

        boolean isOrderCorrect = port0Name.contains("Xbox") && !port1Name.contains("Xbox");
        SmartDashboard.putBoolean("Controller Status", (isOrderCorrect));

        if (!isOrderCorrect) {
            DriverStation.reportWarning("CHECK JOYSTICKS: Driver and Operator might be swapped!", false);
        }
    }

    public static void matchTime(){
        double matchTime = DriverStation.getMatchTime();
        SmartDashboard.putNumber("Match Time", matchTime);
    }

    /**
     * This should get called from Robot.java in the main robot periodic
     */
    public static void periodic(){
        controllerStatus();
    }
}
