package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.CowboyUtils;

/**
 * This class is used ONLY for miscellaneous values that should be added to the
 * Elastic driver dashboard.
 */
public class dashboardData {
    private static void controllerStatus() {
        String port0Name = DriverStation.getJoystickName(0);
        String port1Name = DriverStation.getJoystickName(1);

        boolean isOrderCorrect = port0Name.contains("Extreme 3D") && port1Name.contains("Gamepad");
        SmartDashboard.putBoolean("Controller Status", (isOrderCorrect));

        if (!isOrderCorrect) {
            DriverStation.reportWarning("CHECK JOYSTICKS: Driver and Operator might be swapped!", false);
        }
    }

    private static void matchTime() {
        double matchTime = DriverStation.getMatchTime();
        SmartDashboard.putNumber("Match Time", matchTime);
    }

    private static void updateHubStatus() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        double matchTime = DriverStation.getMatchTime();

        // 1. Connection Safety
        if (alliance.isEmpty()) {
            RobotState.hubState.active = false;
            RobotState.hubState.timeUntilSwap = -1;
        }
        // 2. Autonomous Mode (20 Seconds)
        else if (DriverStation.isAutonomousEnabled()) {
            RobotState.hubState.active = true;
            RobotState.hubState.timeUntilSwap = matchTime;
        }
        // 3. Teleop Period (Starts at 150.0: 10s Transition + 110s Tele + 30s Endgame)
        else if (DriverStation.isTeleopEnabled()) {
            String gameData = DriverStation.getGameSpecificMessage();

            if (gameData.isEmpty()) {
                RobotState.hubState.active = true;
                RobotState.hubState.timeUntilSwap = 0;
            } else {
                boolean redInactiveFirst = (gameData.charAt(0) == 'R');
                boolean shift1Active = switch (alliance.get()) {
                    case Red -> !redInactiveFirst;
                    case Blue -> redInactiveFirst;
                };

                // Logic mapped to 150.0s total Teleop clock
                if (matchTime > 140) {
                    // 10s Transition Period (150 down to 140)
                    RobotState.hubState.active = true;
                    RobotState.hubState.timeUntilSwap = matchTime - 140;
                } else if (matchTime > 115) {
                    // Shift 1 (25s: 140 down to 115)
                    RobotState.hubState.active = shift1Active;
                    RobotState.hubState.timeUntilSwap = matchTime - 115;
                } else if (matchTime > 90) {
                    // Shift 2 (25s: 115 down to 90)
                    RobotState.hubState.active = !shift1Active;
                    RobotState.hubState.timeUntilSwap = matchTime - 90;
                } else if (matchTime > 65) {
                    // Shift 3 (25s: 90 down to 65)
                    RobotState.hubState.active = shift1Active;
                    RobotState.hubState.timeUntilSwap = matchTime - 65;
                } else if (matchTime > 40) {
                    // Shift 4 (25s: 65 down to 40)
                    RobotState.hubState.active = !shift1Active;
                    RobotState.hubState.timeUntilSwap = matchTime - 40;
                } else if (matchTime > 30) {
                    // Final period before Endgame (40 down to 30)
                    RobotState.hubState.active = shift1Active;
                    RobotState.hubState.timeUntilSwap = matchTime - 30;
                } else {
                    // End Game (Last 30s): Hub always active
                    RobotState.hubState.active = true;
                    RobotState.hubState.timeUntilSwap = 0;
                }
            }
        }
        // 4. Disabled / Default State
        else {
            RobotState.hubState.active = false;
            RobotState.hubState.timeUntilSwap = -1;
        }

        SmartDashboard.putBoolean("Hub Active", RobotState.hubState.active);
        SmartDashboard.putNumber("Hub Time Until Swap", RobotState.hubState.timeUntilSwap);
    }

    /**
     * This should get called from Robot.java in the main robot periodic
     */
    public static void periodic() {
        controllerStatus();
        matchTime();
        updateHubStatus();
    }
}
