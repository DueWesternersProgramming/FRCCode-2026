package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
/**
 * Used to define/initilize tunable values, statically accessible from subsystems.
 */

import edu.wpi.first.epilogue.Logged;
public class Tuning {
    public Tuning() {}

    public static final LoggedNetworkNumber tuningRPM =
        new LoggedNetworkNumber("/Tuning/Shooter RPM", 6000);
    public static final LoggedNetworkBoolean tuningEnabled =
        new LoggedNetworkBoolean("/Tuning/Shooter/Tuning Enabled", false);
    public static final LoggedNetworkBoolean sotmEnabled =
        new LoggedNetworkBoolean("/Tuning/SOTM Rotation", true);
    
    public static final LoggedNetworkBoolean harrisonSpeedsActivated =
        new LoggedNetworkBoolean("/Tuning/HarrisonSpeedsActivated", true);
}
