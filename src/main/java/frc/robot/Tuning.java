package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
/**
 * Used to define/initilize tunable values, statically accessible from subsystems.
 */
public class Tuning {
    private Tuning() {}

    public static final LoggedNetworkNumber tuningRPM =
        new LoggedNetworkNumber("/Tuning/Shooter RPM", 8);
    public static final LoggedNetworkBoolean tuningEnabled =
        new LoggedNetworkBoolean("/Tuning/Shooter/Tuning Enabled", false);
    public static final LoggedNetworkBoolean sotmEnabled =
        new LoggedNetworkBoolean("/Tuning/SOTM Rotation", true);

    public static final LoggedNetworkNumber shooterP =
        new LoggedNetworkNumber("/Tuning/ShooterP", 0.005);
    
    public static final LoggedNetworkNumber shooterKv =
        new LoggedNetworkNumber("/Tuning/ShooterKv", 0.0001);

}
