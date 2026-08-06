package frc.robot.subsystems.drive;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DriveSubsystemConstants {
        public static final double FRONT_LEFT_VIRTUAL_OFFSET_RADIANS = 0;
        public static final double FRONT_RIGHT_VIRTUAL_OFFSET_RADIANS = 0; // -We do not apply an offset to the
                                                                           // CANcoder
                                                                           // angle, we just zero the encoders
                                                                           // with the
                                                                           // wheels forward with bolt side
                                                                           // facing
                                                                           // LEFT!!!
                                                                           // -In radians not degrees
        public static final double REAR_LEFT_VIRTUAL_OFFSET_RADIANS = 0;
        public static final double REAR_RIGHT_VIRTUAL_OFFSET_RADIANS = 0;

        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double MAX_SPEED_METERS_PER_SECOND = 6.0;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI; // radians per second

        public static final double DIRECTION_SLEW_RATE = 8;
        public static final double MAGNITUDE_SLEW_RATE = 8; // Responsiveness, or the "jerk" of the drivebase
        public static final double ROTATIONAL_SLEW_RATE = 6;

        // Chassis configuration

        public static final double DRIVE_BASE_RADIUS_METERS = Units.inchesToMeters(15.38); // measurement from
                                                                                           // center point of
                                                                                           // robot
        // to the
        // center of one of the wheels. (use the
        // CAD)

        public static final double LEFT_RIGHT_DISTANCE_METERS = Units.inchesToMeters(21.750000); // Distance
                                                                                                 // between
                                                                                                 // centers of
                                                                                                 // right
        // and left wheels on robot

        public static final double FRONT_BACK_DISTANCE_METERS = Units.inchesToMeters(21.750000);// Distance
                                                                                                // between
                                                                                                // front and
                                                                                                // back
        // wheels on robot

        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                        new Translation2d(LEFT_RIGHT_DISTANCE_METERS / 2, FRONT_BACK_DISTANCE_METERS / 2),
                        new Translation2d(LEFT_RIGHT_DISTANCE_METERS / 2, -FRONT_BACK_DISTANCE_METERS / 2),
                        new Translation2d(-LEFT_RIGHT_DISTANCE_METERS / 2, FRONT_BACK_DISTANCE_METERS / 2),
                        new Translation2d(-LEFT_RIGHT_DISTANCE_METERS / 2, -FRONT_BACK_DISTANCE_METERS / 2));

        public static final int GYRO_ORIENTATION = -1; // 1 for upside down, -1 for right side up.

        public static final boolean FIELD_RELATIVE = true;

        public static enum DrivingRatios {
                // Enum constants for number of teeth
                R1(12),
                R2(14),
                R3(16);

                private final int ratioValue;

                DrivingRatios(int ratioValue) {
                        this.ratioValue = ratioValue;
                }

                public int getValue() {
                        return ratioValue;
                }
        }

        public static final DrivingRatios DRIVING_RATIO = DrivingRatios.R2;

        public static final double WHEEL_DIAMETER_METERS = 0.1016;

        public static final double TURNING_MOTOR_REDUCTION = 26; // Ratio between internal relative
                                                                 // encoder and
                                                                 // the absolute encoder

        public static final double TRANSLATION_P = 1.0;
        public static final double ROT_MOTION_P = 0.0;

        public static final double TRANSLATION_I = 0.0;
        public static final double ROT_MOTION_I = 0.0;

        public static final double TRANSLATION_D = 0.0;
        public static final double ROT_MOTION_D = 0.0;

        public static final double FREE_SPEED_RPM = 5676;

        // public static final boolean TURNING_ENCODER_INVERTED = false;

        public static final double DRIVING_MOTOR_FREE_SPEED_RPS = FREE_SPEED_RPM / 60;

        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

        public static final double DRIVING_MOTOR_REDUCTION = (54.0 * 25 * 30)
                        / (DRIVING_RATIO.getValue() * 32 * 15);

        public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS
                        * WHEEL_CIRCUMFERENCE_METERS) / DRIVING_MOTOR_REDUCTION;

        public static final double DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION = (WHEEL_DIAMETER_METERS
                        * Math.PI) / DRIVING_MOTOR_REDUCTION; // meters, per rotation
        public static final double DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND_PER_RPM = ((WHEEL_DIAMETER_METERS
                        * Math.PI) / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second, per RPM

        public static final double TURNING_ENCODER_POSITION_FACTOR_RADIANS_PER_ROTATION = (2 * Math.PI)
                        / TURNING_MOTOR_REDUCTION; // radians, per rotation
        public static final double TURNING_ENCODER_VELOCITY_FACTOR_RADIANS_PER_SECOND_PER_RPM = (2 * Math.PI)
                        / TURNING_MOTOR_REDUCTION / 60.0; // radians per second, per RPM

        public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS = 0; // radians
        public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS = (2 * Math.PI); // radians

        // These PID constants relate to the movement and acceleration of the swerve
        // motors themselfs.
        public static final double DRIVING_P = 0.07;
        public static final double DRIVING_I = 0;
        public static final double DRIVING_D = 0;
        public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
        public static final double DRIVING_MIN_OUTPUT_NORMALIZED = -1;
        public static final double DRIVING_MAX_OUTPUT_NORMALIZED = 1;

        public static final double TURNING_P = 1.25;
        public static final double TURNING_I = 0;
        public static final double TURNING_D = 0;
        public static final double TURNING_FF = 0;
        public static final double TURNING_MIN_OUTPUT_NORMALIZED = -1;
        public static final double TURNING_MAX_OUTPUT_NORMALIZED = 1;

        public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

        public static final int DRIVING_MOTOR_CURRENT_LIMIT_AMPS = 40; // amps
        public static final int TURNING_MOTOR_CURRENT_LIMIT_AMPS = 20; // amps

        public static final class PathPlannerConstants {
                // public static final Alliance DEFAULT_ALLIANCE = Alliance.Blue;

                public static final double kMaxAngularAcceleration = 4 * Math.PI;
                public static final double kMaxAccelerationMetersPerSecondSquared = 3.00;

                public static final PathConstraints DEFAULT_PATH_CONSTRAINTS = new PathConstraints(
                                .3,
                                .3,
                                .2,
                                2 * Math.PI);

                public static final double MAX_VELOCITY = 6.0; // Meters per second
                public static final double MAX_ACCELERATION = 6.0; // Meters per second squared
                public static final double MAX_ANGULAR_SPEED = 540.0; // Degrees per second
                public static final double MAX_ANGULAR_ACCELERATION = 720.0; // Degrees per second squared
        }
}
