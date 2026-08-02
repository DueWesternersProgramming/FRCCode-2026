package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class RobotConstants {

        public static final class SimMode {
                public static enum SimModes {
                        REGULAR,
                        REPLAY,
                }

                public static final SimModes SIM_MODE = SimModes.REGULAR;
        }

        public static final class ScoringConstants {

                public static enum FieldZones {
                        BLUE_ZONE,
                        NEUTRAL_ZONE,
                        RED_ZONE
                }

                public static enum DynamicAutoScoringPositions {
                        LEFT,
                        CENTER,
                        RIGHT
                }

                public static final Pose2d[] BLUE_ALLIANCE_DYNAMIC_AUTO_SCORING_POSES;
                static {
                        BLUE_ALLIANCE_DYNAMIC_AUTO_SCORING_POSES = new Pose2d[3];
                        BLUE_ALLIANCE_DYNAMIC_AUTO_SCORING_POSES[0] = new Pose2d(3, 5.6,
                                        new Rotation2d(Math.toRadians(-42)));
                        BLUE_ALLIANCE_DYNAMIC_AUTO_SCORING_POSES[1] = new Pose2d(2.3, 4,
                                        new Rotation2d(Math.toRadians(0)));
                        BLUE_ALLIANCE_DYNAMIC_AUTO_SCORING_POSES[2] = new Pose2d(3, 2.5,
                                        new Rotation2d(Math.toRadians(42)));
                }

                public static enum BumpLabels {
                        BLUE_LEFT,
                        BLUE_RIGHT,
                        RED_LEFT,
                        RED_RIGHT
                }

                public static final Pose2d BLUE_ALLIANCE_HUB = new Pose2d(4.630, 4.040, new Rotation2d()); // use the
                                                                                                           // flipping
                                                                                                           // util to
                                                                                                           // get red.
                                                                                                           // See
                                                                                                           // methods in
                                                                                                           // CowboyUtils.

                public static final Pose2d BLUE_ALLIANCE_LEFT_FEEDING_TARGET = new Pose2d(1, 7, new Rotation2d());
                public static final Pose2d BLUE_ALLIANCE_RIGHT_FEEDING_TARGET = new Pose2d(1, 1.5, new Rotation2d());

                public static final Pose2d[][] BUMP_POSITION_POSES;
                static {
                        BUMP_POSITION_POSES = new Pose2d[2][2]; // Red and blue respectively, and then left/right for
                                                                // that corresponding side.
                        BUMP_POSITION_POSES[0][0] = new Pose2d(4.616, 5.5, new Rotation2d(Math.toRadians(0)));
                        BUMP_POSITION_POSES[0][1] = new Pose2d(4.616, 2.5, new Rotation2d(Math.toRadians(0)));
                        BUMP_POSITION_POSES[1][0] = new Pose2d(11.912, 2.5, new Rotation2d(Math.toRadians(0)));
                        BUMP_POSITION_POSES[1][1] = new Pose2d(11.912, 5.5, new Rotation2d(Math.toRadians(0)));
                }
        }

        public static interface PortConstants {

                public static class CAN {
                        public static final int FRONT_LEFT_CANCODER = 1;
                        public static final int FRONT_RIGHT_CANCODER = 2;
                        public static final int REAR_LEFT_CANCODER = 3;
                        public static final int REAR_RIGHT_CANCODER = 4;

                        public static final int FRONT_LEFT_DRIVING = 5;
                        public static final int REAR_LEFT_DRIVING = 7;
                        public static final int FRONT_RIGHT_DRIVING = 6;
                        public static final int REAR_RIGHT_DRIVING = 8;

                        public static final int FRONT_LEFT_TURNING = 9;
                        public static final int REAR_LEFT_TURNING = 11;
                        public static final int FRONT_RIGHT_TURNING = 10;
                        public static final int REAR_RIGHT_TURNING = 12;

                        public static final int PDH = 13;

                        public static final int CANDLE = 14;

                        public static final int INTAKE_MOTOR = 15;


                        public static final int LEFT_SHOOTER_MOTOR = 16;
                        public static final int RIGHT_SHOOTER_MOTOR = 17;
                        
                        public static final int FLOOR_ROLLERS_MOTOR = 18;
                        public static final int VERTICAL_ROLLERS_MOTOR = 19;

                }

                public static class Controller {
                        public static final double JOYSTICK_AXIS_THRESHOLD = 0.2;
                        public static final int DRIVE_CONTROLLER = 0;
                        public static final int OPERATOR_CONTROLLER = 1;

                        // Joystick Axis

                        public static final int DRIVE_COMMAND_X_AXIS = 0;
                        public static final int DRIVE_COMMAND_Y_AXIS = 1;
                        public static final int DRIVE_COMMAND_ROT_AXIS = 4;
                }
        }

        public static final class TeleopConstants {
                public static final double MAX_SPEED_PERCENT = 1; // ex: 0.4 -> 40%
        }


        public static final class FeederConstants {
                public static final double FLOOR_ROLLERS_FEEDING_SPEED = .8;
                public static final double VERTICAL_ROLLERS_FEEDING_SPEED = 1;

                public static final double FLOOR_ROLLERS_REVERSE_SPEED = -0.5;
                public static final double VERTICAL_ROLLERS_REVERSE_SPEED = -0.5;
        }

        public static final class IntakeContants {
                
        }

        

        public static final class SubsystemEnabledConstants {
                public static final boolean VISION_SUBSYSTEM_ENABLED = true;
        }
}
