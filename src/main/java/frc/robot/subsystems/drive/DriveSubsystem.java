package frc.robot.subsystems.drive;

import java.util.stream.IntStream;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.utils.AntiTipping;
import frc.robot.utils.CowboyUtils;
import frc.robot.RobotConstants.SimMode;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.questnav.QuestNavSubsystemConstants;
import frc.robot.utils.SwerveUtils;
import frc.robot.utils.TimestampedPose;
import com.pathplanner.lib.config.RobotConfig;
import org.littletonrobotics.junction.Logger;

/**
 * The {@code Drivetrain} class contains fields and methods pertaining to the
 * function of the drivetrain.
 * 
 */
public class DriveSubsystem extends SubsystemBase {
    RobotConfig config;

    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveSubsystemConstants.magnitudeSlewRate());
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveSubsystemConstants.rotationalSlewRate());
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;
    private Rotation2d m_trackedRotation = new Rotation2d();

    private static SwerveDrivePoseEstimator hybridOdometry;

    GyroIO gyroIO;
    GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    ModuleIO[] moduleIO;
    ModuleIOInputsAutoLogged[] moduleInputs;

    AntiTipping antiTipping;

    FollowPath.Builder bLinePathBuilder;

    /** Creates a new Drivetrain. */
    public DriveSubsystem(ModuleIO[] moduleIOs, GyroIO gyroIO) {
        this.moduleIO = moduleIOs;
        this.gyroIO = gyroIO;

        // Create one AutoLogged instance per module:
        moduleInputs = IntStream.range(0, moduleIOs.length)
                .mapToObj(i -> new ModuleIOInputsAutoLogged())
                .toArray(ModuleIOInputsAutoLogged[]::new);

        hybridOdometry = new SwerveDrivePoseEstimator(
                DriveSubsystemConstants.DRIVE_KINEMATICS,
                gyroIO.getGyroYawRotation2d(),
                new SwerveModulePosition[] {
                        moduleIOs[0].getPosition(),
                        moduleIOs[1].getPosition(),
                        moduleIOs[2].getPosition(),
                        moduleIOs[3].getPosition()
                }, new Pose2d(0, 0, new Rotation2d()));

        gyroIO.reset();
        resetEncoders();

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // // Configure AutoBuilder
        // AutoBuilder.configure(
        //         this::getPose, // Robot pose supplier
        //         this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
        //         this::getRobotRelativeChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        //         (speeds) -> runChassisSpeeds(speeds, false), // Method that will drive the robot given ROBOT RELATIVE
        //                                                      // ChassisSpeeds. Also optionally outputs individual module
        //                                                      // feedforwards
        //         new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
        //                                         // holonomic drive trains
        //                 new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        //                 new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        //         ),
        //         config, // The robot configuration
        //         () -> {
        //             // Boolean supplier that controls when the path will be mirrored for the red
        //             // alliance
        //             // This will flip the path being followed to the red side of the field.
        //             // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        //             var alliance = DriverStation.getAlliance();
        //             if (alliance.isPresent()) {
        //                 return alliance.get() == DriverStation.Alliance.Red;
        //             }
        //             return false;
        //         },
        //         this // Reference to this subsystem to set requirements
        // );
        // PathfindingCommand.warmupCommand().schedule();

        // B-Line Path Builder
        bLinePathBuilder = new FollowPath.Builder(
                this, // The drive subsystem to require
                this::getPose, // Supplier for current robot pose
                this::getRobotRelativeChassisSpeeds, // Supplier for current speeds
                this::runChassisSpeeds, // Consumer to drive the robot
                new PIDController(10.0, 0.0, 0.0), // Translation PID
                new PIDController(3.0, 0.0, 0.0), // Rotation PID
                new PIDController(2.0, 0.0, 0.0) // Cross-track PID
        )
                .withDefaultShouldFlip(); // Auto-flip for red alliance

        // if it is autonomous, assume that we start in the correct initial position

        // AdvantageKit logging
        FollowPath.setDoubleLoggingConsumer(p -> Logger.recordOutput(p.getFirst(), p.getSecond()));
        FollowPath.setBooleanLoggingConsumer(p -> Logger.recordOutput(p.getFirst(), p.getSecond()));
        FollowPath.setPoseLoggingConsumer(p -> Logger.recordOutput(p.getFirst(), p.getSecond()));
        FollowPath.setTranslationListLoggingConsumer(p -> Logger.recordOutput(p.getFirst(), p.getSecond()));

        antiTipping = new AntiTipping(
                gyroIO::getGyroRollAngle,
                gyroIO::getGyroPitchAngle,
                0.04, // kP
                2.0, // tipping threshold (degrees) //3
                1.5 // max correction speed (m/s) //2.5
        );

    }

    public FollowPath.Builder getBLineBuilder() {
        return bLinePathBuilder;
    }

    public SwerveModuleState[] getModuleStates() {

        SwerveModuleState[] states = new SwerveModuleState[moduleIO.length];
        for (int i = 0; i < moduleIO.length; i++) {
            states[i] = moduleIO[i].getState();
        }

        return states;
    }

    @Override
    public void periodic() {

        antiTipping.calculate();

        updateOdometry();

        Logger.recordOutput("Module states", getModuleStates());

        Logger.recordOutput("Gyro", gyroIO.getGyroYawAngle());

        Logger.recordOutput("DriveSubsystem/CurrentFieldZone", CowboyUtils.getFieldZoneFromPose(getPose()).name());

        for (int i = 0; i < moduleIO.length; i++) {
            moduleIO[i].updateInputs(moduleInputs[i]);
            // e.g. produces “DriveModule0”, “DriveModule1”, etc.
            // FL, FR, RL, RR
            Logger.processInputs("DriveSubsystem/DriveModule" + i, moduleInputs[i]);
        }
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("DriveSubsystem/Gyro", gyroInputs);

        updateOdometrySensorMeasurements();

    }

    private void updateOdometry() {
        // Update the odometry (Called in periodic)

        if (RobotBase.isSimulation() && SimMode.SIM_MODE == SimMode.SimModes.REGULAR) {
            m_trackedRotation = m_trackedRotation.plus(new Rotation2d(
                    DriveSubsystemConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond
                            * ModuleIOSim.getPeriodicRate()));
            gyroIO.setGyroAngle(m_trackedRotation.getDegrees());
        }

        hybridOdometry.update(
                gyroIO.getGyroYawRotation2d(),
                new SwerveModulePosition[] {
                        moduleIO[0].getPosition(),
                        moduleIO[1].getPosition(),
                        moduleIO[2].getPosition(),
                        moduleIO[3].getPosition()
                });

        Logger.recordOutput("DriveSubsystem/OdometryPoseHybrid", hybridOdometry.getEstimatedPosition());
        RobotState.robotPose = hybridOdometry.getEstimatedPosition();
    }

    private void updateOdometrySensorMeasurements() {

        // PV poses:

        TimestampedPose timestampedPose;
        while ((timestampedPose = RobotState.getAprilTagCameraMeasurments().poll()) != null) {
            hybridOdometry.addVisionMeasurement(
                    timestampedPose.pose(), timestampedPose.timestamp());
        }

        // QuestNav poses:

        if (DriverStation.isEnabled()) {

            while ((timestampedPose = RobotState.getQuestMeasurments().poll()) != null) {
                hybridOdometry.addVisionMeasurement(
                        timestampedPose.pose(), timestampedPose.timestamp(),
                        QuestNavSubsystemConstants.QUESTNAV_STD_DEVS);
            }
        }
    }

    public Pose2d getPose() {
        return hybridOdometry.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        hybridOdometry.resetPosition(
                gyroIO.getGyroYawRotation2d(),
                new SwerveModulePosition[] {
                        moduleIO[0].getPosition(),
                        moduleIO[1].getPosition(),
                        moduleIO[2].getPosition(),
                        moduleIO[3].getPosition()
                },
                pose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit,
            boolean antiTippingEnabled) {

        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimit) {
            // Convert XY to polar for rate limiting
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            // Calculate the direction slew rate based on an estimate of the lateral
            // acceleration
            double directionSlewRate;

            if (m_currentTranslationMag != 0.0) {
                directionSlewRate = Math.abs(DriveSubsystemConstants.directionSlewRate() / m_currentTranslationMag);
            } else {
                directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - m_prevTime;
            double angleDif = SwerveUtils.angleDifference(inputTranslationDir, m_currentTranslationDir);

            if (angleDif < 0.45 * Math.PI) {
                m_currentTranslationDir = SwerveUtils.stepTowardsCircular(m_currentTranslationDir,
                        inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
            } else if (angleDif > 0.85 * Math.PI) {
                if (m_currentTranslationMag > 1e-4) {
                    m_currentTranslationMag = m_magLimiter.calculate(0.0);
                } else {
                    m_currentTranslationDir = SwerveUtils.wrapAngle(m_currentTranslationDir + Math.PI);
                    m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                }
            } else {
                m_currentTranslationDir = SwerveUtils.stepTowardsCircular(m_currentTranslationDir,
                        inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(0.0);
            }

            m_prevTime = currentTime;

            xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
            ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
            m_currentRotation = m_rotLimiter.calculate(rot);

        } else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
            m_currentRotation = rot;
        }

        // if (Robot.isSimulation() && CowboyUtils.isRedAlliance()) {
        // xSpeedCommanded = -xSpeedCommanded; // Away from the DS
        // ySpeedCommanded = -ySpeedCommanded; // Away from long wall
        // m_currentRotation = -m_currentRotation;
        // }

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * DriveSubsystemConstants.MAX_SPEED_METERS_PER_SECOND;
        double ySpeedDelivered = ySpeedCommanded * DriveSubsystemConstants.MAX_SPEED_METERS_PER_SECOND;
        double rotDelivered = m_currentRotation * DriveSubsystemConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

        ChassisSpeeds speeds;
        // if (antiTipping.isTipping() && antiTippingEnabled) {
        // ChassisSpeeds ogSpeeds = antiTipping.getVelocityAntiTipping();
        // speeds = ogSpeeds;// new ChassisSpeeds(ogSpeeds.vyMetersPerSecond,
        // ogSpeeds.vxMetersPerSecond,
        // // ogSpeeds.omegaRadiansPerSecond);
        // } else {
        speeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
        // }

        Rotation2d rotation = gyroIO.getGyroYawRotation2d();

        var swerveModuleStates = DriveSubsystemConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rotation)
                        : speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveSubsystemConstants.MAX_SPEED_METERS_PER_SECOND);

        moduleIO[0].setDesiredState(swerveModuleStates[0]);
        moduleIO[1].setDesiredState(swerveModuleStates[1]);
        moduleIO[2].setDesiredState(swerveModuleStates[2]);
        moduleIO[3].setDesiredState(swerveModuleStates[3]);
    }

    public void runChassisSpeeds(ChassisSpeeds speeds) {
        runChassisSpeeds(speeds, false);
    }

    public void runChassisSpeeds(ChassisSpeeds speeds, Boolean fieldRelative) {
        Rotation2d rotation = Rotation2d.fromDegrees(gyroIO.getGyroYawAngle());

        var swerveModuleStates = DriveSubsystemConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rotation)
                        : speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveSubsystemConstants.MAX_SPEED_METERS_PER_SECOND);

        moduleIO[0].setDesiredState(swerveModuleStates[0]);
        moduleIO[1].setDesiredState(swerveModuleStates[1]);
        moduleIO[2].setDesiredState(swerveModuleStates[2]);
        moduleIO[3].setDesiredState(swerveModuleStates[3]);

    }

    public ChassisSpeeds getTheoreticalSpeeds(double xSpeed, double ySpeed, double rot, boolean fieldRelative,
            boolean rateLimit) {
        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimit) {
            // Convert XY to polar for rate limiting
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            // Calculate the direction slew rate based on an estimate of the lateral
            // acceleration
            double directionSlewRate;

            if (m_currentTranslationMag != 0.0) {
                directionSlewRate = Math.abs(DriveSubsystemConstants.directionSlewRate() / m_currentTranslationMag);
            } else {
                directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - m_prevTime;
            double angleDif = SwerveUtils.angleDifference(inputTranslationDir, m_currentTranslationDir);

            if (angleDif < 0.45 * Math.PI) {
                m_currentTranslationDir = SwerveUtils.stepTowardsCircular(m_currentTranslationDir,
                        inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
            } else if (angleDif > 0.85 * Math.PI) {
                if (m_currentTranslationMag > 1e-4) {
                    m_currentTranslationMag = m_magLimiter.calculate(0.0);
                } else {
                    m_currentTranslationDir = SwerveUtils.wrapAngle(m_currentTranslationDir + Math.PI);
                    m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                }
            } else {
                m_currentTranslationDir = SwerveUtils.stepTowardsCircular(m_currentTranslationDir,
                        inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(0.0);
            }

            m_prevTime = currentTime;

            xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
            ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
            m_currentRotation = m_rotLimiter.calculate(rot);

        } else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
            m_currentRotation = rot;
        }

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * DriveSubsystemConstants.MAX_SPEED_METERS_PER_SECOND;
        double ySpeedDelivered = ySpeedCommanded * DriveSubsystemConstants.MAX_SPEED_METERS_PER_SECOND;
        double rotDelivered = m_currentRotation * DriveSubsystemConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

        Rotation2d rotation = gyroIO.getGyroYawRotation2d();

        return !fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                        rotation)
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        moduleIO[0].setDesiredState(new SwerveModuleState(0,
                Rotation2d.fromDegrees(45)));
        moduleIO[1].setDesiredState(new SwerveModuleState(0,
                Rotation2d.fromDegrees(-45)));
        moduleIO[2].setDesiredState(new SwerveModuleState(0,
                Rotation2d.fromDegrees(-45)));
        moduleIO[3].setDesiredState(new SwerveModuleState(0,
                Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveSubsystemConstants.MAX_SPEED_METERS_PER_SECOND);
        moduleIO[0].setDesiredState(desiredStates[0]);
        moduleIO[1].setDesiredState(desiredStates[1]);
        moduleIO[2].setDesiredState(desiredStates[2]);
        moduleIO[3].setDesiredState(desiredStates[3]);

    }

    /**
     * Resets the drive encoders to currently read a position of 0 and sets the
     * turn encoders using the absolute encoders.
     */
    public void resetEncoders() {

        moduleIO[0].resetEncoders();
        moduleIO[2].resetEncoders();
        moduleIO[1].resetEncoders();
        moduleIO[3].resetEncoders();

    }

    public Command resetEncodersCommand() {
        return new InstantCommand(() -> resetEncoders(), this);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyroIO.reset();
    }

    /**
     * Returns the yaw heading of the robot.
     * 
     * @return the robot's heading in degrees, from -180 to 180
     */
    public Double getHeading() {
        return gyroIO.getGyroYawAngle();
    }

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        SwerveModuleState[] states = getModuleStates();
        return ChassisSpeeds.fromFieldRelativeSpeeds(DriveSubsystemConstants.DRIVE_KINEMATICS.toChassisSpeeds(states),
                gyroIO.getGyroYawRotation2d());
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        SwerveModuleState[] states = getModuleStates();
        return DriveSubsystemConstants.DRIVE_KINEMATICS.toChassisSpeeds(states);
    }

    public Command gyroReset() {
        return Commands.startEnd(() -> {
            gyroIO.reset();
        }, () -> {
            // end
        });
    }

    public Command xCommand() {
        return new InstantCommand(() -> {
            // init
            RobotState.xLocked = !RobotState.xLocked;
        });

    }

}