package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.RobotConstants.PortConstants;

public class ShooterSubsystemIOSparkMax implements ShooterSubsystemIO {
        private SparkMax leftShooterMotor1;
        private SparkMax leftShooterMotor2;
        private SparkMax rightShooterMotor;
        private SparkMax rightShooterMotor2;
        private SparkMaxConfig driverConfig;
        private SparkMaxConfig followerConfig;
        private SparkClosedLoopController closedLoopController;

        public ShooterSubsystemIOSparkMax() {
                leftShooterMotor1 = new SparkMax(
                                PortConstants.CAN.LEFT_SHOOTER_MOTOR1,
                                MotorType.kBrushless);
                leftShooterMotor2 = new SparkMax(
                                PortConstants.CAN.LEFT_SHOOTER_MOTOR2,
                                MotorType.kBrushless);

                rightShooterMotor = new SparkMax(
                                PortConstants.CAN.RIGHT_SHOOTER_MOTOR1,
                                MotorType.kBrushless);
                rightShooterMotor2 = new SparkMax(
                                PortConstants.CAN.RIGHT_SHOOTER_MOTOR2,
                                MotorType.kBrushless);

                driverConfig = new SparkMaxConfig();
                followerConfig = new SparkMaxConfig();

                // LEFT MOTOR CONFIG
                driverConfig
                                .smartCurrentLimit(40)
                                .idleMode(IdleMode.kCoast)
                                .inverted(false);

                driverConfig.closedLoop
                                .pid(ShooterConstants.P, ShooterConstants.I, ShooterConstants.D)
                                .outputRange(-1.0, 1.0);

                driverConfig.closedLoop.feedForward
                                .kV(ShooterConstants.FF);

                followerConfig
                                .smartCurrentLimit(40)
                                .idleMode(IdleMode.kCoast)
                                .follow(leftShooterMotor1, true); // inverted follower

                // Apply configs
                leftShooterMotor1.configure(
                                driverConfig,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kNoPersistParameters);

                rightShooterMotor.configure(
                                followerConfig,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kNoPersistParameters);

                rightShooterMotor2.configure(
                                followerConfig,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kNoPersistParameters);
                leftShooterMotor2.configure(
                                followerConfig.follow(leftShooterMotor1, false),
                                ResetMode.kResetSafeParameters,
                                PersistMode.kNoPersistParameters);

                closedLoopController = leftShooterMotor1.getClosedLoopController();
        }

        @Override
        public void setRPM(double rpm) {
                closedLoopController.setSetpoint(rpm, ControlType.kVelocity);
        }

        @Override
        public void setPercentSpeed(double speed) {
                closedLoopController.setSetpoint(speed, ControlType.kDutyCycle);
        }

        @Override
        public void setVoltage(double volts) {
                closedLoopController.setSetpoint(volts, ControlType.kVoltage);
        }

        public boolean hasReachedTargetVelocity() {
                double target = closedLoopController.getSetpoint();
                double current = Math.abs(leftShooterMotor1.getEncoder().getVelocity());

                return (current >= target - 100) && (current <= target + 100);
        }

        @Override
        public void updateInputs(ShooterSubsystemIOInputs inputs) {
                inputs.leftMotor1RPM = leftShooterMotor1.getEncoder().getVelocity();
                inputs.leftMotor1TempC = leftShooterMotor1.getMotorTemperature();
                inputs.leftMotor1CurrentDraw = leftShooterMotor1.getOutputCurrent();

                inputs.leftMotor2RPM = leftShooterMotor2.getEncoder().getVelocity();
                inputs.leftMotor2TempC = leftShooterMotor2.getMotorTemperature();
                inputs.leftMotor2CurrentDraw = leftShooterMotor2.getOutputCurrent();

                inputs.rightMotor1RPM = rightShooterMotor.getEncoder().getVelocity();
                inputs.rightMotor1TempC = rightShooterMotor.getMotorTemperature();
                inputs.rightMotor1CurrentDraw = rightShooterMotor.getOutputCurrent();

                inputs.rightMotor2RPM = rightShooterMotor2.getEncoder().getVelocity();
                inputs.rightMotor2TempC = rightShooterMotor2.getMotorTemperature();
                inputs.rightMotor2CurrentDraw = rightShooterMotor2.getOutputCurrent();
        }
}