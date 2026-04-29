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
        private SparkMax leftShooterMotor;
        private SparkMax rightShooterMotor;
        private SparkMaxConfig leftShooterMotorConfig;
        private SparkMaxConfig rightShooterMotorConfig;
        private SparkClosedLoopController closedLoopController;

        public ShooterSubsystemIOSparkMax() {
                leftShooterMotor = new SparkMax(
                                PortConstants.CAN.LEFT_SHOOTER_MOTOR,
                                MotorType.kBrushless);

                rightShooterMotor = new SparkMax(
                                PortConstants.CAN.RIGHT_SHOOTER_MOTOR,
                                MotorType.kBrushless);

                leftShooterMotorConfig = new SparkMaxConfig();
                rightShooterMotorConfig = new SparkMaxConfig();

                // LEFT MOTOR CONFIG
                leftShooterMotorConfig
                                .smartCurrentLimit(40)
                                .idleMode(IdleMode.kCoast)
                                .inverted(true);

                leftShooterMotorConfig.closedLoop
                                .pid(0.00005, 0.0, 0.0)
                                .outputRange(-1.0, 1.0);

                leftShooterMotorConfig.closedLoop.feedForward
                                .kV(0.000165);

                // RIGHT MOTOR CONFIG (follower)
                rightShooterMotorConfig
                                .smartCurrentLimit(40)
                                .idleMode(IdleMode.kCoast)
                                .follow(leftShooterMotor, true); // inverted follower

                // Apply configs
                leftShooterMotor.configure(
                                leftShooterMotorConfig,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kNoPersistParameters);

                rightShooterMotor.configure(
                                rightShooterMotorConfig,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kNoPersistParameters);

                closedLoopController = leftShooterMotor.getClosedLoopController();
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
                double current = Math.abs(leftShooterMotor.getEncoder().getVelocity());

                return (current >= target - 100) && (current <= target + 100);
        }

        @Override
        public void updateInputs(ShooterSubsystemIOInputs inputs) {
                inputs.leftMotorRPM = leftShooterMotor.getEncoder().getVelocity();
                inputs.leftMotorTempC = leftShooterMotor.getMotorTemperature();
                inputs.leftMotorCurrentDraw = leftShooterMotor.getOutputCurrent();

                inputs.rightMotorRPM = rightShooterMotor.getEncoder().getVelocity();
                inputs.rightMotorTempC = rightShooterMotor.getMotorTemperature();
                inputs.rightMotorCurrentDraw = rightShooterMotor.getOutputCurrent();

                inputs.reachedTargetVelocity = hasReachedTargetVelocity();
        }
}