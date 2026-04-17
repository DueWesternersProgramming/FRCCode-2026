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
    SparkMax leftShooterMotor;
    SparkMax rightShooterMotor;
    SparkMaxConfig leftShooterMotorConfig;
    SparkMaxConfig rightShooterMotorConfig;
    SparkClosedLoopController closedLoopController;

    public ShooterSubsystemIOSparkMax() {
        leftShooterMotor = new SparkMax(PortConstants.CAN.LEFT_SHOOTER_MOTOR, MotorType.kBrushless);
        rightShooterMotor = new SparkMax(PortConstants.CAN.RIGHT_SHOOTER_MOTOR, MotorType.kBrushless);

        leftShooterMotorConfig = new SparkMaxConfig();
        rightShooterMotorConfig = new SparkMaxConfig();

        leftShooterMotorConfig.smartCurrentLimit(40)
                .idleMode(IdleMode.kCoast).inverted(true);

        closedLoopController = leftShooterMotor.getClosedLoopController();

        leftShooterMotorConfig.closedLoop
                .pid(0.5, 0, 0)
                .outputRange(-1, 1);

        leftShooterMotorConfig.closedLoop.feedForward.kS(0).kV(0);

        leftShooterMotorConfig.closedLoop.maxMotion
                .maxAcceleration(4000)
                .cruiseVelocity(6500)
                .allowedProfileError(25);

        rightShooterMotorConfig.smartCurrentLimit(40)
                .idleMode(IdleMode.kCoast).follow(leftShooterMotor, true);

        leftShooterMotor.configure(leftShooterMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);

        rightShooterMotor.configure(rightShooterMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    @Override
    public void setRPM(double rpm) {
        closedLoopController.setSetpoint(rpm, ControlType.kMAXMotionVelocityControl);
    }

    @Override
    public void setPercentSpeed(double speed) {
        closedLoopController.setSetpoint(speed, ControlType.kDutyCycle);
    }

    @Override
    public void updateInputs(ShooterSubsystemIOInputs inputs) {
        inputs.leftMotorRPM = leftShooterMotor.getEncoder().getVelocity();
        inputs.leftMotorTempC = leftShooterMotor.getMotorTemperature();
        inputs.leftMotorCurrentDraw = leftShooterMotor.getOutputCurrent();

        inputs.rightMotorRPM = rightShooterMotor.getEncoder().getVelocity();
        inputs.rightMotorTempC = rightShooterMotor.getMotorTemperature();
        inputs.rightMotorCurrentDraw = rightShooterMotor.getOutputCurrent();
    }
}
