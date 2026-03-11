package frc.robot.subsystems.feeder;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import frc.robot.RobotConstants.PortConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class FeederSubsystemIOSparkMax implements FeederSubsystemIO {
    SparkMax feederMotor;

    SparkMaxConfig feederMotorConfig;

    public FeederSubsystemIOSparkMax() {
        feederMotor = new SparkMax(PortConstants.CAN.FEEDER_MOTOR, MotorType.kBrushless);

        feederMotorConfig = new SparkMaxConfig();
        feederMotorConfig.smartCurrentLimit(30).idleMode(IdleMode.kCoast);
        feederMotor.configure(feederMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setFeederPercentSpeed(double percent) {
        feederMotor.set(percent);
    }

    @Override
    public void updateInputs(FeederSubsystemIOInputs inputs) {
        inputs.feederMotorRPM = feederMotor.getEncoder().getVelocity();
        inputs.feederMotorTempC = feederMotor.getMotorTemperature();

    }
}
