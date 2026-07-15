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
    SparkMax floorRollersSparkMax;
    SparkMax verticalRollersSparkMax;

    SparkMaxConfig sparkMaxConfig;


    double floorRollersPercent = 0;
    double verticalRollersPercent = 0;

    public FeederSubsystemIOSparkMax() {
        floorRollersSparkMax = new SparkMax(PortConstants.CAN.FLOOR_ROLLERS_MOTOR, MotorType.kBrushless);
        verticalRollersSparkMax = new SparkMax(PortConstants.CAN.VERTICAL_ROLLERS_MOTOR, MotorType.kBrushless);


        sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.smartCurrentLimit(40).idleMode(IdleMode.kCoast);
        
        floorRollersSparkMax.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        verticalRollersSparkMax.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setFloorRollersPercentSpeed(double percent) {
        floorRollersSparkMax.set(percent);
        floorRollersPercent = percent;
    }

    @Override
    public void setVerticalRollersPercentSpeed(double percent) {
        verticalRollersSparkMax.set(percent);
        verticalRollersPercent = percent;
    }

    @Override
    public void updateInputs(FeederSubsystemIOInputs inputs) {
        inputs.floorRollersPercent = floorRollersPercent;
        inputs.verticalRollersPercent = verticalRollersPercent;

        inputs.floorRollersMotorRPM = floorRollersSparkMax.getEncoder().getVelocity();
        inputs.verticalRollersMotorRPM = verticalRollersSparkMax.getEncoder().getVelocity();

        inputs.floorRollersMotorTempC = floorRollersSparkMax.getMotorTemperature();
        inputs.verticalRollersMotorTempC = verticalRollersSparkMax.getMotorTemperature();
    }
}
