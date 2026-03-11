package frc.robot.subsystems.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import frc.robot.RobotConstants.PortConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IndexerSubsystemIOSparkMax implements IndexerSubsystemIO {
    SparkMax conveyorMotor;
    SparkMax rollerMotor;

    SparkMaxConfig conveyerMotorConfig;
    SparkMaxConfig rollerMotorConfig;

    public IndexerSubsystemIOSparkMax() {
        conveyorMotor = new SparkMax(PortConstants.CAN.CONVEYOR_MOTOR, MotorType.kBrushless);
        rollerMotor = new SparkMax(PortConstants.CAN.INDEXER_MOTOR, MotorType.kBrushless);

        conveyerMotorConfig = new SparkMaxConfig();
        conveyerMotorConfig.smartCurrentLimit(30).idleMode(IdleMode.kCoast);
        conveyorMotor.configure(conveyerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rollerMotorConfig = new SparkMaxConfig();
        rollerMotorConfig.smartCurrentLimit(30).idleMode(IdleMode.kCoast);
        rollerMotor.configure(conveyerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setConveyorPercentSpeed(double percent) {
        conveyorMotor.set(percent);
    }

    @Override
    public void setRollerPercentSpeed(double percent) {
        rollerMotor.set(percent);
    }

    @Override
    public void updateInputs(IndexerSubsystemIOInputs inputs) {
        inputs.conveyorMotorPercent = conveyorMotor.getAppliedOutput();
        inputs.conveyorMotorRMP = conveyorMotor.getEncoder().getVelocity();
        inputs.conveyorMotorTempC = conveyorMotor.getMotorTemperature();

        inputs.indexerMotorPercent = rollerMotor.getAppliedOutput();
        inputs.indexerMotorRPM = rollerMotor.getEncoder().getVelocity();
        inputs.indexerMotorTempC = rollerMotor.getMotorTemperature();


    }
}
