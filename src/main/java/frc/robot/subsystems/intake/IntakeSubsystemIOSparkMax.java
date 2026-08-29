package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import frc.robot.RobotConstants.PortConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeSubsystemIOSparkMax implements IntakeSubsystemIO {
    SparkMax intakeMotor;
    SparkMax deploymentMotor;

    SparkMaxConfig intakeMotorConfig;

    public IntakeSubsystemIOSparkMax() {
        intakeMotor = new SparkMax(PortConstants.CAN.INTAKE_MOTOR, MotorType.kBrushless);

        SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
        intakeMotorConfig.smartCurrentLimit(30).idleMode(IdleMode.kCoast).inverted(true);
        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setPercentSpeed(double percent) {
        intakeMotor.set(percent);
    }

    @Override
    public double getIntakeCurrent() {
        return intakeMotor.getOutputCurrent();
    }

    @Override
    public double getIntakeRPM() {
        return intakeMotor.getEncoder().getVelocity();
    }

    @Override
    public boolean isIntakeStalled() {
        return intakeMotor.getOutputCurrent() > IntakeSubsystemConstants.INTAKE_MOTOR_STALL_CURRENT || intakeMotor.getEncoder().getVelocity() < IntakeSubsystemConstants.INTAKE_MOTOR_STALL_RPM;
    }

    @Override
    public void updateInputs(IntakeSubsystemIOInputs inputs) {
        inputs.intakePercent = intakeMotor.getAppliedOutput();
        inputs.intakeTempC = intakeMotor.getMotorTemperature();
        inputs.intakeRPM = intakeMotor.getEncoder().getVelocity();
        inputs.intakeCurrent = intakeMotor.getOutputCurrent();
        inputs.intakeStalled = isIntakeStalled();
    }
}