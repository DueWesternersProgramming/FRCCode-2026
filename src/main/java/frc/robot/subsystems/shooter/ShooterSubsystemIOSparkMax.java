package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.RobotConstants.PortConstants;
import frc.robot.RobotConstants.SwerveModuleConstants;
import frc.robot.RobotState.ShooterStates;

public class ShooterSubsystemIOSparkMax implements ShooterSubsystemIO {
    SparkMax shooterMotor;
    SparkMaxConfig shooterMotorConfig;
    SparkClosedLoopController closedLoopController;

    public ShooterSubsystemIOSparkMax() {
    shooterMotor = new SparkMax(PortConstants.CAN.SHOOTER_MOTOR, MotorType.kBrushless);
    shooterMotorConfig = new SparkMaxConfig();
    closedLoopController = shooterMotor.getClosedLoopController();

    shooterMotorConfig.smartCurrentLimit(30)
                      .idleMode(IdleMode.kCoast);

    // kV is often roughly (1.0 / Max RPM). For a Neo, ~0.00017 is a starting point.
    FeedForwardConfig shooterFF = new FeedForwardConfig();
                shooterFF.kV(0.00017);
    shooterMotorConfig.closedLoop.apply(shooterFF);

    shooterMotor.configure(shooterMotorConfig, 
                           ResetMode.kResetSafeParameters, 
                           PersistMode.kNoPersistParameters);
}

    @Override
    public void setRMP(double rpm){
        closedLoopController.setSetpoint(rpm, ControlType.kVelocity);
    }

    @Override
    public void setPercentSpeed(double speed){
        closedLoopController.setSetpoint(speed, ControlType.kDutyCycle);
    }

    @Override
    public void updateInputs(ShooterSubsystemIOInputs inputs) {
        inputs.motorPercent = shooterMotor.getAppliedOutput();
        inputs.motorRMP = shooterMotor.getEncoder().getVelocity();
        inputs.motorTempC = shooterMotor.getMotorTemperature();
        inputs.shooterState = ShooterStates.OFF;
    }
}
