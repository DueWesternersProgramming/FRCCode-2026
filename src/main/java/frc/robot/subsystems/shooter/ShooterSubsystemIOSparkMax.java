package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.RobotConstants.PortConstants;

public class ShooterSubsystemIOSparkMax implements ShooterSubsystemIO {
    SparkMax shooterMotor;
    SparkMaxConfig shooterMotorConfig;
    SparkClosedLoopController closedLoopController;

    public ShooterSubsystemIOSparkMax(){
        shooterMotor = new SparkMax(PortConstants.CAN.SHOOTER_MOTOR, MotorType.kBrushless);
        shooterMotorConfig = new SparkMaxConfig();
        shooterMotorConfig.closedLoop.pid(.3, 0, 0);

        closedLoopController = shooterMotor.getClosedLoopController();
    }
//Setting the RPM point and making labeling that it is a control that affects velocity
    @Override
    public void setRMP(double rpm){
        closedLoopController.setSetpoint(rpm, ControlType.kVelocity);
    }

    @Override
    public void setPercentSpeed(double speed){
        closedLoopController.setSetpoint(speed, ControlType.kDutyCycle);
    }
//Gets the motor rpm and temperature settings
    @Override
    public void updateInputs(ShooterSubsystemIOInputs inputs) {
        inputs.motorRMP = shooterMotor.getEncoder().getVelocity();
        inputs.motorTempC = shooterMotor.getMotorTemperature();
    }
}
