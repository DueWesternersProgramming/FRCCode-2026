package frc.robot.subsystems.led;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.RobotConstants.LEDConstants.AnimationTypes;
import frc.robot.RobotState;

public class LEDSubsystemIOSim implements LEDSubsystemIO {
    public LEDSubsystemIOSim(){
    }

    @Override
    public void updateInputs(LEDSubsystemIOInputs inputs) {

    }

    @Override
    public void setAnimation(AnimationTypes animation) {
        //System.out.println("Setting LEDs to: " +  animation);
    }
}