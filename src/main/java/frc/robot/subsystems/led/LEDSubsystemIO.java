package frc.robot.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.RGBWColor;

import frc.robot.subsystems.led.LEDSubsystemConstants.AnimationTypes;


public interface LEDSubsystemIO {

    @AutoLog
    public static class LEDSubsystemIOInputs {
        public double temp = 0.0;
        public AnimationTypes currentAnimation = AnimationTypes.NONE;
    }

    default void updateInputs(LEDSubsystemIOInputs inputs) {
    }

    default public void setAnimation(AnimationTypes animation, RGBWColor color){}


}
