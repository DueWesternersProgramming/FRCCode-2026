package frc.robot.subsystems.led;

import com.ctre.phoenix6.signals.RGBWColor;

import frc.robot.RobotConstants.LEDConstants.AnimationTypes;

public class LEDSubsystemIOSim implements LEDSubsystemIO {
    private AnimationTypes currentAnimation = null;

    public LEDSubsystemIOSim() {
    }

    @Override
    public void updateInputs(LEDSubsystemIOInputs inputs) {
        inputs.currentAnimation = this.currentAnimation;
    }

    @Override
    public void setAnimation(AnimationTypes animation, RGBWColor color) {
        currentAnimation = animation;
    }
}