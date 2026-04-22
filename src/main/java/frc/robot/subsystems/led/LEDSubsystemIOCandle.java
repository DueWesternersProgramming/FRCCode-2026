package frc.robot.subsystems.led;

import org.w3c.dom.css.RGBColor;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import frc.robot.RobotConstants.LEDConstants;
import frc.robot.RobotConstants.PortConstants;
import frc.robot.RobotConstants.LEDConstants.AnimationTypes;

public class LEDSubsystemIOCandle implements LEDSubsystemIO {
    CANdle candle;
    CANdleConfiguration candleConfig;
    public boolean runningAnimation;
    private AnimationTypes currentAnimation = AnimationTypes.NONE;

    public LEDSubsystemIOCandle() {
        candle = new CANdle(PortConstants.CAN.CANDLE, "rio");
        candleConfig = new CANdleConfiguration();
        candleConfig.LED.StripType = StripTypeValue.RGB;

        candleConfig.LED.BrightnessScalar = .5;
        candleConfig.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Enabled;

        candle.getConfigurator().apply(candleConfig);
        runningAnimation = false;

        for (int i = 0; i < 8; ++i) {
            candle.setControl(new EmptyAnimation(i));
        }
    }

    @Override
    public void setAnimation(AnimationTypes animation, RGBWColor color) {
        currentAnimation = animation;
        switch (animation) {
            default:
            case ColorFlow:
                candle.setControl(
                        new ColorFlowAnimation(0, 100).withSlot(0)
                                .withColor(color));
                break;
            case Rainbow:
                candle.setControl(
                        new RainbowAnimation(0, 100).withSlot(0));
                break;
            case Twinkle:
                candle.setControl(
                        new TwinkleAnimation(0, 100).withSlot(0)
                                .withColor(color));
                break;
            case TwinkleOff:
                candle.setControl(
                        new TwinkleOffAnimation(0, 100).withSlot(0)
                                .withColor(color));
                break;
            case Fire:
                candle.setControl(
                        new FireAnimation(0, 100).withSlot(0));
                break;
            case SetAll:
                candle.setControl(new SolidColor(9, 69).withColor(color));
        }
    }


    @Override
    public void updateInputs(LEDSubsystemIOInputs inputs) {
        inputs.currentAnimation = currentAnimation;
        inputs.temp = candle.getDeviceTemp().getValueAsDouble();
    }
}
