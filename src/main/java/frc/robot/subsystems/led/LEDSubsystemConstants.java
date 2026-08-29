package frc.robot.subsystems.led;

import com.ctre.phoenix6.signals.RGBWColor;

public class LEDSubsystemConstants {

                public static enum LEDStatus {
                        IDLE,
                        INTAKING,
                        INTAKE_STALLED,
                        SHOOTING,
                        REVERSING
                }

                public static final int LED_COUNT = 300;

                public static enum AnimationTypes {
                        ColorFlow,
                        Fire,
                        Larson,
                        Rainbow,
                        RgbFade,
                        SingleFade,
                        Strobe,
                        Twinkle,
                        TwinkleOff,
                        SetAll,
                        NONE
                }

                public static final RGBWColor ANIMATION_COLOR = new RGBWColor(255, 0, 0);
        }
