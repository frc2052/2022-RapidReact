package frc.robot.subsystems;

import java.lang.reflect.Method;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import frc.robot.Constants;
import frc.robot.util.HsvToRgb;

public class LEDSubsystem {

    // This is a singleton pattern for making sure only 1 instance of this class exists that can be called from anywhere.
    private LEDSubsystem() {}
    private static LEDSubsystem instance;
    public static LEDSubsystem getInstance() {
        if (instance == null) {
            instance = new LEDSubsystem();
        }
        return instance;
    }

    private final CANifier canifer = new CANifier(Constants.LEDs.kCANiferPort);
    private double [] rgb = new double[3];

    private double saturation = 1;
    private double hue = 0;
    private double value = 1;

    private boolean useHSV;

    private LEDStatusMode currentLEDStatusMode = LEDStatusMode.RAINBOW;

    public void onLoop() {
        runLEDStatusMode();

        if (saturation > 1) {
            saturation = 1;
        } else if (saturation < 0) {
            saturation = 0;
        }

        if (value > 1) {
            value = 1;
        } else if (value < 0) {
            value = 0;
        }

        //Converts HSV (Hue Saturation Value) to RGB (Red Green Blue)
        if (useHSV) {
            rgb = HsvToRgb.convert(hue, saturation, value);
        }

        canifer.setLEDOutput(rgb[0], LEDChannel.LEDChannelA);
        canifer.setLEDOutput(rgb[1], LEDChannel.LEDChannelB);
        canifer.setLEDOutput(rgb[2], LEDChannel.LEDChannelC);
    }

    public void setRgb(double red, double green, double blue) {
        rgb[0] = green;
        rgb[1] = red;
        rgb[2] = blue;
    }

    public void setLEDStatusMode(LEDStatusMode statusMode) {
        currentLEDStatusMode = statusMode;
    }

    public void runLEDStatusMode() {
        switch (currentLEDStatusMode) {
            case RAINBOW:
                rainbowStatusMode();
                break;
            case OFF:
                LEDsOffStatusMode();
                break;
            default:
                break;
        }
    }

    public void rainbowStatusMode() {
        useHSV = true;
        hue += 1;
        saturation = 1;
        value = 1;

        if (hue >= 360) {
            hue = 0;
        }
    }

    public void LEDsOffStatusMode() {
        useHSV = false;
        rgb[0] = 0;
        rgb[1] = 0;
        rgb[2] = 0;
    }

    public enum LEDStatusMode {
        RAINBOW("Rainbow"),
        OFF("Off"),
        SOLID_RED("Solid Red"),
        SOLID_WHITE("Solid White"),
        AUTONOMOUS_DEFAULT("Autonomous Default"),
        AUTONOMOUS_FINISHED("Autonomous Finished"),
        TELEOP_DEFAULT("Teleop Default"),
        VISION_TARGETING("Vision Targeting"),
        VISION_TARGET_FOUND("Vision Target Found"),
        CLIMBING_DEFAULT("Climbing Default"),
        CLIMBING_LOW_BAR("Climbing Low Bar"),
        CLIMBING_MID_BAR("Climbing Middle Bar"),
        CLIMBING_HIGH_BAR("Climbing High Bar"),
        CLIMBING_TRAVERSAL("Climbing Traversal Bar");

        public String name;

        LEDStatusMode(String name) {
            this.name = name;
        }
    }
}
