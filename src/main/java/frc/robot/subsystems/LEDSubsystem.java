package frc.robot.subsystems;

import java.lang.reflect.Method;
import java.util.Timer;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.HsvToRgb;

public class LEDSubsystem extends SubsystemBase {

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

    private long lastOnChangeTime = 0;
    private boolean areLedsOn = false;

    private boolean isGoingUp = true;
//    private long timer = 0;

    private LEDStatusMode currentLEDStatusMode = LEDStatusMode.CLIMBING_TRAVERSAL;

    @Override
    public void periodic() {

        //System.out.println(timer);

        runLEDStatusMode();

        if (useHSV) {
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

    private void runLEDStatusMode() {
        switch (currentLEDStatusMode) {
            case RAINBOW:
                rainbowStatusMode();
                break;
            case OFF:
                offStatusMode();
                break;
            case BLINK_RED:
                blinkingRedStatusMode();
                break;
            case VISION_TARGET_FOUND:
                visionTargetFoundStatusMode();
                break;
            case CLIMBING_DEFAULT:
                fadeInOutWhite();
                break;
            case CLIMBING_TRAVERSAL:
                fadeInOutWhiteTraversalBar();
                break;
            default:
                break;
        }
    }

    private void rainbowStatusMode() {
        useHSV = true;
        hue += 1;
        saturation = 1;
        value = 1;

        if (hue >= 360) {
            hue = 0;
        }
    }

    private void offStatusMode() {
        useHSV = false;
        rgb[0] = 0;
        rgb[1] = 0;
        rgb[2] = 0;
    }

    private void blinkingRedStatusMode() {
        useHSV = false;
        evaluateOnOffInterval(2000, 1000);
        if (areLedsOn) {
            rgb[0] = 0;
            rgb[1] = 1;
            rgb[2] = 0;
        } else {
            rgb[0] = 0;
            rgb[1] = 0;
            rgb[2] = 0;
        }
    }

    private void visionTargetFoundStatusMode() {
        useHSV = false;
        evaluateOnOffInterval(500, 500);
        if (areLedsOn) {
            rgb[0] = 1;
            rgb[1] = 0;
            rgb[2] = 0;
        } else {
            rgb[0] = 0;
            rgb[1] = 0;
            rgb[2] = 0;
        }
    }

    private void fadeInOutWhite() {
        useHSV = false;
        rgb[0] = rgb[1] = rgb[2];
        if (isGoingUp) {
            rgb[2] += 0.01;
        } else {
            rgb[2] -= 0.01;
        }

        if (rgb[0] >= 1) {
            isGoingUp = false;
        } else if (rgb[0] <= 0) {
            isGoingUp = true;
        }
    }

    private void fadeInOutWhiteTraversalBar() {
        useHSV = false;
        rgb[0] = rgb[1] = rgb[2];
        if (isGoingUp) {
            rgb[2] += 0.15;
        } else {
            rgb[2] -= 0.15;
        }

        if (rgb[0] >= 1) {
            isGoingUp = false;
        } else if (rgb[0] <= 0) {
            isGoingUp = true;
        }
    }

    private void evaluateOnOffInterval(int onMs, int offMs) {
        long timer = System.currentTimeMillis();
        if (lastOnChangeTime + onMs + offMs < timer) { 
            //should be off, turn back on
            lastOnChangeTime = timer;
            areLedsOn = true;

        } else if (lastOnChangeTime + onMs < timer) {
            areLedsOn = false;
        }
    }

    public enum LEDStatusMode {
        RAINBOW("Rainbow"),
        OFF("Off"),
        BLINK_RED("Blink Red"),
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
