package frc.robot.subsystems;

// Subsystem to Control CANifier LED Controller

import java.lang.reflect.Method;
import java.util.Timer;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.HsvToRgb;

public class LEDSubsystem extends SubsystemBase {

    private final CANifier canifier;

    // This is a singleton pattern for making sure only 1 instance of this class exists that can be called from anywhere. Call with LEDSubsystem.getInstance()
    private LEDSubsystem() {
        canifier = new CANifier(Constants.LEDs.CANIFIER_PORT);
    }
    private static LEDSubsystem instance;       // Static that stores the instance of class
    public static LEDSubsystem getInstance() {  // Method to allow calling this class and getting the single instance from anywhere, creating the instance if the first time.
        if (instance == null) {
            instance = new LEDSubsystem();
        }
        return instance;
    }

    private double [] rgb = new double[3]; // Is actually GRB in the order of the array

    private double saturation = 1;
    private double hue = 0;
    private double value = 1;

    private long lastOnChangeTime = 0;

    private boolean areLedsOn = false;
    private boolean isGoingUp = true;

    private LEDStatusMode currentLEDStatusMode = LEDStatusMode.RAINBOW; // Default Modes
    private LEDStatusMode lastLEDStatusMode = LEDStatusMode.RAINBOW;

    
    public enum LEDStatusMode {
        RAINBOW("Rainbow"),
        OFF("Off"),
        BLINK_RED("Blink Red"),
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

    public void setLEDStatusMode(LEDStatusMode statusMode) {
        currentLEDStatusMode = statusMode;
    }

    @Override
    public void periodic() { // Loop for updating LEDs in parallel with all other loops on the robot

        if (currentLEDStatusMode != lastLEDStatusMode) {
            LEDsOff();
            runLEDStatusModeInitial();
            lastLEDStatusMode = currentLEDStatusMode;
        }

        runLEDStatusMode();

        canifier.setLEDOutput(rgb[0], LEDChannel.LEDChannelA);  // G (Green)
        canifier.setLEDOutput(rgb[1], LEDChannel.LEDChannelB);  // R (Red)
        canifier.setLEDOutput(rgb[2], LEDChannel.LEDChannelC);  // B (Blue)
    }

    private void runLEDStatusModeInitial() {
        switch (currentLEDStatusMode) {
            case RAINBOW:
                hue = 0;
                saturation = 1;
                value = 1;
                break;
            case OFF:
                break;
            case BLINK_RED:
                break;
            case SOLID_WHITE:
                break;
            case AUTONOMOUS_DEFAULT:
                break;
            case AUTONOMOUS_FINISHED:
                break;
            case TELEOP_DEFAULT:
                break;    
            case VISION_TARGETING:
                break;
            case VISION_TARGET_FOUND:
                break;
            case CLIMBING_DEFAULT:
                break;
            case CLIMBING_LOW_BAR:
                break;
            case CLIMBING_MID_BAR:
                break;
            case CLIMBING_TRAVERSAL:
                break;          
            default:
                System.err.println("LED INITIAL SWITCH FELL THROUGH");
                break;
        }
    }

    private void runLEDStatusMode() {
        switch (currentLEDStatusMode) {
            case RAINBOW:
                rainbowStatusMode();
                break;
            case OFF:
                break;
            case BLINK_RED:
                blinkingRedStatusMode();
                break;
            case SOLID_WHITE:
                break;
            case AUTONOMOUS_DEFAULT:
                break;
            case AUTONOMOUS_FINISHED:
                autonomousFinishedStatusMode();
                break;
            case TELEOP_DEFAULT:
                break;    
            case VISION_TARGETING:
                visionTargetingStatusMode();
                break;
            case VISION_TARGET_FOUND:
                visionTargetFoundStatusMode();
                break;
            case CLIMBING_DEFAULT:
                fadeInOutWhite();
                break;
            case CLIMBING_LOW_BAR:
                break;
            case CLIMBING_MID_BAR:
                break;
            case CLIMBING_TRAVERSAL:
                fadeInOutWhiteTraversalBar();
                break;          
            default:
            System.err.println("LED STATUS MODE SWITCH FELL THROUGH");
                break;
        }
    }

    private void rainbowStatusMode() {
        hue += 1;

        if (hue >= 360) {
            hue = 0;
        }

        setRGBfromHSV();
    }

    private void blinkingRedStatusMode() {
        evaluateOnOffInterval(2000, 1000);
        if (areLedsOn) {
            rgb[0] = 0;
            rgb[1] = 1;
            rgb[2] = 0;
        } else {
            LEDsOff();
        }
    }

    private void visionTargetingStatusMode() {
        evaluateOnOffInterval(500, 500);
        if (areLedsOn) {
            rgb[0] = 0;
            rgb[1] = 1;
            rgb[2] = 0;
        } else {
            LEDsOff();
        }
    }

    private void visionTargetFoundStatusMode() {
        evaluateOnOffInterval(300, 300);
        if (areLedsOn) {
            rgb[0] = 1;
            rgb[1] = 0;
            rgb[2] = 0;
        } else {
            LEDsOff();
        }
    }

    private void autonomousFinishedStatusMode() {
        if (isGoingUp) {
            rgb[0] += 0.01;
        } else {
            rgb[0] -= 0.01;
        }

        if (rgb[0] >= 1) {
            isGoingUp = false;
        } else if (rgb[0] <= 0) {
            isGoingUp = true;
        }
    }

    private void fadeInOutWhite() {
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

    private void LEDsOff() {
        rgb[0] = 0;
        rgb[1] = 0;
        rgb[2] = 0;
    }

    private void setRGBfromHSV() {
        if (hue > 360) {
            hue = 360;
        } else if (hue < 0) {
            hue = 0;
        }

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
}
