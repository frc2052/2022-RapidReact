package frc.robot.subsystems.LEDs;

// Subsystem to Control CANifier LED Controller

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.HsvToRgb;

public class LEDSubsystem extends SubsystemBase {

//    private final CANifier canifier;

    private LEDSubsystem() {}
    private static LEDSubsystem instance;
    private static LEDLoop channel1Instance;
    private static LEDLoop channel2Instance;
    public static LEDLoop getChannel1Instance() {
        if (instance == null) {
            instance = new LEDSubsystem();
        }
        if (channel1Instance == null) {
            channel1Instance = instance.new LEDLoop(Constants.LEDs.R_1_PWM_PORT, Constants.LEDs.G_1_PWM_PORT, Constants.LEDs.B_1_PWM_PORT);
        }
        return channel1Instance;
    }
    public static LEDLoop getChannel2Instance() {
        if (instance == null) {
            instance = new LEDSubsystem();
        }
        if (channel2Instance == null) {
            channel2Instance = instance.new LEDLoop(Constants.LEDs.R_2_PWM_PORT, Constants.LEDs.G_2_PWM_PORT, Constants.LEDs.B_2_PWM_PORT);
        }
        return channel2Instance;
    }
    public static void setBothChannelModes(LEDStatusMode statusMode) {
        getChannel1Instance().setLEDStatusMode(statusMode);
        getChannel2Instance().setLEDStatusMode(statusMode);
    }

    public static void setBothChannelBrightnesses(double brightness) {
        getChannel1Instance().setBrightness(brightness);
        getChannel2Instance().setBrightness(brightness);
    }

    public enum LEDStatusMode {
        RAINBOW("Rainbow"),
        OFF("Off"),
        BLINK_RED("Blink Red"),
        SOLID_WHITE("Solid White"),
        CURRENT_DEFAULT("Current Default Mode for Game Stage"),
        AUTONOMOUS_INTAKE_ON("Autonomous Intake On"),
        AUTONOMOUS_DEFAULT("Autonomous Default"),
        AUTONOMOUS_FINISHED("Autonomous Finished"),
        TELEOP_DEFAULT("Teleop Default"),
        VISION_TARGETING("Vision Targeting"),
        VISION_TARGET_FOUND("Vision Target Found"),
        ENG_GAME_WARNING("End Game Warning - 10 Seconds Till End Game"),
        CLIMBING_DEFAULT("Climbing Default"),
        CLIMBER_EXTENDING("Climber Extending"),
        CLIMBER_RETRACTING("Climber Retracting"),
        CLIMBER_MAX_EXTENSION("Climber Max Extension"),
        CLIMBER_MIN_EXTENSION("Climber Min Extension"),
        CLIMBING_LOW_BAR("Climbing Low Bar"),
        CLIMBING_MID_BAR("Climbing Middle Bar"),
        CLIMBING_HIGH_BAR("Climbing High Bar"),
        CLIMBING_TRAVERSAL("Climbing Traversal Bar"),
        CLIMBING_LOCK_ENGAGED("Climber Lock Engaged"),
        TEST_MODE("Test Mode"),
        LIGHT_SHOW("Light Show"); // Meant for either demonstration or when the drivetrian is dead

        public String name;

        LEDStatusMode(String name) {
            this.name = name;
        }
    }

    private class LEDLoop extends SubsystemBase {
        PWM redChannel, blueChannel, greenChannel;

    private double [] rgb = new double[3]; // Array of RGB values, is actually GRB in the order of the array

    private double saturation;
    private double hue;
    private double value;
    private double externalBrightnessModifier;
    private double counter;
    private long lastOnChangeTime = 0;

    private boolean areLedsOn = false;
    private boolean isGoingUp = true;

    private Timer timer;

    private LEDStatusMode currentLEDStatusMode; // Default Modes
    private LEDStatusMode lastLEDStatusMode;

    private LEDStatusMode runningStatusMode;
    private LEDStatusMode lastRunningStatusMode;

    // This is a singleton pattern for making sure only 1 instance of this class exists that can be called from anywhere. Call with LEDSubsystem.getInstance()
    public LEDLoop(int redPWMPort, int greenPWMPort, int bluePWMPort) {
//        canifier = new CANifier(Constants.LEDs.CANIFIER_PORT);

        redChannel = new PWM(redPWMPort);
        greenChannel = new PWM(greenPWMPort);
        blueChannel = new PWM(bluePWMPort);

        externalBrightnessModifier = (int)(SmartDashboard.getNumber("LED Brightness", 100) - 100) / 100.0;

        isGoingUp = true;
        timer = new Timer();

        currentLEDStatusMode = LEDStatusMode.TELEOP_DEFAULT;
        lastLEDStatusMode = LEDStatusMode.TELEOP_DEFAULT;

        runningStatusMode = LEDStatusMode.TELEOP_DEFAULT;
        lastRunningStatusMode = LEDStatusMode.TELEOP_DEFAULT;
    }
    // private static LEDSubsystem instance;       // Static that stores the instance of class
    // public static LEDSubsystem getInstance() {  // Method to allow calling this class and getting the single instance from anywhere, creating the instance if the first time.
    //     if (instance == null) {
    //         instance = new LEDSubsystem();
    //     }
    //     return instance;
    // }

    public void setLEDStatusMode(LEDStatusMode statusMode) {
        currentLEDStatusMode = statusMode;
    }

    public void setBrightness(double brightness) {
        externalBrightnessModifier = (int)(brightness - 100) / 100.0;
    }

    @Override
    public void periodic() { // Loop for updating LEDs in parallel with all other loops on the robot - Currently commented out becasue Canifier is fried
        
        double matchTime = DriverStation.getMatchTime(); // The current approximate match time

        if (matchTime >= 120 && matchTime <= 125) {
            currentLEDStatusMode = LEDStatusMode.ENG_GAME_WARNING;
        }

        if (currentLEDStatusMode != lastLEDStatusMode) {
            counter = 0;
            LEDsOff();
            runLEDStatusModeInitial(currentLEDStatusMode);
            lastLEDStatusMode = currentLEDStatusMode;
        }

        runLEDStatusMode();

        greenChannel.setRaw((int) (rgb[0] / 255));
        redChannel.setRaw((int) (rgb[1] / 255));
        blueChannel.setRaw((int) (rgb[2] / 255));

        // canifier.setLEDOutput(rgb[0] + externalBrightnessModifier, LEDChannel.LEDChannelA);  // G (Green)
        // canifier.setLEDOutput(rgb[1] + externalBrightnessModifier, LEDChannel.LEDChannelB);  // R (Red)
        // canifier.setLEDOutput(rgb[2] + externalBrightnessModifier, LEDChannel.LEDChannelC);  // B (Blue)
    }

    private void runLEDStatusModeInitial(LEDStatusMode statusMode) {
        switch (statusMode) {
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
            case AUTONOMOUS_INTAKE_ON:
                break;
            case AUTONOMOUS_FINISHED:
                break;
            case TELEOP_DEFAULT:
                break;    
            case VISION_TARGETING:
                rgb[2] = 0.5;
                break;
            case VISION_TARGET_FOUND:
                rgb[0] = 0.5;
                break;
            case ENG_GAME_WARNING:
                break;
            case CLIMBING_DEFAULT:
                break;
            case CLIMBER_EXTENDING:
                break;
            case CLIMBER_RETRACTING:
                rgb[1] = 0.5;
                rgb[2] = rgb[1] * 0.2;
                break;
            case CLIMBER_MAX_EXTENSION:
                rgb[2] = 0.5;
                break;
            case CLIMBER_MIN_EXTENSION:
                break;
            case CLIMBING_LOW_BAR:
                break;
            case CLIMBING_MID_BAR:
                break;
            case CLIMBING_HIGH_BAR:
                break;
            case CLIMBING_TRAVERSAL:
                break;
            case CLIMBING_LOCK_ENGAGED:
                break;
            case TEST_MODE:
                break;
            case LIGHT_SHOW:
                timer.start();
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
                LEDsOff();
                break;
            case BLINK_RED:
                blinkingRedStatusMode();
                break;
            case SOLID_WHITE:
                LEDsOnWhite();
                break;
            case AUTONOMOUS_INTAKE_ON:
                intakeOnStatusMode();
                break;
            case AUTONOMOUS_DEFAULT:
                autonomousDefaultStatusMode();
                break;
            case AUTONOMOUS_FINISHED:
                autonomousFinishedStatusMode();
                break;
            case TELEOP_DEFAULT:
                fireFlyStatusMode();
                break;    
            case VISION_TARGETING:
                visionTargetingStatusMode();
                break;
            case VISION_TARGET_FOUND:
                visionTargetFoundStatusMode();
                break;
            case ENG_GAME_WARNING:
                endGameWarningStatusMode();
                break;
            case CLIMBING_DEFAULT:
                climbingDefaultStatusMode();
                break;
            case CLIMBER_EXTENDING:
                climberExtendingStatusMode();
                break;
            case CLIMBER_RETRACTING:
                climberRetractingStatusMode();
                break;
            case CLIMBER_MAX_EXTENSION:
                climberMaxExtensionStatusMode();
                break;
            case CLIMBER_MIN_EXTENSION:
                break;
            case CLIMBING_LOW_BAR:
                lowBarStatusMode();
                break;
            case CLIMBING_MID_BAR:
                midBarStatusMode();
                break;
            case CLIMBING_HIGH_BAR:
                highBarStatusMode();
                break;
            case CLIMBING_TRAVERSAL:
                traversalBarStatusMode();
                break;
            case CLIMBING_LOCK_ENGAGED:
                climbingLockEngagedStatusMode();
                break;
            case TEST_MODE:
                testStatusMode();
                break;
            case LIGHT_SHOW:
                lightShowStatusMode();
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
        if (isGoingUp) {
            rgb[2] += 0.1;
        } else {
            rgb[2] -= 0.1;
        }

        if (rgb[2] >= 1) {
            isGoingUp = false;
        } else if (rgb[2] <= 0.5) {
            isGoingUp = true;
        }

        /*evaluateOnOffInterval(500, 500);
        if (areLedsOn) {
            rgb[0] = 0;
            rgb[1] = 1;
            rgb[2] = 0;
        } else {
            LEDsOff();
        }*/
    }

    private void visionTargetFoundStatusMode() {
        if (isGoingUp) {
            rgb[0] += 0.2;
        } else {
            rgb[0] -= 0.2;
        }

        if (rgb[0] >= 1) {
            isGoingUp = false;
        } else if (rgb[0] <= 0.5) {
            isGoingUp = true;
        }
        /*evaluateOnOffInterval(300, 300);
        if (areLedsOn) {
            rgb[0] = 1;
            rgb[1] = 0;
            rgb[2] = 0;
        } else {
            LEDsOff();
        }*/
    }

    private void intakeOnStatusMode() {
        rgb[1] = 1;
        rgb[0] = 0.2;
    }

    private void autonomousDefaultStatusMode() {}

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

    private void endGameWarningStatusMode() {
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
            counter++;
        }

        if (counter == 5) {
            currentLEDStatusMode = LEDStatusMode.CLIMBING_DEFAULT;
        }
    }

    private void climbingDefaultStatusMode() {
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

    private void climberExtendingStatusMode() {
        if (isGoingUp) {
            rgb[2] += 0.1;
        } else {
            rgb[2] -= 0.1;
        }

        if (rgb[2] >= 1) {
            isGoingUp = false;
        } else if (rgb[2] <= 0.5) {
            isGoingUp = true;
        }
    }

    private void climberRetractingStatusMode() {
        if (isGoingUp) {
            rgb[1] += 0.1;
            rgb[2] = rgb[1] * 0.2;
        } else {
            rgb[2] -= 0.1;
            rgb[2] = rgb[1] * 0.2;
        }

        if (rgb[2] >= 1) {
            isGoingUp = false;
        } else if (rgb[2] <= 0.5) {
            isGoingUp = true;
        }
    }

    private void climberMaxExtensionStatusMode() {
        if (isGoingUp) {
            rgb[0] += 0.1;
        } else {
            rgb[0] -= 0.1;
        }

        if (rgb[0] >= 1) {
            isGoingUp = false;
        } else if (rgb[0] <= 0.5) {
            isGoingUp = true;
        }
    }

    private void lowBarStatusMode() {
        rgb[0] = rgb[1] = rgb[2];
        if (isGoingUp) {
            rgb[2] += 0.05;
        } else {
            rgb[2] -= 0.05;
        }

        if (rgb[0] >= 1) {
            isGoingUp = false;
        } else if (rgb[0] <= 0) {
            isGoingUp = true;
        }
    }

    private void midBarStatusMode() {
        rgb[0] = rgb[1] = rgb[2];
        if (isGoingUp) {
            rgb[2] += 0.08;
        } else {
            rgb[2] -= 0.08;
        }

        if (rgb[0] >= 1) {
            isGoingUp = false;
        } else if (rgb[0] <= 0) {
            isGoingUp = true;
        }
    }

    private void highBarStatusMode() {
        rgb[0] = rgb[1] = rgb[2];
        if (isGoingUp) {
            rgb[2] += 0.11;
        } else {
            rgb[2] -= 0.11;
        }

        if (rgb[0] >= 1) {
            isGoingUp = false;
        } else if (rgb[0] <= 0) {
            isGoingUp = true;
        }
    }

    private void traversalBarStatusMode() {
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

    private void fireFlyStatusMode() {
        if (isGoingUp) {
            counter += 0.4;
        } else {
            counter -= 0.4;
        }

        rgb[1] = 0.0003 * Math.pow(counter, 2) - 0.2;
        rgb[0] = rgb[1] * 0.2;

        if (counter >= 60) {
            isGoingUp = false;
        } else if (counter <= 0) {
            isGoingUp = true;
        }
    }

    private void climbingLockEngagedStatusMode() {
        evaluateOnOffInterval(500, 500);
        if (areLedsOn) {
            rgb[1] = 1;
        } else {
            rgb[0] = 0;
        }
    }
    
    private void testStatusMode() {
        evaluateOnOffInterval(1000, 1000);
        if (areLedsOn) {
            rgb[0] = 1;
            rgb[1] = 0.2;
        } else {
            LEDsOff();
        }
    }

    private void lightShowStatusMode() {
        double time = timer.get();
        if (runningStatusMode != lastRunningStatusMode) {
            runLEDStatusModeInitial(runningStatusMode);
            lastRunningStatusMode = runningStatusMode;
        }
        if (time < 10) {

        } else if (time < 5) {
            runningStatusMode = LEDStatusMode.RAINBOW;
            rainbowStatusMode();
        } else {
            timer.reset();
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

    private void LEDsOnWhite() {
        rgb[0] = 0.3;
        rgb[1] = 0.3;
        rgb[2] = 0.3;
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
}
