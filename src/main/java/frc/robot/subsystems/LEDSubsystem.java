package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem to control the robot's LEDs, by determining what number should be encoded to DIO pins and
 * sent to the Arduino we used for controlling the patterns and colors
 */
public class LEDSubsystem extends SubsystemBase {
    private DigitalOutput codeChannel1, codeChannel2, codeChannel3, codeChannel4, codeChannel5;

    private LEDStatusMode currentStatusMode;
    private LEDAlertStatusMode alertStatusMode;
    private LEDStatusMode currentDefaultStatusMode;

    private boolean disableLEDs;

    private Timer timer;

// This is a singleton pattern for making sure only 1 instance of this class exists that can be called from anywhere. Call with LEDSubsystem.getInstance()
// Construtor is private, instance is stored in static variable for the class, and instance is accessed using public getInstance() method.
    private LEDSubsystem() {
        codeChannel1 = new DigitalOutput(0);    // DIO outputs
        codeChannel2 = new DigitalOutput(1);
        codeChannel3 = new DigitalOutput(2);
        codeChannel4 = new DigitalOutput(3);
        codeChannel5 = new DigitalOutput(4);

        timer = new Timer();

        currentStatusMode = LEDStatusMode.DISABLED;

        // SmartDashboard.putNumber("LED CODE", 0); // For manually inputting code to encode to DIO pins
    }
    private static LEDSubsystem instance;       // Static that stores the instance of class
    public static LEDSubsystem getInstance() {  // Method to allow calling this class and getting the single instance from anywhere, creating the instance if the first time.
        if (instance == null) {
            instance = new LEDSubsystem();
        }
        return instance;
    }
    
    public enum LEDStatusMode {
        OFF(0, 10),
        RAINBOW(1, 10),
        SOLID_WHITE(3, 10),
        DISABLED(4, 1),
        DISABLED_RED_PULSE(20, 5),
        DISABLED_BLUE_PULSE(21, 5),
        AUTONOMOUS_INTAKE_ON(5, 10),
        AUTONOMOUS_DEFAULT(6, 10),
        AUTONOMOUS_FINISHED(7, 9),
        TELEOP_DEFAULT(8, 10),
        CLIMBING_DEFAULT(12, 10),
        CLIMBER_EXTENDING(13, 10),
        CLIMBER_RETRACTING(14, 10),
        CLIMBER_MAX_EXTENSION(15, 3),
        CLIMBER_MIN_EXTENSION(16, 3),
        CLIMBING_SWINGING_FORWARD(25, 3),
        CLIMBING_SWINGING_BACKWARD(26, 3),
        CLIMBING_HIGH_BAR(18, 8),
        CLIMBING_TRAVERSAL(19, 8),
        VISION_TARGETING(9, 4),
        VISION_TARGET_LINED_UP(10, 3),
        SHOOTING(24, 2),
        INTAKE_ON_0_BALLS(28, 9),
        INTAKE_ON_1_BALL(30, 8),
        HOPPER_FULL(29, 7),
        LIMELIGHT_DEAD(17, 1), // Placed here rather than alert status modes to be activated by DashboardControlsSubsystem only while the Limelight is in fact dead.
        ;

        private final int code; // Code to be encoded into the DIO pins to be received by arduino. May seem in a weird order because these weren't exactly created in order.
        private final int rank; // Ranking of status mode to determine if trying to set a new status mode should overide the current or not

        LEDStatusMode(int code, int rank) {
            this.code = code;
            this.rank = rank;
        }

        public int getRank() {
            return rank;
        }
    }

    /** Alert status modes are for setting the alert status mode once and having it last for the designated period of time
     * (Iteger.MAX_VALUE if indefinite) rather than constantly setting it like other status modes.
     */
    public enum LEDAlertStatusMode {
        CLIMBING_LOCK_ENGAGED(2, 1, Integer.MAX_VALUE),
        ENG_GAME_WARNING(11, 2, 5),
        CLIMBER_ARMS_BACK(22, 3, 0.2),
        CLIMBER_ARMS_FORWARD(23, 3, 0.2),
        CLIMBING_TOP_OF_SWING(27, 3, 0.5),
        //LIGHT_SHOW(30, 10, Integer.MAX_VALUE); // Meant for either demonstration or when the drivetrian is dead
        ;

        private final int code; // Code to be encoded into the DIO pins to be received by arduino
        private final int rank; // Ranking of status mode to determine if trying to set a new status mode should overide the current or not
        private final double duration;

        LEDAlertStatusMode(int code, int rank, double duration) {
            this.code = code;
            this.rank = rank;
            this.duration = duration;
        }

        public int getRank() {
            return rank;
        }
    }

    @Override
    public void periodic() {
        int matchTime = (int) DriverStation.getMatchTime(); // The current approximate match time
        int code = 0;

        if(!disableLEDs) {
            if (matchTime == 40) {
                alertStatusMode = LEDAlertStatusMode.ENG_GAME_WARNING;
            }

            if (alertStatusMode == null) {
                if (currentStatusMode == null) {
                    if (currentDefaultStatusMode == LEDStatusMode.DISABLED) {   // If disabled, finds gets the alliance color from the driver station and pulses that. Only pulses color if connected to station or FMS, else pulses default disabled color (Firefl status mode)
                        if (DriverStation.getAlliance() == Alliance.Red) {
                            currentStatusMode = LEDStatusMode.DISABLED_RED_PULSE;
                        } else if (DriverStation.getAlliance() == Alliance.Blue) {
                            currentStatusMode = LEDStatusMode.DISABLED_BLUE_PULSE;
                        } else {
                            currentStatusMode = LEDStatusMode.DISABLED; // Reaches here if DriverStation.getAlliance returns Invalid, which just means it can't determine our alliance and we do cool default effect
                        }
                    } else {
                        currentStatusMode = currentDefaultStatusMode;
                    }

                    // Old way of determining default status mode depending on game state.
                        // if (DriverStation.isAutonomous()) {
                        //     currentStatusMode = LEDStatusMode.AUTONOMOUS_DEFAULT;
                        // } else if (DriverStation.isTeleop()) {
                        //     currentStatusMode = LEDStatusMode.TELEOP_DEFAULT;
                        // } else if (DriverStation.isTest()) {
                        //     currentStatusMode = LEDStatusMode.TEST_MODE;
                        // } else {
                        //     currentStatusMode = LEDStatusMode.DISABLED;
                        // }
                }

                code = currentStatusMode.code;
            } else {
                code = alertStatusMode.code;

                if (timer.hasElapsed(alertStatusMode.duration)) {
                    clearAlertStatusMode();
                }
            }
        } else {
            code = 0;
        }
        
        // code = (int) SmartDashboard.getNumber("LED CODE", 0); // For manually inputting code to encode to DIO pins

        // SmartDashboard.putBoolean("channel1", (code & 1) > 0);
        // SmartDashboard.putBoolean("channel2", (code & 2) > 0);
        // SmartDashboard.putBoolean("channel3", (code & 4) > 0);
        // SmartDashboard.putBoolean("channel4", (code & 8) > 0);
        // SmartDashboard.putBoolean("channel5", (code & 16) > 0);

        // Code for encoding the code to binary on the digitalOutput pins
        codeChannel1.set((code & 1) > 0);
        codeChannel2.set((code & 2) > 0);
        codeChannel3.set((code & 4) > 0);
        codeChannel4.set((code & 8) > 0);
        codeChannel5.set((code & 16) > 0);

        clearStatusMode(); // Clears status mode after every loop to make sure high priority status modes 
        // don't stick around forever and everything trying to use it has to be activley setting the status mode
    }

    public void setLEDStatusMode(LEDStatusMode statusMode) {
        if (!disableLEDs) {
            if (statusMode == null) { // Makes sure to check we're not setting to null (which is allowed to go back to default mode) first before calling methods on it.
                currentStatusMode = null;
            } else {
                if (currentStatusMode == null) {
                    currentStatusMode = statusMode;
                } else if (statusMode.getRank() <= currentStatusMode.getRank()) { // Compares rank of desired status mode to the current, and does nothing if the current is a higher rank.
                    currentStatusMode = statusMode;
                }
            }
        }
    }

    public void setAlertLEDStatusMode(LEDAlertStatusMode statusMode) {
        if (!disableLEDs) {
            if (alertStatusMode != statusMode) {
                alertStatusMode = statusMode;
                
                timer.reset();
                timer.start();
            }
        }
    }

    /** Sets the mode to display when the current mode is null (nothing is trying to be displayed) */
    public void setDefaultLEDStatusMode(LEDStatusMode statusMode) {
        currentDefaultStatusMode = statusMode;
    }

    public void clearStatusMode() {
        currentStatusMode = null;
    }

    public void clearAlertStatusMode() {
        alertStatusMode = null;
    }

    // Disables LEDs (turns them off)
    public void disable() {
        disableLEDs = true;
    }

    // Enables LEDs (turns them on)
    public void enable() {
        disableLEDs = false;
    }

    // Unfinished lightShow method indended for making the robot look nice 
    // if we're sitting on the field with a dead drivetrain or somthing
        // private void lightShow() {
        //     double time = timer.get();
        //     if (time < 15) {
        //         currentStatusMode = LEDStatusMode.TELEOP_DEFAULT;
        //     } else if (time < 8) {
        //         currentStatusMode = LEDStatusMode.RAINBOW;
        //     } else {
        //         timer.reset();
        //     }
        // }
}

// -- OLD ROBOT LED CODE FOR USE WITH CANIFIER AND DOING LED LOGIC ON THE ROBOTRIO --
// Bad because it created alot more CAN traffic and weird processing

//    private final CANifier canifier;

    // private static LEDSubsystem channel1Instance;
    // private static LEDSubsystem channel2Instance;
    // public static LEDSubsystem getChannel1Instance() {
    //     if (channel1Instance == null) {
    //         channel1Instance = new LEDSubsystem(Constants.LEDs.R_1_PWM_PORT, Constants.LEDs.G_1_PWM_PORT, Constants.LEDs.B_1_PWM_PORT);
    //     }
    //     return channel1Instance;
    // }
    // public static LEDSubsystem getChannel2Instance() {
    //     if (channel2Instance == null) {
    //         channel2Instance = new LEDSubsystem(Constants.LEDs.R_2_PWM_PORT, Constants.LEDs.G_2_PWM_PORT, Constants.LEDs.B_2_PWM_PORT);
    //     }
    //     return channel2Instance;
    // }
    // public static void setBothChannelModes(LEDStatusMode statusMode) {
    //     getChannel1Instance().setLEDStatusMode(statusMode);
    //     getChannel2Instance().setLEDStatusMode(statusMode);
    // }

    // public static void setBothChannelBrightnesses(double brightness) {
    //     getChannel1Instance().setBrightness(brightness);
    //     getChannel2Instance().setBrightness(brightness);
    // }

    // public static void clearBothChannels() {
    //     getChannel1Instance().setLEDStatusMode(null);
    //     getChannel2Instance().setLEDStatusMode(null);
    // }

//     PWM redChannel, blueChannel, greenChannel;

//     private double [] rgb = new double[3]; // Array of RGB values, is actually GRB in the order of the array

//     private double saturation;
//     private double hue;
//     private double value;
//     private double externalBrightnessModifier;
//     private double counter;
//     private long lastOnChangeTime = 0;

//     private boolean areLedsOn = false;
//     private boolean isGoingUp = true;

//     private Timer timer;

//     private LEDStatusMode currentStatusMode; // Default Modes
//     private LEDStatusMode lastLEDStatusMode;

//     private LEDStatusMode runningStatusMode;
//     private LEDStatusMode lastRunningStatusMode;

//     // This is a singleton pattern for making sure only 1 instance of this class exists that can be called from anywhere. Call with LEDSubsystem.getInstance()
//     private LEDSubsystem(int redPWMPort, int greenPWMPort, int bluePWMPort) {
// //        canifier = new CANifier(Constants.LEDs.CANIFIER_PORT);

//         redChannel = new PWM(redPWMPort);
//         greenChannel = new PWM(greenPWMPort);
//         blueChannel = new PWM(bluePWMPort);

//         externalBrightnessModifier = (int)(SmartDashboard.getNumber("LED Brightness", 100) - 100) / 100.0;

//         isGoingUp = true;
//         timer = new Timer();

//         currentStatusMode = LEDStatusMode.TELEOP_DEFAULT;
//         lastLEDStatusMode = LEDStatusMode.TELEOP_DEFAULT;

//         runningStatusMode = LEDStatusMode.TELEOP_DEFAULT;
//         lastRunningStatusMode = LEDStatusMode.TELEOP_DEFAULT;
//     }
//     // private static LEDSubsystem instance;       // Static that stores the instance of class
//     // public static LEDSubsystem getInstance() {  // Method to allow calling this class and getting the single instance from anywhere, creating the instance if the first time.
//     //     if (instance == null) {
//     //         instance = new LEDSubsystem();
//     //     }
//     //     return instance;
//     // }

//     public void setLEDStatusMode(LEDStatusMode statusMode) {
//         currentStatusMode = statusMode;
//     }

//     public void setBrightness(double brightness) {
//         externalBrightnessModifier = (int)(brightness - 100) / 100.0;
//     }

//     @Override
//     public void periodic() { // Loop for updating LEDs in parallel with all other loops on the robot - Currently commented out becasue Canifier is fried

//         double matchTime = DriverStation.getMatchTime(); // The current approximate match time

//         if (matchTime >= 120 && matchTime <= 125) {
//             currentStatusMode = LEDStatusMode.ENG_GAME_WARNING;
//         } else if (currentStatusMode == null) {
//             if (DriverStation.isAutonomous()) {
//                 currentStatusMode = LEDStatusMode.AUTONOMOUS_DEFAULT;
//             } else if (DriverStation.isTeleop()) {
//                 currentStatusMode = LEDStatusMode.TELEOP_DEFAULT;
//             } else if (DriverStation.isTest()) {
//                 currentStatusMode = LEDStatusMode.TEST_MODE;
//             }
//         }

//         if (currentStatusMode != lastLEDStatusMode) {
//             counter = 0;
//             LEDsOff();
//             runLEDStatusModeInitial(currentStatusMode);
//             lastLEDStatusMode = currentStatusMode;
//         }

//         runLEDStatusMode();

//         greenChannel.setRaw((int) (rgb[0] / 255));
//         redChannel.setRaw((int) (rgb[1] / 255));
//         blueChannel.setRaw((int) (rgb[2] / 255));

//         // canifier.setLEDOutput(rgb[0] + externalBrightnessModifier, LEDChannel.LEDChannelA);  // G (Green)
//         // canifier.setLEDOutput(rgb[1] + externalBrightnessModifier, LEDChannel.LEDChannelB);  // R (Red)
//         // canifier.setLEDOutput(rgb[2] + externalBrightnessModifier, LEDChannel.LEDChannelC);  // B (Blue)
//     }

//     private void runLEDStatusModeInitial(LEDStatusMode statusMode) {
//         switch (statusMode) {
//             case RAINBOW:
//                 hue = 0;
//                 saturation = 1;
//                 value = 1;
//                 break;
//             case OFF:
//                 break;
//             case BLINK_RED:
//                 break;
//             case SOLID_WHITE:
//                 break;
//             case AUTONOMOUS_DEFAULT:
//                 break;
//             case AUTONOMOUS_INTAKE_ON:
//                 break;
//             case AUTONOMOUS_FINISHED:
//                 break;
//             case TELEOP_DEFAULT:
//                 break;    
//             case VISION_TARGETING:
//                 rgb[2] = 0.5;
//                 break;
//             case VISION_TARGET_FOUND:
//                 rgb[0] = 0.5;
//                 break;
//             case ENG_GAME_WARNING:
//                 break;
//             case CLIMBING_DEFAULT:
//                 break;
//             case CLIMBER_EXTENDING:
//                 break;
//             case CLIMBER_RETRACTING:
//                 rgb[1] = 0.5;
//                 rgb[2] = rgb[1] * 0.2;
//                 break;
//             case CLIMBER_MAX_EXTENSION:
//                 rgb[2] = 0.5;
//                 break;
//             case CLIMBER_MIN_EXTENSION:
//                 break;
//             case CLIMBING_LOW_BAR:
//                 break;
//             case CLIMBING_MID_BAR:
//                 break;
//             case CLIMBING_HIGH_BAR:
//                 break;
//             case CLIMBING_TRAVERSAL:
//                 break;
//             case CLIMBING_LOCK_ENGAGED:
//                 break;
//             case TEST_MODE:
//                 break;
//             case LIGHT_SHOW:
//                 timer.start();
//                 break;
//             default:
//                 System.err.println("LED INITIAL SWITCH FELL THROUGH");
//                 break;
//         }
//     }

//     private void runLEDStatusMode() {
//         switch (currentStatusMode) {
//             case RAINBOW:
//                 rainbowStatusMode();
//                 break;
//             case OFF:
//                 LEDsOff();
//                 break;
//             case BLINK_RED:
//                 blinkingRedStatusMode();
//                 break;
//             case SOLID_WHITE:
//                 LEDsOnWhite();
//                 break;
//             case AUTONOMOUS_INTAKE_ON:
//                 intakeOnStatusMode();
//                 break;
//             case AUTONOMOUS_DEFAULT:
//                 autonomousDefaultStatusMode();
//                 break;
//             case AUTONOMOUS_FINISHED:
//                 autonomousFinishedStatusMode();
//                 break;
//             case TELEOP_DEFAULT:
//                 fireFlyStatusMode();
//                 break;    
//             case VISION_TARGETING:
//                 visionTargetingStatusMode();
//                 break;
//             case VISION_TARGET_FOUND:
//                 visionTargetFoundStatusMode();
//                 break;
//             case ENG_GAME_WARNING:
//                 endGameWarningStatusMode();
//                 break;
//             case CLIMBING_DEFAULT:
//                 climbingDefaultStatusMode();
//                 break;
//             case CLIMBER_EXTENDING:
//                 climberExtendingStatusMode();
//                 break;
//             case CLIMBER_RETRACTING:
//                 climberRetractingStatusMode();
//                 break;
//             case CLIMBER_MAX_EXTENSION:
//                 climberMaxExtensionStatusMode();
//                 break;
//             case CLIMBER_MIN_EXTENSION:
//                 break;
//             case CLIMBING_LOW_BAR:
//                 lowBarStatusMode();
//                 break;
//             case CLIMBING_MID_BAR:
//                 midBarStatusMode();
//                 break;
//             case CLIMBING_HIGH_BAR:
//                 highBarStatusMode();
//                 break;
//             case CLIMBING_TRAVERSAL:
//                 traversalBarStatusMode();
//                 break;
//             case CLIMBING_LOCK_ENGAGED:
//                 climbingLockEngagedStatusMode();
//                 break;
//             case TEST_MODE:
//                 testStatusMode();
//                 break;
//             case LIGHT_SHOW:
//                 lightShowStatusMode();
//                 break;
//             default:
//             System.err.println("LED STATUS MODE SWITCH FELL THROUGH");
//                 break;
//         }
//     }

//     private void rainbowStatusMode() {
//         hue += 1;

//         if (hue >= 360) {
//             hue = 0;
//         }

//         setRGBfromHSV();
//     }

//     private void blinkingRedStatusMode() {
//         evaluateOnOffInterval(2000, 1000);
//         if (areLedsOn) {
//             rgb[0] = 0;
//             rgb[1] = 1;
//             rgb[2] = 0;
//         } else {
//             LEDsOff();
//         }
//     }

//     private void visionTargetingStatusMode() {
//         if (isGoingUp) {
//             rgb[2] += 0.1;
//         } else {
//             rgb[2] -= 0.1;
//         }

//         if (rgb[2] >= 1) {
//             isGoingUp = false;
//         } else if (rgb[2] <= 0.5) {
//             isGoingUp = true;
//         }

//         /*evaluateOnOffInterval(500, 500);
//         if (areLedsOn) {
//             rgb[0] = 0;
//             rgb[1] = 1;
//             rgb[2] = 0;
//         } else {
//             LEDsOff();
//         }*/
//     }

//     private void visionTargetFoundStatusMode() {
//         if (isGoingUp) {
//             rgb[0] += 0.2;
//         } else {
//             rgb[0] -= 0.2;
//         }

//         if (rgb[0] >= 1) {
//             isGoingUp = false;
//         } else if (rgb[0] <= 0.5) {
//             isGoingUp = true;
//         }
//         /*evaluateOnOffInterval(300, 300);
//         if (areLedsOn) {
//             rgb[0] = 1;
//             rgb[1] = 0;
//             rgb[2] = 0;
//         } else {
//             LEDsOff();
//         }*/
//     }

//     private void intakeOnStatusMode() {
//         rgb[1] = 1;
//         rgb[0] = 0.2;
//     }

//     private void autonomousDefaultStatusMode() {}

//     private void autonomousFinishedStatusMode() {
//         if (isGoingUp) {
//             rgb[0] += 0.01;
//         } else {
//             rgb[0] -= 0.01;
//         }

//         if (rgb[0] >= 1) {
//             isGoingUp = false;
//         } else if (rgb[0] <= 0) {
//             isGoingUp = true;
//         }
//     }

//     private void endGameWarningStatusMode() {
//         rgb[0] = rgb[1] = rgb[2];
//         if (isGoingUp) {
//             rgb[2] += 0.15;
//         } else {
//             rgb[2] -= 0.15;
//         }

//         if (rgb[0] >= 1) {
//             isGoingUp = false;
//         } else if (rgb[0] <= 0) {
//             isGoingUp = true;
//             counter++;
//         }

//         if (counter == 5) {
//             currentStatusMode = LEDStatusMode.CLIMBING_DEFAULT;
//         }
//     }

//     private void climbingDefaultStatusMode() {
//         rgb[0] = rgb[1] = rgb[2];
//         if (isGoingUp) {
//             rgb[2] += 0.01;
//         } else {
//             rgb[2] -= 0.01;
//         }

//         if (rgb[0] >= 1) {
//             isGoingUp = false;
//         } else if (rgb[0] <= 0) {
//             isGoingUp = true;
//         }
//     }

//     private void climberExtendingStatusMode() {
//         if (isGoingUp) {
//             rgb[2] += 0.1;
//         } else {
//             rgb[2] -= 0.1;
//         }

//         if (rgb[2] >= 1) {
//             isGoingUp = false;
//         } else if (rgb[2] <= 0.5) {
//             isGoingUp = true;
//         }
//     }

//     private void climberRetractingStatusMode() {
//         if (isGoingUp) {
//             rgb[1] += 0.1;
//             rgb[2] = rgb[1] * 0.2;
//         } else {
//             rgb[2] -= 0.1;
//             rgb[2] = rgb[1] * 0.2;
//         }

//         if (rgb[2] >= 1) {
//             isGoingUp = false;
//         } else if (rgb[2] <= 0.5) {
//             isGoingUp = true;
//         }
//     }

//     private void climberMaxExtensionStatusMode() {
//         if (isGoingUp) {
//             rgb[0] += 0.1;
//         } else {
//             rgb[0] -= 0.1;
//         }

//         if (rgb[0] >= 1) {
//             isGoingUp = false;
//         } else if (rgb[0] <= 0.5) {
//             isGoingUp = true;
//         }
//     }

//     private void lowBarStatusMode() {
//         rgb[0] = rgb[1] = rgb[2];
//         if (isGoingUp) {
//             rgb[2] += 0.05;
//         } else {
//             rgb[2] -= 0.05;
//         }

//         if (rgb[0] >= 1) {
//             isGoingUp = false;
//         } else if (rgb[0] <= 0) {
//             isGoingUp = true;
//         }
//     }

//     private void midBarStatusMode() {
//         rgb[0] = rgb[1] = rgb[2];
//         if (isGoingUp) {
//             rgb[2] += 0.08;
//         } else {
//             rgb[2] -= 0.08;
//         }

//         if (rgb[0] >= 1) {
//             isGoingUp = false;
//         } else if (rgb[0] <= 0) {
//             isGoingUp = true;
//         }
//     }

//     private void highBarStatusMode() {
//         rgb[0] = rgb[1] = rgb[2];
//         if (isGoingUp) {
//             rgb[2] += 0.11;
//         } else {
//             rgb[2] -= 0.11;
//         }

//         if (rgb[0] >= 1) {
//             isGoingUp = false;
//         } else if (rgb[0] <= 0) {
//             isGoingUp = true;
//         }
//     }

//     private void traversalBarStatusMode() {
//         rgb[0] = rgb[1] = rgb[2];
//         if (isGoingUp) {
//             rgb[2] += 0.15;
//         } else {
//             rgb[2] -= 0.15;
//         }

//         if (rgb[0] >= 1) {
//             isGoingUp = false;
//         } else if (rgb[0] <= 0) {
//             isGoingUp = true;
//         }
//     }

//     private void fireFlyStatusMode() {
//         if (isGoingUp) {
//             counter += 0.4;
//         } else {
//             counter -= 0.4;
//         }

//         rgb[1] = 0.0003 * Math.pow(counter, 2) - 0.2;
//         rgb[0] = rgb[1] * 0.2;

//         if (counter >= 60) {
//             isGoingUp = false;
//         } else if (counter <= 0) {
//             isGoingUp = true;
//         }
//     }

//     private void climbingLockEngagedStatusMode() {
//         evaluateOnOffInterval(500, 500);
//         if (areLedsOn) {
//             rgb[1] = 1;
//         } else {
//             rgb[0] = 0;
//         }
//     }
    
//     private void testStatusMode() {
//         evaluateOnOffInterval(1000, 1000);
//         if (areLedsOn) {
//             rgb[0] = 1;
//             rgb[1] = 0.2;
//         } else {
//             LEDsOff();
//         }
//     }

//     private void lightShowStatusMode() {
//         double time = timer.get();
//         if (runningStatusMode != lastRunningStatusMode) {
//             runLEDStatusModeInitial(runningStatusMode);
//             lastRunningStatusMode = runningStatusMode;
//         }
//         if (time < 10) {

//         } else if (time < 5) {
//             runningStatusMode = LEDStatusMode.RAINBOW;
//             rainbowStatusMode();
//         } else {
//             timer.reset();
//         }
//     }

//     private void evaluateOnOffInterval(int onMs, int offMs) {
//         long timer = System.currentTimeMillis();
//         if (lastOnChangeTime + onMs + offMs < timer) { 
//             //should be off, turn back on
//             lastOnChangeTime = timer;
//             areLedsOn = true;

//         } else if (lastOnChangeTime + onMs < timer) {
//             areLedsOn = false;
//         }
//     }

//     private void LEDsOff() {
//         rgb[0] = 0;
//         rgb[1] = 0;
//         rgb[2] = 0;
//     }

//     private void LEDsOnWhite() {
//         rgb[0] = 0.3;
//         rgb[1] = 0.3;
//         rgb[2] = 0.3;
//     }

//     private void setRGBfromHSV() {
//         if (hue > 360) {
//             hue = 360;
//         } else if (hue < 0) {
//             hue = 0;
//         }

//         if (saturation > 1) {
//             saturation = 1;
//         } else if (saturation < 0) {
//             saturation = 0;
//         }

//         if (value > 1) {
//             value = 1;
//         } else if (value < 0) {
//             value = 0;
//         }

//     //Converts HSV (Hue Saturation Value) to RGB (Red Green Blue)
//         rgb = HsvToRgb.convert(hue, saturation, value);
//     }
// }
