package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem to control the robot's LEDs, by determining what number should be encoded to DIO pins and
 * sent to the Arduino we used for controlling the patterns and colors
 */
public class LEDSubsystem extends SubsystemBase {
    private DigitalOutput codeChannel1, codeChannel2, codeChannel3, codeChannel4, codeChannel5;

    private LEDStatusMode currentStatusMode;

    private boolean disableLEDs;

// This is a singleton pattern for making sure only 1 instance of this class exists that can be called from anywhere. Call with LEDSubsystem.getInstance()
// Construtor is private, instance is stored in static variable for the class, and instance is accessed using public getInstance() method.
    private LEDSubsystem() {
        codeChannel1 = new DigitalOutput(Constants.LEDs.CHANNEL_1_PIN);    // DIO outputs
        codeChannel2 = new DigitalOutput(Constants.LEDs.CHANNEL_2_PIN);
        codeChannel3 = new DigitalOutput(Constants.LEDs.CHANNEL_3_PIN);
        codeChannel4 = new DigitalOutput(Constants.LEDs.CHANNEL_4_PIN);
        codeChannel5 = new DigitalOutput(Constants.LEDs.CHANNEL_5_PIN);

        currentStatusMode = LEDStatusMode.RAINBOW;
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
        TELEOP_DEFAULT(8, 10),
        SHOOTING(24, 2),
        INTAKE_ON_0_BALLS(28, 9),
        INTAKE_ON_1_BALL(30, 8),
        HOPPER_FULL(29, 7);

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

    @Override
    public void periodic() {
        int code = 0;

        // We are just doing rainbow because firefly is now a demo robot and rainbows are cool
        code = 1;

        // if(!disableLEDs) {
        //     if (currentStatusMode == null) {
        //         if (currentDefaultStatusMode == LEDStatusMode.DISABLED) {   // If disabled, finds gets the alliance color from the driver station and pulses that. Only pulses color if connected to station or FMS, else pulses default disabled color (Firefl status mode)
        //             if (DriverStation.getAlliance() == Alliance.Red) {
        //                 currentStatusMode = LEDStatusMode.DISABLED_RED_PULSE;
        //             } else if (DriverStation.getAlliance() == Alliance.Blue) {
        //                 currentStatusMode = LEDStatusMode.DISABLED_BLUE_PULSE;
        //             } else {
        //                 currentStatusMode = LEDStatusMode.DISABLED; // Reaches here if DriverStation.getAlliance returns Invalid, which just means it can't determine our alliance and we do cool default effect
        //             }
        //         } else {
        //             currentStatusMode = currentDefaultStatusMode;
        //         }
        //     }

        //     code = currentStatusMode.code;
        // } else {
        //     code = 0;
        // }

        // Code for encoding the code to binary on the digitalOutput pins
        codeChannel1.set((code & 1) > 0);   // 2^0
        codeChannel2.set((code & 2) > 0);   // 2^1
        codeChannel3.set((code & 4) > 0);   // 2^2
        codeChannel4.set((code & 8) > 0);   // 2^3
        codeChannel5.set((code & 16) > 0);  // 2^4

        //clearStatusMode(); // Clears status mode after every loop to make sure high priority status modes 
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

    /** Sets the mode to display when the current mode is null (nothing is trying to be displayed) */
    // public void setDefaultLEDStatusMode(LEDStatusMode statusMode) {
    //     currentDefaultStatusMode = statusMode;
    // }

    public void clearStatusMode() {
        currentStatusMode = null;
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