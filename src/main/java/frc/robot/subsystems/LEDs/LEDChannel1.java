package frc.robot.subsystems.LEDs;

import frc.robot.Constants;

public class LEDChannel1 extends LEDSubsystem {

    private LEDChannel1() {
        super(Constants.LEDs.R_1_PWM_PORT, Constants.LEDs.G_1_PWM_PORT, Constants.LEDs.B_1_PWM_PORT);
    }
    private static LEDSubsystem instance;       // Static that stores the instance of class
    public static LEDSubsystem getInstance() {  // Method to allow calling this class and getting the single instance from anywhere, creating the instance if the first time.
        if (instance == null) {
            instance = new LEDChannel1();
        }
        return instance;
    }
}
