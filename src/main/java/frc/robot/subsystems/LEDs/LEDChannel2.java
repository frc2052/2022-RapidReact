package frc.robot.subsystems.LEDs;

import frc.robot.Constants;

public class LEDChannel2 extends LEDSubsystem {

    private LEDChannel2() {
        super(Constants.LEDs.R_2_PWM_PORT, Constants.LEDs.G_2_PWM_PORT, Constants.LEDs.B_2_PWM_PORT);
    }
    private static LEDSubsystem instance;       // Static that stores the instance of class
    public static LEDSubsystem getInstance() {  // Method to allow calling this class and getting the single instance from anywhere, creating the instance if the first time.
        if (instance == null) {
            instance = new LEDChannel2();
        }
        return instance;
    }
}
