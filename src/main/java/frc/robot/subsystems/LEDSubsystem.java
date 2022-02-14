package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import frc.robot.Constants;

public class LEDSubsystem {

    private static final CANifier canifer = new CANifier(Constants.LEDs.kCANiferPort);

    public static void onLoop() {
        canifer.setLEDOutput(255, LEDChannel.LEDChannelA);
    }
}
