package frc.robot.util.buttonbindings.profiles;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.util.buttonbindings.ButtonAssignment;
import frc.robot.util.buttonbindings.ButtonList;
import frc.robot.util.buttonbindings.ButtonList.ButtonCommands;

public class Default {
    // private final Joystick driveJoystick;
    // private final Joystick turnJoystick;
    // private final Joystick secondaryPannel;

    // private final List<ButtonAssignment> driveJoystickButtons = new ArrayList<ButtonAssignment>();
    // private final List<ButtonAssignment> turnJoystickButtons = new ArrayList<ButtonAssignment>();
    // private final List<ButtonAssignment> secondaryPannelButtons = new ArrayList<ButtonAssignment>();

    public static void configureButtons(Joystick driveJoystick, Joystick turnJoystick, Joystick secondaryPannel) {

        new JoystickButton(driveJoystick, 1).whenPressed(ButtonCommands.VISION_SHOOT.command); // Vision Shoot
        new JoystickButton(driveJoystick, 2);
        new JoystickButton(driveJoystick, 3);
        new JoystickButton(driveJoystick, 4);
        new JoystickButton(driveJoystick, 5).whenHeld(ButtonCommands.SHOOT_LOW_GOAL.command);
        new JoystickButton(driveJoystick, 6);
        new JoystickButton(driveJoystick, 7);
        new JoystickButton(driveJoystick, 8);
        new JoystickButton(driveJoystick, 9);
        new JoystickButton(driveJoystick, 10);
        new JoystickButton(driveJoystick, 11);

    }

}
