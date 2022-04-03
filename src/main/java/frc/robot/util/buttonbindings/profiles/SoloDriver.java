package frc.robot.util.buttonbindings.profiles;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer.ButtonCommands;

public class SoloDriver {
    public static void configureButtons(Joystick driveJoystick, Joystick turnJoystick, Joystick secondaryPannel) {

        new JoystickButton(driveJoystick, 1).whenHeld(ButtonCommands.VISION_SHOOT_ALL.command);
        new JoystickButton(driveJoystick, 2).whenPressed(ButtonCommands.INTAKE_TOGGLE.command);
        new JoystickButton(driveJoystick, 3).whenHeld(ButtonCommands.INTAKE_ON.command);
        new JoystickButton(driveJoystick, 4).whileHeld(ButtonCommands.NONVISION_SHOOT_ALL.command);
        new JoystickButton(driveJoystick, 5).whenHeld(ButtonCommands.NONVISION_SHOOT_LOW_GOAL.command);
        new JoystickButton(driveJoystick, 6);
        new JoystickButton(driveJoystick, 7);
        new JoystickButton(driveJoystick, 8);
        new JoystickButton(driveJoystick, 9);
        new JoystickButton(driveJoystick, 10);
        new JoystickButton(driveJoystick, 11);

        new JoystickButton(turnJoystick, 1).whenHeld(ButtonCommands.VISION_SHOOT_SINGLE.command);
        new JoystickButton(turnJoystick, 2);
        new JoystickButton(turnJoystick, 3);
        new JoystickButton(turnJoystick, 4);
        new JoystickButton(turnJoystick, 5);
        new JoystickButton(turnJoystick, 6);
        new JoystickButton(turnJoystick, 7);
        new JoystickButton(turnJoystick, 8);
        new JoystickButton(turnJoystick, 9);
        new JoystickButton(turnJoystick, 10);
        new JoystickButton(turnJoystick, 11);

        new JoystickButton(secondaryPannel, 1).whenPressed(ButtonCommands.INTAKE_TOGGLE.command);
        new JoystickButton(secondaryPannel, 2);
        new JoystickButton(secondaryPannel, 3).whenHeld(ButtonCommands.CLIMBER_RETRACT.command);
        new JoystickButton(secondaryPannel, 4).whenPressed(ButtonCommands.TOGGLE_CLIMBER_ANGLE.command);
        new JoystickButton(secondaryPannel, 5).whenHeld(ButtonCommands.CLIMBER_EXTEND.command);
        new JoystickButton(secondaryPannel, 6).whenHeld(ButtonCommands.INTAKE_REVERSE.command);
        new JoystickButton(secondaryPannel, 7).whenHeld(ButtonCommands.INTAKE_ON.command);
        new JoystickButton(secondaryPannel, 8).whenHeld(ButtonCommands.INTAKE_REVERSE.command);
        new JoystickButton(secondaryPannel, 9);
        new JoystickButton(secondaryPannel, 10);
        new JoystickButton(secondaryPannel, 11).whenPressed(ButtonCommands.UNLOCK_CLIMBER.command);
        new JoystickButton(secondaryPannel, 12).whenPressed(ButtonCommands.LOCK_CLIMBER.command);
    }
}
