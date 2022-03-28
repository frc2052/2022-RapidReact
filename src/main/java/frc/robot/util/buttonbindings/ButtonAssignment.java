package frc.robot.util.buttonbindings;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ButtonAssignment {
    private JoystickButton joystickButton;
    private Joystick joystick;
    private int buttonNumber;

    public ButtonAssignment(Joystick joystick, int buttonNumber) {
        this.joystick = joystick;
        this.buttonNumber = buttonNumber;
        this.joystickButton = new JoystickButton(joystick, buttonNumber);
    } 

    public JoystickButton getJoystickButton() {
        return joystickButton;
    }    

    public Joystick getJoystick() {
        return joystick;
    }

    public int getButtonNumber() {
        return buttonNumber;
    }
}
