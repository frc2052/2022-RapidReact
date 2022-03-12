package frc.robot.util.buttonbindings;

import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;

public class ButtonList {
    
    public enum ButtonCommands {
        VISION_SHOOT(null),
        SHOOT_LOW_GOAL(null),
        NON_VISION_SHOOT(null),
        MANUAL_SHOOT(null),
        RESET_GYRO(null),
        SHOOT_SINGLE(null),
        CLIMBER_RETRACT(null),
        TOGGLE_CLIMBER_ANGLE(null),
        CLIMBER_EXTEND(null),
        INTAKE_REVERSE(null),
        INTAKE_ON(null),
        CLIMBER_LIMIT_OVERRIDE(null),
        UNLOCK_CLIMBER(null),
        LOCK_CLIMBER(null);

        public Command command;

        private ButtonCommands(Command command) {
            this.command = command;
        }

        public void setCommand(Command command) {
            this.command = command;
        }
    }

    public enum ButtonActions {
        VISION_SHOOT(null),
        NON_VISION_SHOOT(null);

        public Button command;

        private ButtonActions(Button command) {
            this.command = command;
        }

        public void setCommand(Button command) {
            this.command = command;
        }
    }
}
