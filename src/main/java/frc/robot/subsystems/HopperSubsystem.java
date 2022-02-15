package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorIDs;

public class HopperSubsystem extends SubsystemBase {
    private VictorSPX hopperMotor;

    public HopperSubsystem() {
        hopperMotor = new VictorSPX(MotorIDs.HOPPER_MOTOR);
    }

    public void hopperGo() {
        hopperMotor.set(ControlMode.PercentOutput, Constants.Intake.kIntakeSpeed);
    }

    public void hopperStop() {
        hopperMotor.set(ControlMode.PercentOutput, 0);
    }

    public void hopperReverse() {
        hopperMotor.set(ControlMode.PercentOutput, -Constants.Intake.kIntakeSpeed);
    }

}