package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorIDs;

public class HopperSubsystem extends SubsystemBase {
    private TalonSRX hopperMotor;

    public HopperSubsystem() {
        hopperMotor = new TalonSRX(MotorIDs.HOPPER_MOTOR);
    }

    //the hopper goes
    public void hopperGo() {
        hopperMotor.set(ControlMode.PercentOutput, Constants.Intake.kHopperSpeed);
    }

    //the hopper stops
    public void hopperStop() {
        hopperMotor.set(ControlMode.PercentOutput, 0);
    }

    //the hopper goes in reverse
    public void hopperReverse() {
        hopperMotor.set(ControlMode.PercentOutput, -Constants.Intake.kHopperSpeed);
    }

}