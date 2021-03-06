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

    public void run() {
        hopperMotor.set(ControlMode.PercentOutput, Constants.Intake.kHopperSpeed);
    }

    public void stop() {
        hopperMotor.set(ControlMode.PercentOutput, 0);
    }

    public void reverse() {
        hopperMotor.set(ControlMode.PercentOutput, -Constants.Intake.kHopperSpeed);
    }

}