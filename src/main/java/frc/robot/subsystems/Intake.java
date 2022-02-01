package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private Solenoid outSolenoid;
    private Solenoid inSolenoid;
    private VictorSPX intakeMotor;
    
    public void intakeArmIn(){
        inSolenoid.set(true);
        outSolenoid.set(false);
    }
  
    public void intakeArmOut(){
        inSolenoid.set(false);
        outSolenoid.set(true);
    }
    //replaced "down" with "out" and "up" with "in" to be more intuitive for this year's robot

    public void intakeOn(){
        intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.kIntakeSpeed);
    }
   
    public void intakeReverse(){
        intakeMotor.set(ControlMode.PercentOutput, -Constants.Intake.kIntakeSpeed);
    }
   
    public void intakeStop(){
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }
}
