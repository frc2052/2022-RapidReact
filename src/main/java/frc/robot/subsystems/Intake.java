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
    //retracts arm
    public void intakeArmOut(){
        inSolenoid.set(false);
        outSolenoid.set(true);
    }
    //extends arm 
    //replaced "down" with "out" and "up" with "in" because it makes more sense


    public void intakeOn(){
        intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.kIntakeSpeed);
    }
    
    //turns the wheels of the intake
    
    public void intakeReverse(){
        intakeMotor.set(ControlMode.PercentOutput, -Constants.Intake.kIntakeSpeed);
    }
   
    //turns the wheels of the intake in the opposite direction


    public void intakeStop(){
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }
    //stops the wheels of the intake

}
