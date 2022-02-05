package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
//what is this for? 

public class Intake {
    private Solenoid outSolenoid;
    private Solenoid inSolenoid;
    private VictorSPX intakeMotor;
    private VictorSPX hopperMotor;

    private double intakePct;

    public void intakeArmIn(){
        inSolenoid.set(true);
        outSolenoid.set(false);
    }
  
    public void intakeArmOut(){
        inSolenoid.set(false);
        outSolenoid.set(true);
    }

    public void intakeOn(){
        intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.kIntakeSpeed);
        intakePct =  Constants.Intake.kIntakeSpeed;
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

    public void intakeReverse(){
        intakeMotor.set(ControlMode.PercentOutput, -Constants.Intake.kIntakeSpeed);
        intakePct = -Constants.Intake.kIntakeSpeed;
       
    }
   
    public void intakeStop(){
        intakeMotor.set(ControlMode.PercentOutput, 0);
        intakePct = 0;
    }

    public double getIntakeSpeed() {
        return intakePct;
    }
}
