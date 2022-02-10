package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


    public class Intake extends SubsystemBase {
    private Solenoid outSolenoid;
    private Solenoid inSolenoid;
    private VictorSPX intakeMotor;
    private VictorSPX hopperMotor;

    private double intakePct;
    private boolean isIntakeArmOut = false;

    public void intakeArmIn(){
        inSolenoid.set(true);
        outSolenoid.set(false);
        isIntakeArmOut = false;
    }
  
    public void intakeArmOut(){
        inSolenoid.set(false);
        outSolenoid.set(true);
        isIntakeArmOut = true;
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

    public void putToSmartDashboard() {
        SmartDashboard.putBoolean("Intake Arm Out", isIntakeArmOut);
    }
}