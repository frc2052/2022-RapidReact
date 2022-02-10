package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.Solenoids;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;


public class IntakeSubsystem {
    private Solenoid outSolenoid;
    private Solenoid inSolenoid;
    private boolean isArmOut;
    private VictorSPX intakeMotor;
    private VictorSPX hopperMotor;

    private double intakePct;

    public IntakeSubsystem() {
        outSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Solenoids.INTAKE_OUT_SOLENOID);
        inSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Solenoids.INTAKE_IN_SOLENOID);
        isArmOut = false;
        intakePct = 0;
        intakeMotor = new VictorSPX(MotorIDs.INTAKE_MOTOR);
        hopperMotor = new VictorSPX(MotorIDs.HOPPER_MOTOR);
    }

    public void intakeArmIn(){
        inSolenoid.set(true);
        outSolenoid.set(false);
        isArmOut = false;
        boolean isArmOut = false;
    }
  
    public void intakeArmOut(){
        inSolenoid.set(false);
        outSolenoid.set(true);
        isArmOut = true;
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
