package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;
import frc.robot.Constants.MotorIDs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
//    private Solenoid outSolenoid;
//    private Solenoid inSolenoid;
    private boolean isArmOut;
    private TalonSRX intakeMotor;

    private double intakePct;
    private boolean isIntakeArmOut = false;

    public IntakeSubsystem() {
//        outSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Solenoids.INTAKE_OUT_SOLENOID);
//        inSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Solenoids.INTAKE_IN_SOLENOID);
        isArmOut = false;
        intakePct = 0;
        intakeMotor = new TalonSRX(MotorIDs.INTAKE_MOTOR);
    }

    public void intakeArmIn(){
//        inSolenoid.set(true);
//        outSolenoid.set(false);
        isIntakeArmOut = false;
    }
  
    public void intakeArmOut(){
  //      inSolenoid.set(false);
  //      outSolenoid.set(true);
        isIntakeArmOut = true;
    }
    
    //intake takes in
    public void intakeOn(){
        intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.kIntakeSpeed);
        intakePct =  Constants.Intake.kIntakeSpeed;
    }

    //intake untakes in
    public void intakeReverse(){
        intakeMotor.set(ControlMode.PercentOutput, -Constants.Intake.kIntakeSpeed);
        intakePct = -Constants.Intake.kIntakeSpeed;
       
    }
   
    //intake stops
    public void intakeStop(){
        intakeMotor.set(ControlMode.PercentOutput, 0);
        intakePct = 0;
    }

    //so we know what's going on
    public double getIntakeSpeed() {
        return intakePct;
    }
    
    public boolean isArmOut() {
        return isArmOut;
    }

    public void putToSmartDashboard() {
        SmartDashboard.putBoolean("Intake Arm Out", isIntakeArmOut);
    }
}