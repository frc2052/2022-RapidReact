package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.Solenoids;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private Solenoid outSolenoid;
    private Solenoid inSolenoid;
    private boolean isArmOut;
    private VictorSPX intakeMotor;

    private double intakePct;
    private boolean isIntakeArmOut = false;

    public IntakeSubsystem() {
        outSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Solenoids.INTAKE_OUT_SOLENOID);
        inSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Solenoids.INTAKE_IN_SOLENOID);
        isArmOut = false;
        intakePct = 0;
        intakeMotor = new VictorSPX(MotorIDs.INTAKE_MOTOR);
    }

    //solenoid in = arm out
    public void intakeArmOut(){
        inSolenoid.set(true);
        outSolenoid.set(false);
        isIntakeArmOut = true;
    }

    //solenoid out = arm in
    public void intakeArmIn(){
        inSolenoid.set(false);
        outSolenoid.set(true);
        isIntakeArmOut = false;
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

    public void putToSmartDashboard() {
        SmartDashboard.putBoolean("Intake Arm Out", isIntakeArmOut);
    }
}