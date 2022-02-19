package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.Solenoids;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final Solenoid outSolenoid;
    private final Solenoid inSolenoid;
    private boolean isArmOut;
    private TalonSRX intakeMotor;

    private double intakePct;
    private boolean isIntakeArmOut = false;

    public IntakeSubsystem() {
        outSolenoid = new Solenoid(
            Constants.Solenoids.COMPRESSOR_MODULE_ID, 
            PneumaticsModuleType.REVPH, 
            Solenoids.INTAKE_OUT_SOLENOID
        );
        inSolenoid = new Solenoid(
            Constants.Solenoids.COMPRESSOR_MODULE_ID, 
            PneumaticsModuleType.REVPH, 
            Solenoids.INTAKE_IN_SOLENOID
        );
        isArmOut = false;
        intakePct = 0;
        intakeMotor = new TalonSRX(MotorIDs.INTAKE_MOTOR);
    }

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

    public boolean isIntakeArmOut() {
        return isIntakeArmOut;
    }

    public void intakeOn(){
        intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.kIntakeSpeed);
        intakePct =  Constants.Intake.kIntakeSpeed;
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