package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.Solenoids;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final DoubleSolenoid armSolenoid;
    private TalonSRX intakeMotor;
    private boolean isIntakeArmOut;

    public IntakeSubsystem() {
        armSolenoid = new DoubleSolenoid(
            Constants.Solenoids.COMPRESSOR_MODULE_ID, 
            PneumaticsModuleType.REVPH,
            Solenoids.INTAKE_IN_SOLENOID, 
            Solenoids.INTAKE_OUT_SOLENOID
        );
        intakeMotor = new TalonSRX(MotorIDs.INTAKE_MOTOR);
        isIntakeArmOut = armSolenoid.get() == Value.kReverse;
    }

    public void intakeArmIn(){
        armSolenoid.set(Value.kForward);
        isIntakeArmOut = false;
    }
  
    public void intakeArmOut(){
        armSolenoid.set(Value.kReverse);
        isIntakeArmOut = true;
    }

    public boolean isIntakeArmOut() {
        return isIntakeArmOut;
    }

    public void intakeOn(){
        intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.kIntakeSpeed);
    }

    public void intakeReverse(){
        intakeMotor.set(ControlMode.PercentOutput, -Constants.Intake.kIntakeSpeed);  
    }
   
    public void intakeStop(){
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    public void putToSmartDashboard() {
        SmartDashboard.putBoolean("Intake Arm Out", isIntakeArmOut);
    }
}