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
    private boolean isArmOut;

    public IntakeSubsystem() {
        armSolenoid = new DoubleSolenoid(
            Constants.Solenoids.COMPRESSOR_MODULE_ID, 
            PneumaticsModuleType.REVPH,
            Solenoids.INTAKE_IN_SOLENOID, 
            Solenoids.INTAKE_OUT_SOLENOID
        );
        intakeMotor = new TalonSRX(MotorIDs.INTAKE_MOTOR);
        isArmOut = armSolenoid.get() == Value.kReverse;
    }

    public void armIn(){
        armSolenoid.set(Value.kForward);
        isArmOut = false;
    }
  
    public void armOut(){
        armSolenoid.set(Value.kReverse);
        isArmOut = true;
    }

    public boolean isArmOut() {
        return isArmOut;
    }

    public void run(){
        intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.kIntakeSpeed);
    }

    public void reverse(){
        intakeMotor.set(ControlMode.PercentOutput, -Constants.Intake.kIntakeSpeed);  
    }
   
    public void stop(){
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Arm Out", isArmOut);
    }
}