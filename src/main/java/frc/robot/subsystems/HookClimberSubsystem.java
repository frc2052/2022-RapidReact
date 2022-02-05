package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;

public class HookClimberSubsystem {
    
    private Solenoid forward1Solenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Solenoids.FORWARD_1_SOLENOID);
      private Solenoid backward1Solenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Solenoids.BACKWARD_1_SOLENOID);
      private Solenoid forward2Solenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Solenoids.FORWARD_2_SOLENOID);;
      private Solenoid backward2Solenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Solenoids.BACKWARD_2_SOLENOID);
      private TalonSRX extendArmMotor = new TalonSRX(Constants.MotorIDs.ARM_MOTOR);
//This extend the arm at a set speed
    public void extendArm(){
        extendArmMotor.set(ControlMode.PercentOutput, Constants.Arm.kArmSpeed);
}
// This retract the arm at a set speed
    public void retractArm(){
        extendArmMotor.set(ControlMode.PercentOutput, -Constants.Arm.kArmSpeed);
}
//Stops the arm from moving 
public void armStop(){
    extendArmMotor.set(ControlMode.PercentOutput, 0);
}// shifts arm 1 down
public void shiftArm1Down(){
    forward1Solenoid.set(false);
    backward1Solenoid.set(true);
}
// shifts arm 1 up
public void shiftArm1Up(){
    forward1Solenoid.set(true);
    backward1Solenoid.set(false);
}

// shifts arm 2 down
public void shiftArm2Down(){
    forward2Solenoid.set(false);
    backward2Solenoid.set(true);
}
// shifts arm 2 up
public void shiftArm2Up(){
    forward2Solenoid.set(true);
    backward2Solenoid.set(false);
}
}
