package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Victor;

public class Intake {
    private Solenoid outSolenoid;
    private Solenoid inSolenoid;
    private boolean isArmOut;
    private VictorSPX intakeMotor;
   
    public void intakeArmIn(){
        inSolenoid.set(true);
        outSolenoid.set(false);
        isArmOut = false;
    }
  
    public void intakeArmOut(){
        inSolenoid.set(false);
        outSolenoid.set(true);
        isArmOut = true;
    }

    public void intakeOn(){

    }
   
    public void intakeReverse(){

    }
   
    public void intakeStop(){

    }
}
