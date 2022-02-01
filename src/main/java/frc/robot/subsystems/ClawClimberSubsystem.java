package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;

import frc.robot.Constants;

public class ClawClimberSubsystem {
      private Solenoid closeASolenoid;
      private Solenoid openASolenoid;
      private Solenoid closeBSolenoid;
      private Solenoid openBSolenoid;
      private TalonSRX armMotor;

      //Set the arms speed as it accends
      public void rotateArm(){
            armMotor.set(ControlMode.PercentOutput, Constants.Arm.kArmSpeed);
      }
      //Set the arms speed as it descends 
      public void armReverse(){
            armMotor.set(ControlMode.PercentOutput, -Constants.Arm.kArmSpeed);
        }
      //Stops the arm from moving 
        public void armStop(){
            armMotor.set(ControlMode.PercentOutput, 0);
        }

      public boolean getClawALimitSwitch(){
            return false;
      }
      public boolean getClawBLimitSwitch(){
            return false;
      }
      // Opens claw A
      public void openClawA(){
            openASolenoid.set(true);
            closeASolenoid.set(false);
      }
       // Opens claw B
      public void openClawB(){
            openBSolenoid.set(true);
            closeBSolenoid.set(false);
      }
       // Closes claw A
      public void closeClawA(){
            openASolenoid.set(false);
            closeASolenoid.set(true);
      }
      public void closeClawB(){
            openBSolenoid.set(false);
            closeBSolenoid.set(true);
      }
}
