package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants;

public class ClawClimberSubsystem {
      private Solenoid closeASolenoid;
      private Solenoid openASolenoid;
      private Solenoid closeBSolenoid;
      private Solenoid openBSolenoid;
      private Talon armMotor;

      // TODO - make motor that will spin the armm to a cretain point talonsrx
      public void rotateArm(){
            armMotor.set(ControlMode.PercentOutput, Constants..kIntakeSpeed);
      }
      public boolean getClawALimitSwitch(){
            return false;
      }
      public boolean getClawBLimitSwitch(){
            return false;
      }

      public void openClawA(){
            openASolenoid.set(true);
            closeASolenoid.set(false);
      }
      public void openClawB(){
            openBSolenoid.set(true);
            closeBSolenoid.set(false);
      }
      public void closeClawA(){
            openASolenoid.set(false);
            closeASolenoid.set(true);
      }
      public void closeClawB(){
            openBSolenoid.set(false);
            closeBSolenoid.set(true);
      }
}
