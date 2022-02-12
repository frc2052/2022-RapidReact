package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import frc.robot.Constants;

public class ClawClimberSubsystem {
      private Solenoid closeASolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Solenoids.CLOSE_A_SOLENOID);
      private Solenoid openASolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Solenoids.OPEN_A_SOLENOID);
      private Solenoid closeBSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Solenoids.CLOSE_B_SOLENOID);;
      private Solenoid openBSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Solenoids.OPEN_B_SOLENOID);
      private TalonSRX armMotor = new TalonSRX(Constants.MotorIDs.ARM_MOTOR);
      private DigitalInput clawALimitSwitch = new DigitalInput(Constants.LimitSwitch.CLAW_A_LIMIT_SWITCH);
      private DigitalInput clawBLimitSwitch = new DigitalInput(Constants.LimitSwitch.CLAW_B_LIMIT_SWITCH);

      //Set the arms speed as it accends
      public void rotateArm(){
            armMotor.set(ControlMode.PercentOutput, Constants.Climber.CLIMBER_EXTENSION_SPEED);
           
      }
      //Set the arms speed as it descends 
      public void armReverse(){
            armMotor.set(ControlMode.PercentOutput, -Constants.Climber.CLIMBER_EXTENSION_SPEED);
        }
      //Stops the arm from moving 
        public void armStop(){
            armMotor.set(ControlMode.PercentOutput, 0);
        }

      public boolean getClawALimitSwitch(){
            return clawALimitSwitch.get() ;
      }
      public boolean getClawBLimitSwitch(){
            return clawBLimitSwitch.get();
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
