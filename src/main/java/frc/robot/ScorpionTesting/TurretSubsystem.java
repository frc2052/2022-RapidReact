/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Copied from 2020 Scorpion for testing aiming using limelight

package frc.robot.ScorpionTesting;

import com.ctre.phoenix.motorcontrol.ControlMode;             //CTRE Phoenix Framework imports
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;   //Access to the smart dashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase;          //Regular for making subsystems
import frc.robot.Constants;                                   //Access to constants class
//import frc.robot.lib.CsvLogger;                               //Acces to CSV Logger

public class TurretSubsystem extends SubsystemBase {          //It truely extends subsystem base, normal for all subsystems
  private TalonSRX turretMotor;   //Declars Talon motor object from TalonSRX class
  private int currentPos = 0;     //Sets current position to an initial value of 0  *not used
  private double motorPower = 0;  //Sets the motor power to an initial value of 0   *not used
  
  private boolean isLinedUp;  //Declares isLinedUp boolean but doesn't set. It's value is currently null
 
  public TurretSubsystem() {  //Contructor method
    turretMotor = new TalonSRX(Constants.CopiedFromScorpion.Motors.kTurretMotorID);  //Sets turret Motor to a vlaue of a new TalonSRX motor and gets the id of it declared in Constants
    turretMotor.configFactoryDefault();                           //Resets motor to factory defaults
    turretMotor.setNeutralMode(NeutralMode.Brake);                //Sets neutral mode to Brake, which commonly electrilizes motor leads to reduce motion
    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);  //Configures feedback sensor to a QuadEncoder (what the motor product is) with a Pid Index of 0 (primary closed loop) and 10ms timeout
    turretMotor.setSelectedSensorPosition(0, 0, 10); //sets the sensor position to 0, Pid Index to 0, and timeoutMs to 10

    //CsvLogger.addLoggingFieldDouble("TurretTicks", "ticks", "getEncoderPos", this); //Csv logging stuff
    //CsvLogger.addLoggingFieldBoolean("TurretOnTarget", "", "getIsOnTarget", this);
  }


  public void resetEncoder() { //public method to reset the turretMotor's TalonSRX encoder
    turretMotor.setSelectedSensorPosition(0);
  }


  public void turnTurret(double power){ //method to turn the turret by turning the motor with an argument passed for the percent power.
    //motorPower = power;
    turretMotor.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() { //method that runs constantly
    // currentPos = turretMotor.getSelectedSensorPosition();
    // if (currentPos > Constants.Turret.kTurretMaxRight && motorPower > 0){
    //   motorPower = 0;
    // } else if (currentPos < Constants.Turret.kTurretMinLeft && motorPower < 0){
    //   motorPower = 0;
    // }
    // turretMotor.set(ControlMode.PercentOutput, motorPower);

    if(turretMotor.getSelectedSensorVelocity() < 0 && turretMotor.getSelectedSensorPosition() < Constants.CopiedFromScorpion.Turret.kTurretMinEncoderPos) {          //gets turrentMotor's sensor velocity and checks if it's less than 0 and is less than the minimul encoder position for the turret motor
      turretMotor.set(ControlMode.PercentOutput, 0);  // if true, sets the motor power to 0%
    } else if(turretMotor.getSelectedSensorVelocity() > 0 && turretMotor.getSelectedSensorPosition() > Constants.CopiedFromScorpion.Turret.kTurretMaxEncoderPos) {   //checks if the turrets velocity is greater than 0 and that the pid index of 0 is greater than the max encoder position 
      turretMotor.set(ControlMode.PercentOutput, 0);  
    }
  }
  
  // takes angle and drives until it gets to angle
  public void driveToPos(double angle) {  //method to turn turret to the ange given
    angle = -angle;     //inverts the value
    isLinedUp = false;  //sets default isLinedUp value to false
    //System.out.println(turretMotor.getSelectedSensorPosition());
    printEncoderPos();  //sends the turret's sensor position to the smartdashboard 
    //System.out.println("****************** TARGET ANGLE: " + angle);
    if(angle < 0 && turretMotor.getSelectedSensorPosition() < Constants.CopiedFromScorpion.Turret.kTurretMinEncoderPos) {          //checks if the angle given is less than 0 and the positon is less than the min encoder position
      //too far to the negative can't keep going
      System.out.println("TURRET TOO FAR NEGATIVE");  //prints that it's too far negative
      turretMotor.set(ControlMode.PercentOutput, 0);  //then stops the turret by reducing the power to 0
    } else if (angle > 0 && turretMotor.getSelectedSensorPosition() > Constants.CopiedFromScorpion.Turret.kTurretMaxEncoderPos) {  //does the same for if the position is greater than the max encoder position
      //too far to the positive can't keep going
      System.out.println("TURRET TOO FAR POSITIVE");
      turretMotor.set(ControlMode.PercentOutput, 0);
    } else {
      //System.out.println("TURRENT ANGLE OFFSET: " + angle);
      isLinedUp = Math.abs(angle) < 2;  //sets isLiedUp boolean to true if the absolute value of the angle is less than 2 (degrees)
      if(Math.abs(angle) < 0.5) {       // if the absolute value of the ange is less than 0.5, it stops the motor
        turretMotor.set(ControlMode.PercentOutput, 0);  
      } else {  //if not that value, it continues
        if (Math.abs(angle) >= 20) {  //if the ange is greater than 20, it sets isLinedUp to false
          isLinedUp = false;
          if(angle > 0) { //if it's greater than 0 it goes to the right by setting the percent output to .75, and less than (indicating neagtive direciton), makes the turret go left by setting power to -0.75 percent
            turretMotor.set(ControlMode.PercentOutput, .75);
          } else {
            turretMotor.set(ControlMode.PercentOutput, -.75);
          }
        } else if (Math.abs(angle) < 20) { //if the angle is less than 20 degrees, it's going to turn at a slower speed by multiplying the angle by .05 for a target speed
          isLinedUp = false;
          double targetSpeed = angle * .05;

          if(Math.abs(targetSpeed) < Constants.CopiedFromScorpion.Turret.kMinTurretSpeed) {  //checks if the target speed is less than the minimum turret speed, and if true sets the speed to the minimum turrent speed which is inverted if negative
            if (targetSpeed < 0) {
              targetSpeed = -Constants.CopiedFromScorpion.Turret.kMinTurretSpeed;
            } else {
              targetSpeed = Constants.CopiedFromScorpion.Turret.kMinTurretSpeed;
            }
          }

          turretMotor.set(ControlMode.PercentOutput, (targetSpeed )); //moves the turret at the found target speed
        }
      }
    }
  }

  public boolean getIsOnTarget() {  // a get method to check if the turret is lined up
    return isLinedUp;
  }

  public double getEncoderPos() {   // a get method for the encoder's position
    return turretMotor.getSelectedSensorPosition();
  }

  public double getTurretDegree() { // get method for the turrets angle in degrees by getting the amount of encoder ticks from the turretMotor using getSelectedSensorPosition(0) (0 being the Pid index)
    double ticks = turretMotor.getSelectedSensorPosition(0);  
    return ticks / Constants.CopiedFromScorpion.Turret.kTicksPerDegree;  // and deviding it by a ticks per degree conversion constant (4096 / 90)
  }

  public void printEncoderPos() {
    SmartDashboard.putNumber("TURRET ENCODER", turretMotor.getSelectedSensorPosition());
  }
}