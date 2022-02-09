
package frc.robot.commands;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PixyCamSubsystem;
import frc.robot.subsystems.PixyCamSubsystem.BallColor;
//import frc.robot.subsystems.PixyCamSubsystem.PixyBlock;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PixyCamManualDriveCommand extends CommandBase {
    private PixyCamSubsystem pixyCam;
  //private DriveTrainSubsystem driveTrain;
  //private Joystick tankJoy;

  /*
   * Creates a new PixyCamDrive.
   */

  public PixyCamManualDriveCommand(PixyCamSubsystem pixy) {
      pixyCam = pixy;
    // Use addRequirements() here to declare subsystem dependencies.
    //driveTrain = drive;
    //tankJoy = joystick;
    addRequirements(pixy);
  }

  /*
  public void getBallPosition() { 
    double xOffset;

//    System.out.println("Read Pixycam");
    ArrayList<PixyBlock> list = pixyCam.read();//list of objects that mach the color that i config the camera to do

    if (list == null) {
//      System.err.println("******************* NULL *******************");
    }    
    else if (list != null && list.size() > 0){ //the camera sees an object it is true. 
      
      System.err.println("FOUND SOMETHING");
        Collections.sort(list, new Comparator<PixyBlock>() { //sort objects so that the smallest objects are first in the list
            @Override
            public int compare(PixyBlock first, PixyBlock second) {
                return first.height - second.height;
            }
      });

      Collections.reverse(list);//reverses the list order
    } else {      
//      System.out.println("incoming list empty");//list is empty
      SmartDashboard.putNumber("PixyOffset", -998);
    }

    if(list !=null && list.size() > 0){//same as line 35 
      PixyBlock biggestBall = list.get(0); //gets the biggest object in the list
       xOffset = biggestBall.centerX - 155;//centers biggest object, range 0 - 315
      if (xOffset > 100){ //turn the turn target to between 0 and 100, if you are over 100, just target 100
        xOffset = 100;
      } else if (xOffset < -100){
        xOffset = -100;
      }

      SmartDashboard.putNumber("PixyOffset", xOffset);
      

      //double turnPercent = -xOffset/100.0;
     // double turnSpeed = Constants.PixyCamDriveConstants.turnSpeed * turnPercent;
     // driveTrain.arcadeDrive(-tankJoy.getY(), turnSpeed);
    //} else {
      //driveTrain.arcadeDrive(0, 0);
    }

  }
*/
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //pixyCam.getBiggestBlock(BallColor.BLUE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}