// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;
//  private PixyCamSubsystem pixy;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    robotContainer.initializeAutonomousCommand();
//    robotContainer.addSelectorsAndCommandButtonsToSmartDashboard();

    LEDSubsystem.setBothChannelModes(LEDStatusMode.TEST_MODE);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    robotContainer.init();

    robotContainer.resetGyro();
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
    LEDSubsystem.setBothChannelModes(LEDStatusMode.AUTONOMOUS_DEFAULT);

    robotContainer.resetRobot();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
//    Rotation2d angle = pixy.angleToBall(BallColor.BLUE);
//		SmartDashboard.putString("Pixyblock.Angle", (angle != null) ? angle.toString() : "-------");

  //   System.err.println("AUTO");
  //   Block b = pixy.getBiggestBlock(BallColor.BLUE);
  //   if (b == null) {
  //     System.err.println("****************BLOCK IS NULL**********************");
  //   } else {
  //     System.err.println("**************BLOCK SIZE " + b.getHeight());
  //   }
  //  {
  //   System.err.println("AUTO");
  //   Block r = pixy.getBiggestBlock(BallColor.RED);
  //   if (r == null) {
  //     System.err.println("****************BLOCK IS NULL**********************");
  //   } else {
  //     System.err.println("**************BLOCK SIZE " + r.getHeight());
  //   }
  // }

  }

  @Override
  public void teleopInit() {
    robotContainer.init();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    LEDSubsystem.setBothChannelModes(LEDStatusMode.TELEOP_DEFAULT);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    robotContainer.resetGyro();
    LEDSubsystem.setBothChannelModes(LEDStatusMode.TEST_MODE);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
