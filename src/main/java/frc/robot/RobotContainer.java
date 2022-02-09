// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.auto.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem();
  private final DashboardControlsSubsystem dashboardControlsSubsystem = new DashboardControlsSubsystem(vision);
  private final Intake intake = new Intake();
  private final TwoWheelFlySubsystem shooter = new TwoWheelFlySubsystem();
  private final IndexerSubsystem indexer = new IndexerSubsystem();

  private final Joystick m_driveJoystick = new Joystick(0);
  private final Joystick m_turnJoystick = new Joystick(1);
  private final Joystick m_secondaryPannel = new Joystick(2);

  private final JoystickButton driveCommandSwitch = new JoystickButton(m_turnJoystick, 1);
  private final JoystickButton resetGyroButton = new JoystickButton(m_secondaryPannel, 1);

  // Slew rate limiters to make joystick inputs more gentle.
  // A value of .1 will requier 10 seconds to get from 0 to 1. It is calculated as 1/rateLimitPerSecond to go from 0 to 1
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter turnLimiter = new SlewRateLimiter(2);

  //private final UsbCameraSubsystem m_intakeCamera = new UsbCameraSubsystem(); // Commented out for now to make Limelight stream work

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drivetrainSubsystem.setDefaultCommand(
      new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> -modifyAxis(m_driveJoystick.getY(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_driveJoystick.getX(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_turnJoystick.getX(), turnLimiter) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        dashboardControlsSubsystem
      )
    );

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driveCommandSwitch.whenHeld(
      new VisionDriveCommand( // Overrides the DefualtDriveCommand and uses VisionDriveCommand when the trigger on the turnJoystick is held.
        m_drivetrainSubsystem,
        () -> -modifyAxis(m_driveJoystick.getY(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_driveJoystick.getX(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        vision,
        dashboardControlsSubsystem
      )
    );

    // Button to reset gyro at any point to make resetting in teleop easier and possible correct for potential gyro drift.
    resetGyroButton.whenPressed(() -> { this.resetGyro(); }, m_drivetrainSubsystem); // Uses a lambda as a Runnable to call this class's resetGyro method, and requires m_drivetrainSubsystem.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Uses options sent to the SmartDashboard with AutoSelector, finds the selected option, and returns a new instance of the desired Auto command.
    // Go ahead and hover over auto constructors for each auto's detailed description.
    switch(dashboardControlsSubsystem.getSelectedAuto()) {
      case AUTO_TESTING:
      return new AutoTesting(m_drivetrainSubsystem, vision, intake);
      case TEST_AUTO_1:
        return new TestAuto1(m_drivetrainSubsystem);
      case SIMPLE_3_BALL:
        return new Simple3BallAuto(m_drivetrainSubsystem, vision);
      case THREE_BALL_DRIVE_AND_SHOOT:
        return new ThreeballDriveAndShoot(m_drivetrainSubsystem, vision);
      case LEFT_TERMINAL_3_BALL: 
        return new LeftTerminal3Cargo(m_drivetrainSubsystem, vision); 
      case RIGHT_MIDDLE_5_BALL_1_DEFENSE:
        return new MiddleRight5BallDefenseAuto(m_drivetrainSubsystem, vision, shooter, intake, indexer);
      case LEFT_2_BALL_1_DEFENSE:
        return new LeftDefenseAuto(m_drivetrainSubsystem, vision);
      case MIDDLE_TERMINAL_3_BALL:
        return new MiddleTerminal3CargoAuto(m_drivetrainSubsystem, vision);
      case MIDDLE_TERMINAL_DEFENSE:
        return new MiddleLeftTerminalDefenseAuto(m_drivetrainSubsystem, vision, intake);
      case FIVE_BALL:
        return new FiveBallDreamAuto(m_drivetrainSubsystem, vision);
      default:
        break;
    }

    System.err.println("NO VALID AUTO SELECTED");
    return null;
  }

  // This code borrowed from the SwerverDriveSpecialist Sample code
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  // This code borrowed from the SwerverDriveSpecialist Sample code
  private static double modifyAxis(double value, SlewRateLimiter limiter) {
    // Deadband
    value = deadband(value, 0.1);
    // Square the axis for finer control at lower values
    value = limiter.calculate(Math.copySign(value * value, value));
    
    return value;
  }

  public void addSelectorsToSmartDashboard() {
    dashboardControlsSubsystem.addSelectorsToSmartDashboard();
  }

  public void checkSmartDashboardControls() {
    dashboardControlsSubsystem.checkSmartDashboardControls();
  }

  public void printToSmartDashboard() {
    m_drivetrainSubsystem.putToSmartDashboard();
    vision.putToSmartDashboard();
  }

  public void resetGyro() {
    m_drivetrainSubsystem.zeroGyroscope();
  }
}
