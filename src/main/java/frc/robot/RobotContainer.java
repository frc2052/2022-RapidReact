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
  private final TwoWheelFlySubsystem twoWheelFlySubsystem = new TwoWheelFlySubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final PneumaticsSubsystem pnuematics = new PneumaticsSubsystem();

  private final PixyCamSubsystem pixySub = new PixyCamSubsystem();
  private final PixyCamManualDriveCommand pixyCmd = new PixyCamManualDriveCommand(pixySub);

  private final Joystick m_driveJoystick = new Joystick(0);
  private final Joystick m_turnJoystick = new Joystick(1);
  
  private final JoystickButton driveCommandSwitch = new JoystickButton(m_turnJoystick, 1);
  private final JoystickButton resetGyroButton = new JoystickButton(m_secondaryPannel, 1);
  private final JoystickButton intakeArmOutButton = new JoystickButton(m_driveJoystick, 2);
  private final JoystickButton intakeArmInButton = new JoystickButton(m_driveJoystick, 3);
  private final JoystickButton intakeStopButton = new JoystickButton(m_driveJoystick, 5);
 
  private final UsbCameraSubsystem m_intakeCamera = new UsbCameraSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
//    pixySub.setDefaultCommand(new PixyCamManualDriveCommand(pixySub));
    m_drivetrainSubsystem.setDefaultCommand(
      new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_driveJoystick.getY(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_driveJoystick.getX(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_turnJoystick.getX(), turnLimiter) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            dashboardControlsSubsystem
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
      new VisionDriveCommand( // Overrides the DefualtDriveCommand and uses VisionDriveCommand when the trigger on the turnJoystick is held.
      m_drivetrainSubsystem,
      () -> -modifyAxis(m_driveJoystick.getY(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(m_driveJoystick.getX(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      vision,
      dashboardControlsSubsystem
    resetGyroButton.whenPressed(new ResetGyroCommand(m_drivetrainSubsystem));
    
    resetGyroButton.whenPressed(new ResetGyroCommand(m_drivetrainSubsystem)); // TEMP to reset the gyro using a button on the secondary pannel to make resetting in teleop easier, should be moved to a Shuffleboard virtual toggle
    intakeStopButton.whenPressed(new IntakeStop(intakeSubsystem));
    intakeArmOutButton.whenPressed(new IntakeArmOut(intakeSubsystem));
    intakeArmInButton.whenPressed(new IntakeArmIn(intakeSubsystem));

    prepareToLaunch.whileHeld(new PrepareToLaunchCargoCommand(indexerSubsystem, twoWheelFlySubsystem, intake));
    feedCargoLaunch.whileHeld(new FeedCargoLaunchCommand(twoWheelFlySubsystem, indexerSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
//    return pixyCmd;
    // Uses options sent to the SmartDashboard with AutoSelector, finds the selected option, and returns a new instance of the desired Auto command.
    switch(dashboardControlsSubsystem.getSelectedAuto()) {
      case TEST_AUTO_1:         // Test Auto that currently just moves slow and tests swerve drive functions.
        return new TestAuto1(m_drivetrainSubsystem);
      case SIMPLE_3_BALL:       // 3 Ball Auto using the two closest cargo near the tarmac.
        return new Simple3BallAuto(m_drivetrainSubsystem, vision);
      
      case THREE_BALL_TERMINAL: // TODO 3 Ball Auto using the closest cargo to the robot and the cargo positioned near the terminal.
        break;
      case FOUR_BALL:           // TODO Uses the preloaded cargo and 3 closest positioned around us (potential 5 ball auto if scored by human player).
        break;
      case FRONT_INTAKE_3_BALL:       // 3 Ball Auto using the two closest cargo near the tarmac.
        return new Simple3BallAutoFrontIntake(m_drivetrainSubsystem, vision);
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
    value = deadband(value, 0.05);

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
