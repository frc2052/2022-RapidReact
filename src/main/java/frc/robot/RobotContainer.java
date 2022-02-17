// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.*;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.FeedOneCargoLaunchCommand;
import frc.robot.commands.FeedTwoCargoLaunchCommand;
import frc.robot.commands.PrepareToLaunchCargoCommand;
import frc.robot.commands.IntakeArmInCommand;
import frc.robot.commands.IntakeArmOutCommand;
import frc.robot.commands.IntakeStopCommand;
import frc.robot.commands.PixyCamDriveCommand;
import frc.robot.commands.VisionDriveCommand;
import frc.robot.commands.climber.ExtendClimberCommand;
import frc.robot.commands.climber.RetractClimberCommand;
import frc.robot.commands.climber.StartClimbCommand;
import frc.robot.commands.climber.ToggleClimberSolenoidCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;
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
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem();
  private final DashboardControlsSubsystem dashboardControlsSubsystem = new DashboardControlsSubsystem(vision);
  private final TwoWheelFlySubsystem twoWheelFlySubsystem = new TwoWheelFlySubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final HopperSubsystem grassHopper = new HopperSubsystem();
  private final PneumaticsSubsystem pneumatics = new PneumaticsSubsystem();
  private final HookClimberSubsystem climberSubsystem = new HookClimberSubsystem();

  private final PixyCamSubsystem pixyCamSubsystem = new PixyCamSubsystem();

  private final Joystick driveJoystick = new Joystick(0);
  private final Joystick turnJoystick = new Joystick(1);
  private final Joystick secondaryPannel = new Joystick(2);
  
  private final JoystickButton visionDriveCommandSwitch = new JoystickButton(turnJoystick, 1);
  private final JoystickButton pixyDriveCommandSwitch = new JoystickButton(turnJoystick, 3);
  private final JoystickButton resetGyroButton = new JoystickButton(secondaryPannel, 1);
  private final JoystickButton intakeArmOutButton = new JoystickButton(driveJoystick, 2);
  private final JoystickButton intakeArmInButton = new JoystickButton(driveJoystick, 3);
  private final JoystickButton intakeStopButton = new JoystickButton(driveJoystick, 5);
  private final JoystickButton prepareToLaunch = new JoystickButton(secondaryPannel, 2);
  private final JoystickButton feedTwoCargoLaunch = new JoystickButton(secondaryPannel, 3);
  private final JoystickButton feedOneCargoLaunch = new JoystickButton(secondaryPannel, 4);
  private final UsbCameraSubsystem intakeCamera = new UsbCameraSubsystem();

  private final JoystickButton startClimbButton = new JoystickButton(secondaryPannel, 4);
  private final JoystickButton extendClimberButton = new JoystickButton(secondaryPannel, 5);
  private final JoystickButton retractClimberButton = new JoystickButton(secondaryPannel, 6);
  private final JoystickButton climberSolenoidToggleButton = new JoystickButton(secondaryPannel, 7);
  private final JoystickButton climberLockToggleButton = new JoystickButton(secondaryPannel, 8);

  // Slew rate limiters to make joystick inputs more gentle.
  // A value of .1 will requier 10 seconds to get from 0 to 1. It is calculated as 1/rateLimitPerSecond to go from 0 to 1
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter turnLimiter = new SlewRateLimiter(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
//    pixySub.setDefaultCommand(new PixyCamManualDriveCommand(pixySub));
    drivetrainSubsystem.setDefaultCommand(
      new DefaultDriveCommand(
            drivetrainSubsystem,
            () -> -modifyAxis(driveJoystick.getY(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(driveJoystick.getX(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(turnJoystick.getX(), turnLimiter) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            dashboardControlsSubsystem
		)
    );

    LEDSubsystem.getInstance();

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
    visionDriveCommandSwitch.whenHeld(
        new VisionDriveCommand( // Overrides the DefualtDriveCommand and uses VisionDriveCommand when the trigger on the turnJoystick is held.
        drivetrainSubsystem,
        () -> -modifyAxis(driveJoystick.getY(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driveJoystick.getX(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        vision,
        dashboardControlsSubsystem
      )
    );

    visionDriveCommandSwitch.whenReleased(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.TELEOP_DEFAULT)); // Probably a better way to do this...
    
    pixyDriveCommandSwitch.whenHeld(
      new PixyCamDriveCommand(
        drivetrainSubsystem,
        pixyCamSubsystem,
        () -> -modifyAxis(driveJoystick.getY(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driveJoystick.getX(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        dashboardControlsSubsystem
      )
    );

    resetGyroButton.whenPressed(() -> { this.resetGyro(); });
    
    startClimbButton.whenPressed(new StartClimbCommand(climberSubsystem));
    extendClimberButton.whenPressed(new ExtendClimberCommand(climberSubsystem));
    retractClimberButton.whenPressed(new RetractClimberCommand(climberSubsystem));
    climberSolenoidToggleButton.whenPressed(new ToggleClimberSolenoidCommand(climberSubsystem));
    climberLockToggleButton.whenPressed(() -> {
      if (climberSubsystem.isLocked()) {
        climberSubsystem.unlockClimber();
      } else {
        climberSubsystem.lockClimber();
      }
    });

    intakeStopButton.whenPressed(new IntakeStopCommand(intakeSubsystem, grassHopper));
    intakeArmOutButton.whenPressed(new IntakeArmOutCommand(intakeSubsystem, grassHopper));
    intakeArmInButton.whenPressed(new IntakeArmInCommand(intakeSubsystem, grassHopper));

    prepareToLaunch.whileHeld(new PrepareToLaunchCargoCommand(twoWheelFlySubsystem, indexerSubsystem, vision, grassHopper));
    feedTwoCargoLaunch.whileHeld(new FeedTwoCargoLaunchCommand(twoWheelFlySubsystem, indexerSubsystem));
    feedOneCargoLaunch.whileHeld(new FeedOneCargoLaunchCommand(twoWheelFlySubsystem, indexerSubsystem));


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
      case AUTO_TESTING:
      return new AutoTesting(drivetrainSubsystem, vision, intakeSubsystem, grassHopper);
      case TEST_AUTO_1:
        return new TestAuto1(drivetrainSubsystem);
      case SIMPLE_3_BALL:
        return new Simple3BallAuto(drivetrainSubsystem, vision);
      case THREE_BALL_DRIVE_AND_SHOOT:
        return new ThreeballDriveAndShoot(drivetrainSubsystem, vision);
      case LEFT_TERMINAL_3_BALL: 
        return new LeftTerminal3Cargo(drivetrainSubsystem, vision);
      case LEFT_2_BALL_1_DEFENSE:
        return new LeftDefenseAuto(drivetrainSubsystem, vision, twoWheelFlySubsystem, intakeSubsystem, indexerSubsystem, grassHopper);
      case MIDDLE_TERMINAL_3_BALL:
        return new MiddleTerminal3CargoAuto(drivetrainSubsystem, vision);
      case MIDDLE_TERMINAL_DEFENSE:
        return new MiddleLeftTerminalDefenseAuto(drivetrainSubsystem, vision, twoWheelFlySubsystem, intakeSubsystem, indexerSubsystem, grassHopper);
      case FIVE_BALL:
        return new RightFiveBallAuto(drivetrainSubsystem, vision, twoWheelFlySubsystem, intakeSubsystem, indexerSubsystem, grassHopper);
      case RIGHT_MIDDLE_5_BALL_1_DEFENSE:
        return new MiddleRight5BallDefenseAuto(drivetrainSubsystem, vision, twoWheelFlySubsystem, intakeSubsystem, indexerSubsystem, grassHopper);
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

  public void printToSmartDashboard() {
    drivetrainSubsystem.putToSmartDashboard();
    vision.putToSmartDashboard();
    intakeSubsystem.putToSmartDashboard();
    twoWheelFlySubsystem.putToSmartDashboard();

    // For Testing Velocity Calculations
    double reqProjectileVelocity = twoWheelFlySubsystem.calculateReqProjectileVelocity(vision.getXDistanceToUpperHub());
    SmartDashboard.putNumber("Required Projectile Velocity", reqProjectileVelocity);
    SmartDashboard.putNumber("Required Angular Velocity", reqProjectileVelocity / Constants.ShooterSub.FLYWHEEL_RADIUS_METERS);
    SmartDashboard.putNumber("Required RPM", twoWheelFlySubsystem.calculateReqShooterRPM(reqProjectileVelocity));
  }

  public void resetGyro() {
    drivetrainSubsystem.zeroGyroscope();
  }
}
