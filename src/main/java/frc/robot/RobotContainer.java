// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterSub;
import frc.robot.auto.*;
import frc.robot.commands.climber.ExtendClimberCommand;
import frc.robot.commands.climber.RetractClimberCommand;
import frc.robot.commands.climber.ToggleClimberSolenoidCommand;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.drive.PixyCamDriveCommand;
import frc.robot.commands.drive.VisionDriveCommand;
import frc.robot.commands.intake.IntakeArmInCommand;
import frc.robot.commands.intake.IntakeArmOutCommand;
import frc.robot.commands.intake.IntakeArmToggleCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeReverseCommand;
import frc.robot.commands.intake.IntakeStopCommand;
import frc.robot.commands.shooter.TuneShooterCommand;
import frc.robot.commands.shooter.ShootCommand.ShootMode;
import frc.robot.commands.shooter.PrepareToLaunchCargoCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;
import frc.robot.util.ProjectileCalculator;
import frc.robot.util.vision.ShooterDistanceConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private DrivetrainSubsystem drivetrainSubsystem;
  private VisionSubsystem vision;
  private DashboardControlsSubsystem dashboardControlsSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private IndexerSubsystem indexerSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private HopperSubsystem grassHopper;
  private PneumaticsSubsystem pneumatics;
  private HookClimberSubsystem climberSubsystem;
  private UsbCameraSubsystem intakeCamera;
  private PixyCamSubsystem pixyCamSubsystem;

  private final Joystick driveJoystick = new Joystick(0);
  private final Joystick turnJoystick = new Joystick(1);
  private final Joystick secondaryPannel = new Joystick(2);
  
  private JoystickButton visionDriveCommandSwitch;
  private JoystickButton pixyDriveCommandSwitch;
  private JoystickButton resetGyroButton;

  private JoystickButton intakeArmToggleButton;
  private JoystickButton intakeInButton;
  private JoystickButton intakeReverseButton;

  private JoystickButton prepareToLaunch;
  private JoystickButton shootSingleButton;
  private JoystickButton shootAllButton;
  private JoystickButton extendClimberButton;
  private JoystickButton retractClimberButton;
  private JoystickButton climberSolenoidToggleButton;
  private JoystickButton climberLockButton;
  private JoystickButton climberUnlockButton;
  private JoystickButton tuneShooterButton;
  // Slew rate limiters to make joystick inputs more gentle.
  // A value of .1 will requier 10 seconds to get from 0 to 1. It is calculated as 1/rateLimitPerSecond to go from 0 to 1
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter turnLimiter = new SlewRateLimiter(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
//    pixySub.setDefaultCommand(new PixyCamManualDriveCommand(pixySub));
    init();
  }

  private void init() {
    vision = new VisionSubsystem();
    dashboardControlsSubsystem = new DashboardControlsSubsystem(vision, climberSubsystem);
    //intakeCamera = new UsbCameraSubsystem();
    pixyCamSubsystem = new PixyCamSubsystem();    

    // //The following subsystems have a dependency on CAN
    drivetrainSubsystem = new DrivetrainSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    indexerSubsystem = new IndexerSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    grassHopper = new HopperSubsystem();
    pneumatics = new PneumaticsSubsystem();
    climberSubsystem = new HookClimberSubsystem();
    //LEDSubsystem.getInstance();

    drivetrainSubsystem.setDefaultCommand(
      new DefaultDriveCommand(
        drivetrainSubsystem,
        () -> -modifyAxis(driveJoystick.getY(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driveJoystick.getX(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(turnJoystick.getX(), turnLimiter) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        dashboardControlsSubsystem
		  )
    );

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // visionDriveCommandSwitch = new JoystickButton(turnJoystick, 1);
    // pixyDriveCommandSwitch = new JoystickButton(turnJoystick, 3);
    resetGyroButton = new JoystickButton(driveJoystick, 11);

    intakeArmToggleButton = new JoystickButton(secondaryPannel, 1);
    intakeInButton = new JoystickButton(secondaryPannel, 7);
    intakeReverseButton = new JoystickButton(secondaryPannel, 6);

    prepareToLaunch = new JoystickButton(secondaryPannel, 2);
    shootSingleButton = new JoystickButton(turnJoystick, 1);
    shootAllButton = new JoystickButton(driveJoystick, 1);
    tuneShooterButton = new JoystickButton(driveJoystick, 8);
    
    // Climber Buttons
    extendClimberButton = new JoystickButton(secondaryPannel, 5);
    retractClimberButton = new JoystickButton(secondaryPannel, 3);

    climberSolenoidToggleButton = new JoystickButton(secondaryPannel, 4);
   
    climberUnlockButton = new JoystickButton(secondaryPannel, 11);
    climberLockButton = new JoystickButton(secondaryPannel, 12);

    // visionDriveCommandSwitch.whenHeld(
    //     new VisionDriveCommand( // Overrides the DefualtDriveCommand and uses VisionDriveCommand when the trigger on the turnJoystick is held.
    //     drivetrainSubsystem,
    //     () -> -modifyAxis(driveJoystick.getY(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //     () -> -modifyAxis(driveJoystick.getX(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //     vision,
    //     dashboardControlsSubsystem
    //   )
    // );

    // visionDriveCommandSwitch.whenReleased(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.TELEOP_DEFAULT)); // Probably a better way to do this...
    
    // pixyDriveCommandSwitch.whenHeld(
    //   new PixyCamDriveCommand(
    //     drivetrainSubsystem,
    //     pixyCamSubsystem,
    //     () -> -modifyAxis(driveJoystick.getY(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //     () -> -modifyAxis(driveJoystick.getX(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //     dashboardControlsSubsystem
    //   )
    // );

    resetGyroButton.whenPressed(() -> { this.resetGyro(); });
    
    // startClimbButton.whenPressed(new StartClimbCommand(climberSubsystem));
    extendClimberButton.whileHeld(new ExtendClimberCommand(climberSubsystem));
    retractClimberButton.whileHeld(new RetractClimberCommand(climberSubsystem));
    climberSolenoidToggleButton.whenPressed(new ToggleClimberSolenoidCommand(climberSubsystem));

    climberUnlockButton.whenPressed(() -> { climberSubsystem.unlockClimber(); });
    climberLockButton.whenPressed(() -> { climberSubsystem.lockClimber(); });

    intakeArmToggleButton.whenPressed(new IntakeArmToggleCommand(intakeSubsystem, indexerSubsystem, grassHopper));
    intakeInButton.whileHeld(new IntakeInCommand(intakeSubsystem, indexerSubsystem, grassHopper));
    intakeReverseButton.whileHeld(new IntakeReverseCommand(intakeSubsystem, grassHopper));

    // prepareToLaunch.whileHeld(new PrepareToLaunchCargoCommand(shooterSubsystem, indexerSubsystem, vision, grassHopper));
    shootSingleButton.whileHeld(
      new ParallelCommandGroup(
        new ShootCommand(ShootMode.SHOOT_SINGLE, shooterSubsystem, indexerSubsystem, vision),
        new VisionDriveCommand( // Overrides the DefualtDriveCommand and uses VisionDriveCommand when the trigger on the turnJoystick is held.
          drivetrainSubsystem,
          () -> -modifyAxis(driveJoystick.getY(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
          () -> -modifyAxis(driveJoystick.getX(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
          vision,
          dashboardControlsSubsystem
        )
      )
    );
    shootAllButton.whileHeld(
      new ParallelCommandGroup(
        new ShootCommand(ShootMode.SHOOT_ALL, shooterSubsystem, indexerSubsystem, vision),
        new VisionDriveCommand( // Overrides the DefualtDriveCommand and uses VisionDriveCommand when the trigger on the turnJoystick is held.
          drivetrainSubsystem,
          () -> -modifyAxis(driveJoystick.getY(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
          () -> -modifyAxis(driveJoystick.getX(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
          vision,
          dashboardControlsSubsystem
        )
      )
    );

    tuneShooterButton.whileHeld(new TuneShooterCommand(shooterSubsystem, indexerSubsystem, intakeSubsystem, grassHopper));
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
      return new AutoTesting(drivetrainSubsystem, vision, shooterSubsystem, intakeSubsystem, grassHopper, indexerSubsystem);
      case TEST_AUTO_1:
        return new TestAuto1(drivetrainSubsystem);
      case SIMPLE_3_BALL:
        return new Simple3BallAuto(drivetrainSubsystem, vision, shooterSubsystem, intakeSubsystem, indexerSubsystem, grassHopper);
      case THREE_BALL_DRIVE_AND_SHOOT:
        return new ThreeballDriveAndShoot(drivetrainSubsystem, vision);
      case LEFT_TERMINAL_3_BALL: 
        return new LeftTerminal3Cargo(drivetrainSubsystem, vision);
      case LEFT_2_BALL_1_DEFENSE:
        return new LeftDefenseAuto(drivetrainSubsystem, vision, shooterSubsystem, intakeSubsystem, indexerSubsystem, grassHopper);
      case MIDDLE_TERMINAL_3_BALL:
        return new MiddleRightTerminal3CargoAuto(drivetrainSubsystem, vision, shooterSubsystem, intakeSubsystem, indexerSubsystem, grassHopper);
      case MIDDLE_TERMINAL_DEFENSE:
        return new MiddleLeftTerminalDefenseAuto(drivetrainSubsystem, vision, shooterSubsystem, intakeSubsystem, indexerSubsystem, grassHopper);
      case FIVE_BALL:
        return new RightFiveBallAuto(drivetrainSubsystem, vision, shooterSubsystem, intakeSubsystem, indexerSubsystem, grassHopper);
      case RIGHT_MIDDLE_5_BALL_1_DEFENSE:
        return new MiddleRight5BallDefenseAuto(drivetrainSubsystem, vision, shooterSubsystem, intakeSubsystem, indexerSubsystem, grassHopper);
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
    if (drivetrainSubsystem != null) {
      drivetrainSubsystem.putToSmartDashboard();
    }
    if (vision != null) {
      vision.putToSmartDashboard();
    }
    if (intakeSubsystem != null) {
      intakeSubsystem.putToSmartDashboard();
    }
    if (climberSubsystem != null) {
      climberSubsystem.putToSmartDashboard();
    }
    if (shooterSubsystem != null) {
      shooterSubsystem.putToSmartDashboard();
      
      // For Testing Velocity Calculations
      double reqProjectileVelocity = ProjectileCalculator.calculateReqProjectileVelocity(vision.getXDistanceToUpperHub());
      SmartDashboard.putNumber("Required Projectile Velocity", reqProjectileVelocity);
      SmartDashboard.putNumber("Required Angular Velocity", reqProjectileVelocity / Constants.ShooterSub.FLYWHEEL_RADIUS_METERS);
      SmartDashboard.putNumber("Required RPM", ProjectileCalculator.calculateReqShooterRPM(reqProjectileVelocity));
    }
  }

  public void resetGyro() {
    drivetrainSubsystem.zeroGyroscope();
  }
}
