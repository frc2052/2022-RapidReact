// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.auto.AutoTesting;
import frc.robot.auto.LeftDefenseAuto;
import frc.robot.auto.LeftTerminal3Cargo;
import frc.robot.auto.MiddleLeft3BallTerminalDefenseAuto;
import frc.robot.auto.MiddleLeftTerminalDefenseAuto;
import frc.robot.auto.MiddleRight5BallDefenseAuto;
import frc.robot.auto.MiddleRightTerminal3CargoAuto;
import frc.robot.auto.OneBallAuto;
import frc.robot.auto.RightFiveBallAuto;
import frc.robot.auto.Simple3BallAuto;
import frc.robot.auto.TestAuto1;
import frc.robot.auto.ThreeballDriveAndShoot;

import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.drive.ProfiledPIDTurnInPlaceCommand;
import frc.robot.commands.drive.VisionDriveCommand;
import frc.robot.commands.climber.ExtendClimberCommand;
import frc.robot.commands.climber.RetractClimberCommand;
import frc.robot.commands.climber.ToggleClimberSolenoidCommand;
import frc.robot.commands.intake.IntakeArmToggleCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeReverseCommand;
import frc.robot.commands.shooter.TuneShooterCommand;
import frc.robot.commands.shooter.ShootCommand.ShootMode;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.ShootLowCommand;
import frc.robot.subsystems.DashboardControlsSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import frc.robot.util.ProjectileCalculator;
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
  private DrivetrainSubsystem drivetrain;
  private VisionSubsystem vision;
  private DashboardControlsSubsystem dashboardControlsSubsystem;
  private ShooterSubsystem shooter;
  private IndexerSubsystem indexer;
  private IntakeSubsystem intake;
  private HopperSubsystem hopper;
  private PneumaticsSubsystem pneumatics;
  private HookClimberSubsystem climber;

  private final Joystick driveJoystick = new Joystick(0);
  private final Joystick turnJoystick = new Joystick(1);
  private final Joystick secondaryPannel = new Joystick(2);
  
  private JoystickButton resetGyroButton;

  private JoystickButton intakeArmToggleButton;
  private JoystickButton intakeInButton;
  private JoystickButton intakeReverseButton;

  private JoystickButton shootSingleButton;
  private JoystickButton shootAllButton;
  private JoystickButton shootLowGoalButton;
  private JoystickButton extendClimberButton;
  private JoystickButton retractClimberButton;
  private JoystickButton climberSolenoidToggleButton;
  private JoystickButton climberLockButton;
  private JoystickButton climberUnlockButton;
  private JoystickButton tuneShooterButton;

  private JoystickButton pidTestingButton;

  // Slew rate limiters to make joystick inputs more gentle.
  // A value of .1 will requier 10 seconds to get from 0 to 1. It is calculated as 1/rateLimitPerSecond to go from 0 to 1
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter turnLimiter = new SlewRateLimiter(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // pixySub.setDefaultCommand(new PixyCamManualDriveCommand(pixySub));
    init();
  }

  private void init() {
    vision = new VisionSubsystem();
    dashboardControlsSubsystem = new DashboardControlsSubsystem(vision, climber);
    //intakeCamera = new UsbCameraSubsystem();  

    // //The following subsystems have a dependency on CAN
    drivetrain = new DrivetrainSubsystem();
    shooter = new ShooterSubsystem();
    indexer = new IndexerSubsystem();
    intake = new IntakeSubsystem();
    hopper = new HopperSubsystem();
    pneumatics = new PneumaticsSubsystem();
    climber = new HookClimberSubsystem();
    //LEDSubsystem.getInstance();

    drivetrain.setDefaultCommand(
      new DefaultDriveCommand(
        drivetrain,
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
    // Drivetrain Button Bindings
    resetGyroButton = new JoystickButton(driveJoystick, 11);

    // Intake Buttons Bindings
    intakeArmToggleButton = new JoystickButton(secondaryPannel, 1);
    intakeInButton = new JoystickButton(secondaryPannel, 7);
    intakeReverseButton = new JoystickButton(secondaryPannel, 6);

    // Shooter Buttons Bindings
    shootSingleButton = new JoystickButton(turnJoystick, 1);
    shootAllButton = new JoystickButton(driveJoystick, 1);
    tuneShooterButton = new JoystickButton(driveJoystick, 8);
    shootLowGoalButton = new JoystickButton(driveJoystick, 5);
    
    // Climber Buttons Bindings
    extendClimberButton = new JoystickButton(secondaryPannel, 5);
    retractClimberButton = new JoystickButton(secondaryPannel, 3);
    climberSolenoidToggleButton = new JoystickButton(secondaryPannel, 4);
    climberUnlockButton = new JoystickButton(secondaryPannel, 11);
    climberLockButton = new JoystickButton(secondaryPannel, 12);

    pidTestingButton = new JoystickButton(secondaryPannel, 2);

    // pixyDriveCommandSwitch.whenHeld(
    //   new PixyCamDriveCommand(
    //     drivetrain,
    //     pixyCamSubsystem,
    //     () -> -modifyAxis(driveJoystick.getY(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //     () -> -modifyAxis(driveJoystick.getX(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //     dashboardControlsSubsystem
    //   )
    // );

    // Drivetrain Button Command Bindings
    resetGyroButton.whenPressed(() -> { this.resetGyro(); });
    
    // Intake Button Command Bindings
    intakeArmToggleButton.whenPressed(new IntakeArmToggleCommand(intake, indexer, hopper));
    intakeInButton.whileHeld(new IntakeInCommand(intake, indexer, hopper));
    intakeReverseButton.whileHeld(new IntakeReverseCommand(intake, hopper));

    // Shooter Button Command Bindings
    shootSingleButton.whileHeld(
      new ParallelCommandGroup(
        new ShootCommand(ShootMode.SHOOT_SINGLE, shooter, indexer, hopper, vision),
        new VisionDriveCommand( // Overrides the DefualtDriveCommand and uses VisionDriveCommand when the trigger on the turnJoystick is held.
          drivetrain,
          () -> -modifyAxis(driveJoystick.getY(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
          () -> -modifyAxis(driveJoystick.getX(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
          vision,
          dashboardControlsSubsystem
        )
      )
    );
    shootAllButton.whileHeld(
      new ParallelCommandGroup(
        new ShootCommand(ShootMode.SHOOT_ALL, shooter, indexer, hopper, vision),
        new VisionDriveCommand( // Overrides the DefualtDriveCommand and uses VisionDriveCommand when the trigger on the turnJoystick is held.
          drivetrain,
          () -> -modifyAxis(driveJoystick.getY(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
          () -> -modifyAxis(driveJoystick.getX(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
          vision,
          dashboardControlsSubsystem
        )
      )
    );
    tuneShooterButton.whileHeld(new TuneShooterCommand(shooter, indexer, intake, hopper));

    shootLowGoalButton.whileHeld(new ShootLowCommand(shooter, indexer));

    // Climber Button Command Bindings
    extendClimberButton.whileHeld(new ExtendClimberCommand(climber));
    retractClimberButton.whileHeld(new RetractClimberCommand(climber));
    climberSolenoidToggleButton.whenPressed(new ToggleClimberSolenoidCommand(climber));
    climberUnlockButton.whenPressed(() -> { climber.unlockClimber(); });
    climberLockButton.whenPressed(() -> { climber.lockClimber(); });

    // TODO: Delete this when done
    pidTestingButton.whenPressed(new ProfiledPIDTurnInPlaceCommand(drivetrain, () -> { return Rotation2d.fromDegrees(180); }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Uses options sent to the SmartDashboard with AutoSelector, finds the selected option, and returns a new instance of the desired Auto command.
    switch(dashboardControlsSubsystem.getSelectedAuto()) {
      case AUTO_TESTING:
        return new AutoTesting(drivetrain, vision, shooter, intake, hopper, indexer, climber);
      case ONE_BALL:
        return new OneBallAuto(drivetrain, vision, shooter, indexer, hopper, climber);
      case SIMPLE_3_BALL:
        return new Simple3BallAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
      case THREE_BALL_DRIVE_AND_SHOOT:
        return new ThreeballDriveAndShoot(drivetrain, vision, shooter, intake, hopper, indexer, climber);
      case LEFT_TERMINAL_3_BALL: 
        return new LeftTerminal3Cargo(drivetrain, vision, shooter, intake, indexer, hopper, climber);
      case LEFT_2_BALL_1_DEFENSE:
        return new LeftDefenseAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
      case MIDDLE_RIGHT_TERMINAL_3_BALL:
        return new MiddleRightTerminal3CargoAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
      case MIDDLE_LEFT_TERMINAL_DEFENSE:
        return new MiddleLeftTerminalDefenseAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
      case MIDDLE_LEFT_3_BALL_TERMINAL_DEFENSE:
        return new MiddleLeft3BallTerminalDefenseAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
      case FIVE_BALL:
        return new RightFiveBallAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
      case RIGHT_MIDDLE_5_BALL_1_DEFENSE:
        return new MiddleRight5BallDefenseAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
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
    if (drivetrain != null) {
      drivetrain.putToSmartDashboard();
    }
    if (vision != null) {
      vision.putToSmartDashboard();
    }
    if (intake != null) {
      intake.putToSmartDashboard();
    }
    if (climber != null) {
      climber.putToSmartDashboard();
    }
    if (shooter != null) {
      shooter.putToSmartDashboard();
      
      // For Testing Velocity Calculations
      double reqProjectileVelocity = ProjectileCalculator.calculateReqProjectileVelocity(vision.getXDistanceToUpperHub());
      SmartDashboard.putNumber("Required Projectile Velocity", reqProjectileVelocity);
      SmartDashboard.putNumber("Required Angular Velocity", reqProjectileVelocity / Constants.ShooterSub.FLYWHEEL_RADIUS_METERS);
      SmartDashboard.putNumber("Required RPM", ProjectileCalculator.calculateReqShooterRPM(reqProjectileVelocity));
    }
  }

  public void resetGyro() {
    drivetrain.zeroGyroscope();
  }

  public void resetRobot() {
    drivetrain.stop();
  }
}
