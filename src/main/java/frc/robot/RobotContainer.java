// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.auto.testing.AutoTesting;
import frc.robot.auto.testing.Left2Ball2DefenseAuto;
import frc.robot.auto.tuned.LeftDefenseAuto;
import frc.robot.auto.testing.LeftTerminal3Cargo;
import frc.robot.auto.tuned.MiddleLeft3BallTerminalDefenseAuto;
import frc.robot.auto.tuned.MiddleLeft4BallTerminalDefenseAuto;
import frc.robot.auto.testing.MiddleRight5BallDefenseAuto;
import frc.robot.auto.testing.MiddleRightTerminal3CargoAuto;
import frc.robot.auto.tuned.OneBallAuto;
import frc.robot.auto.tuned.RightFiveBallAuto;
import frc.robot.auto.tuned.Simple3BallAuto;
import frc.robot.auto.testing.ThreeballDriveAndShoot;

import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.drive.ProfiledPIDTurnInPlaceCommand;
import frc.robot.commands.drive.VisionDriveCommand;
import frc.robot.commands.climber.ExtendClimberCommand;
import frc.robot.commands.climber.RetractClimberCommand;
import frc.robot.commands.climber.ToggleClimberSolenoidCommand;
import frc.robot.commands.climber.ZeroClimberEncoderCommand;
import frc.robot.commands.climber.autoclimbs.HighBarAutoClimbCommand;
import frc.robot.commands.climber.autoclimbs.MidBarAutoClimbCommand;
import frc.robot.commands.intake.IntakeArmToggleCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeReverseCommand;
import frc.robot.commands.intake.OnlyIntakeCommand;
import frc.robot.commands.shooter.TuneShooterCommand;
import frc.robot.commands.shooter.NonVisionShootCommand.NonVisionShootMode;
import frc.robot.commands.shooter.ShootCommand.ShootMode;
import frc.robot.commands.shooter.NonVisionShootCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.ShootLowCommand;
import frc.robot.subsystems.DashboardControlsSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DashboardControlsSubsystem.ButtonBindingsProfile;
import frc.robot.subsystems.ShooterSubsystem.FiringAngle;
import frc.robot.util.ProjectileCalculator;
import frc.robot.util.buttonbindings.ButtonCommands;
import frc.robot.util.buttonbindings.profiles.Default;
import frc.robot.util.buttonbindings.profiles.SoloDriver;
import frc.robot.util.vision.VisionCalculator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  private Joystick driveJoystick = new Joystick(0);
  private Joystick turnJoystick = new Joystick(1);
  private Joystick secondaryPannel = new Joystick(2);
  
  private JoystickButton resetGyroButton;

  private JoystickButton intakeArmToggleButton;
  private JoystickButton intakeInButton;
  private JoystickButton intakeReverseButton;

  private JoystickButton shootSingleButton;
  private JoystickButton shootAllButton;
  private JoystickButton shootLowGoalButton;
  private JoystickButton limelightLineupNonvisionShootButton;
  private JoystickButton nonVisionShootAllButton;
  private JoystickButton extendClimberButton;
  private JoystickButton retractClimberButton;
  private JoystickButton climberSolenoidToggleButton;
  private JoystickButton climberLockButton;
  private JoystickButton climberUnlockButton;
  private JoystickButton tuneShooterButton;
  private JoystickButton climberOverrideButton;
  private JoystickButton commandInterruptButton;

  private JoystickButton pidTestingButton;

  // Slew rate limiters to make joystick inputs more gentle.
  // A value of .1 will requier 10 seconds to get from 0 to 1. It is calculated as 1/rateLimitPerSecond to go from 0 to 1
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter turnLimiter = new SlewRateLimiter(2);

  private boolean initComplete = false;

  private Command autonomousCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // pixySub.setDefaultCommand(new PixyCamManualDriveCommand(pixySub));

    vision = new VisionSubsystem();
    dashboardControlsSubsystem = new DashboardControlsSubsystem(vision, this);
    dashboardControlsSubsystem.addSelectorsToSmartDashboard();
    
    LEDSubsystem.getInstance();

    drivetrain = new DrivetrainSubsystem();

    drivetrain.setDefaultCommand(
      new DefaultDriveCommand(
        drivetrain,
        () -> -modifyAxis(driveJoystick.getY(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driveJoystick.getX(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(turnJoystick.getX(), turnLimiter) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        dashboardControlsSubsystem
		  )
    );
  }

  public void init() {
    if (!initComplete) {
        initComplete = true;

    //intakeCamera = new UsbCameraSubsystem();  

    // //The following subsystems have a dependency on CAN

    shooter = new ShooterSubsystem();
    indexer = new IndexerSubsystem();
    intake = new IntakeSubsystem();
    hopper = new HopperSubsystem();
    pneumatics = new PneumaticsSubsystem();
    climber = new HookClimberSubsystem();
    //LEDSubsystem.getInstance();

    configureButtonBindings();
    }
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
    limelightLineupNonvisionShootButton = new JoystickButton(driveJoystick, 10);
    nonVisionShootAllButton = new JoystickButton(driveJoystick, 4);
    
    // Climber Buttons Bindings
    extendClimberButton = new JoystickButton(secondaryPannel, 5);
    retractClimberButton = new JoystickButton(secondaryPannel, 3);
    climberSolenoidToggleButton = new JoystickButton(secondaryPannel, 4);
    climberUnlockButton = new JoystickButton(secondaryPannel, 11);
    climberLockButton = new JoystickButton(secondaryPannel, 12);
    climberOverrideButton = new JoystickButton(secondaryPannel, 8);
    commandInterruptButton = new JoystickButton(secondaryPannel, 10); // TEMP BUTTON PORT

    pidTestingButton = new JoystickButton(secondaryPannel, 2);

    ButtonCommands.SHOOT_SINGLE.setCommand(
      new ParallelCommandGroup(
        new ShootCommand(ShootMode.SHOOT_SINGLE, shooter, indexer, hopper, vision),
        new VisionDriveCommand( // Overrides the DefualtDriveCommand and uses VisionDriveCommand when the trigger on the turnJoystick is held.
          drivetrain,
          () -> -modifyAxis(driveJoystick.getY(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
          () -> -modifyAxis(driveJoystick.getX(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
          vision,
          dashboardControlsSubsystem
      )
    ));
    ButtonCommands.VISION_SHOOT_ALL.setCommand(new ParallelCommandGroup(
        // new ShootCommand(ShootMode.SHOOT_ALL, shooter, indexer, hopper, vision),
        new ConditionalCommand(
          new NonVisionShootCommand(NonVisionShootMode.SHOOT_ALL, shooter, indexer, hopper, FiringAngle.ANGLE_1, VisionCalculator.getInstance().getShooterConfig(3 * 12, FiringAngle.ANGLE_1)), 
          new ShootCommand(ShootMode.SHOOT_ALL, shooter, indexer, hopper, vision), 
          dashboardControlsSubsystem::getIsLimelightDead
        ),
        new VisionDriveCommand( // Overrides the DefualtDriveCommand and uses VisionDriveCommand when the trigger on the turnJoystick is held.
          drivetrain,
          () -> -modifyAxis(driveJoystick.getY(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
          () -> -modifyAxis(driveJoystick.getX(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
          vision,
          dashboardControlsSubsystem
        )
      ));

    ButtonCommands.RESET_GYRO.setCommand(new InstantCommand(() -> { this.resetGyro(); }));

    ButtonCommands.INTAKE_TOGGLE.setCommand(new IntakeArmToggleCommand(intake, indexer, hopper));
    ButtonCommands.INTAKE_ON.setCommand(
      new ConditionalCommand( // Makes sure the intake doesn't stop the shooter because both of them require the hopper, by checking if shoot button is pressed and using the OnlyIntakeCommand instead if it is.
        new OnlyIntakeCommand(intake, indexer), 
        new IntakeInCommand(intake, indexer, hopper), 
        shootAllButton::get
      )
    );
    ButtonCommands.INTAKE_REVERSE.setCommand(new IntakeReverseCommand(intake, hopper));

    ButtonCommands.TUNE_SHOOTER.setCommand(new TuneShooterCommand(shooter, indexer, intake, hopper));
    ButtonCommands.SHOOT_LOW_GOAL.setCommand(new ShootLowCommand(shooter, indexer));
    ButtonCommands.LINEUP_SHOOT.setCommand(new NonVisionShootCommand(NonVisionShootMode.SHOOT_ALL, shooter, indexer, hopper, FiringAngle.ANGLE_2, VisionCalculator.getInstance().getShooterConfig(7 * 12, FiringAngle.ANGLE_2)));

    ButtonCommands.NON_VISION_SHOOT_ALL.setCommand(
      new NonVisionShootCommand(
        NonVisionShootMode.SHOOT_ALL, 
        shooter, 
        indexer, 
        hopper,
        FiringAngle.ANGLE_1,
        VisionCalculator.getInstance().getShooterConfig(3*12, FiringAngle.ANGLE_1)
    ));

    ButtonCommands.CLIMBER_EXTEND.setCommand(new ExtendClimberCommand(climber, () -> climberOverrideButton.get()));
    ButtonCommands.CLIMBER_RETRACT.setCommand(new RetractClimberCommand(climber, () -> climberOverrideButton.get()));
    ButtonCommands.TOGGLE_CLIMBER_ANGLE.setCommand(new ToggleClimberSolenoidCommand(climber));
    ButtonCommands.UNLOCK_CLIMBER.setCommand(new InstantCommand(() -> { climber.unlock(); }));
    ButtonCommands.LOCK_CLIMBER.setCommand(new InstantCommand(() -> { climber.lock(); }));

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
    intakeArmToggleButton.whenPressed(ButtonCommands.INTAKE_TOGGLE.command);
    intakeInButton.whileHeld(ButtonCommands.INTAKE_ON.command);
    intakeReverseButton.whileHeld(ButtonCommands.INTAKE_REVERSE.command);

    // Shooter Button Command Bindings
    shootSingleButton.whileHeld(ButtonCommands.SHOOT_SINGLE.command);
    shootAllButton.whileHeld(ButtonCommands.VISION_SHOOT_ALL.command);
    tuneShooterButton.whileHeld(ButtonCommands.TUNE_SHOOTER.command);

    shootLowGoalButton.whileHeld(ButtonCommands.SHOOT_LOW_GOAL.command);

    limelightLineupNonvisionShootButton.whileHeld(ButtonCommands.LINEUP_SHOOT.command); // Button for lining up target in Limelight's crosshair and shooting without any vision calculation

    nonVisionShootAllButton.whileHeld(ButtonCommands.NON_VISION_SHOOT_ALL.command);
      
    
    // Climber Button Command Bindings
    extendClimberButton.whileHeld(ButtonCommands.CLIMBER_EXTEND.command);
    retractClimberButton.whileHeld(ButtonCommands.CLIMBER_RETRACT.command);
    climberSolenoidToggleButton.whenPressed(ButtonCommands.TOGGLE_CLIMBER_ANGLE.command);
    climberUnlockButton.whenPressed(ButtonCommands.UNLOCK_CLIMBER.command);
    climberLockButton.whenPressed(ButtonCommands.LOCK_CLIMBER.command);

    // SmartDashboard Command Buttons
    SmartDashboard.putData("Zero Climber Encoder", new ZeroClimberEncoderCommand(climber));
    SmartDashboard.putData("Zero Gyroscope", new InstantCommand(() -> this.resetGyro()));
    SmartDashboard.putData("Mid Bar Auto Climb", new MidBarAutoClimbCommand(climber).withInterrupt(() -> commandInterruptButton.get()));
    SmartDashboard.putData("Mid Bar Auto Climb", new HighBarAutoClimbCommand(climber, drivetrain).withInterrupt(() -> commandInterruptButton.get()));

    // TODO: Delete this when done
    //pidTestingButton.whenPressed(new ProfiledPIDTurnInPlaceCommand(drivetrain, () -> { return Rotation2d.fromDegrees(180); }));

  }

  public void assignButtonBindings(ButtonBindingsProfile buttonBindingsProfile) {
    driveJoystick = new Joystick(0);
    turnJoystick = new Joystick(1);
    secondaryPannel = new Joystick(2);

    switch(buttonBindingsProfile) {
      case DEFAULT:
        Default.configureButtons(driveJoystick, turnJoystick, secondaryPannel);
        break;
      case SOLO_DRIVER:
        SoloDriver.configureButtons(driveJoystick, turnJoystick, secondaryPannel);
        break;
      default:
        System.err.println("REASSIGN BUTTON BINDINGS FELL THROUGH");
        configureButtonBindings();
        break;
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonomousCommand;
  }

  public void initializeAutonomousCommand() {
    switch(dashboardControlsSubsystem.getSelectedAuto()) {
      case AUTO_TESTING:
        autonomousCommand = new AutoTesting(drivetrain, vision, shooter, intake, hopper, indexer, climber);
        break;
      case ONE_BALL:
        autonomousCommand = new OneBallAuto(drivetrain, shooter, indexer, hopper, climber);
        break;
      case SIMPLE_3_BALL:
        autonomousCommand = new Simple3BallAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
        break;
      case LEFT_2_BALL_1_DEFENSE:
        autonomousCommand = new LeftDefenseAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
        break;
      case MIDDLE_LEFT_3_BALL_TERMINAL_DEFENSE:
        autonomousCommand = new MiddleLeft3BallTerminalDefenseAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
        break;
      case MIDDLE_RIGHT_TERMINAL_4_BALL:
        autonomousCommand = new MiddleLeft4BallTerminalDefenseAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
        break;
      case RIGHT_FIVE_BALL:
        autonomousCommand = new RightFiveBallAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
        break;
      case LEFT_2_BALL_2_DEFENSE:
        autonomousCommand = new Left2Ball2DefenseAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
        break;
      case RIGHT_MIDDLE_5_BALL_1_DEFENSE:
        autonomousCommand = new MiddleRight5BallDefenseAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
        break;
    //   case THREE_BALL_DRIVE_AND_SHOOT:
    //     return new ThreeballDriveAndShoot(drivetrain, vision, shooter, intake, hopper, indexer, climber);
    //   case LEFT_TERMINAL_3_BALL: 
    //     return new LeftTerminal3Cargo(drivetrain, vision, shooter, intake, indexer, hopper, climber);
    //   case MIDDLE_RIGHT_TERMINAL_3_BALL:
    //     return new MiddleRightTerminal3CargoAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
    //   case MIDDLE_LEFT_TERMINAL_DEFENSE:
    //     return new MiddleLeftTerminalDefenseAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
      default:
        System.err.println("NO VALID AUTO SELECTED");
        break;
    }
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

  public void resetGyro() {
    drivetrain.zeroGyroscope();
  }

  public void resetRobot() {
    drivetrain.stop();
  }
}
